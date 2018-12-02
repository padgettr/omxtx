/* openMAX transcoder for raspberry pi
 * omxtx.c
 *
 * (c) 2012, 2013 Dickon Hood <dickon@fluff.org>
 * Timing fixups by Adam Charrett <adam@dvbstreamer.org>
 *
 * A trivial OpenMAX transcoder for the Pi.
 *
 * Very much a work-in-progress, and as such is noisy, doesn't produce
 * particularly pretty output, and is probably buggier than a swamp in
 * summer.  Beware of memory leaks.
 *
 * Usage: type ./omxtx after make for usage / options
 *
 * This version has been substantially changed to add new functionality, and
 * compatibility with ffmpeg >= 3.2.2
 * 
 *
 * Dr. R. Padgett <rod_padgett@hotmail.com> May 2016
 *
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

/* NOTES: rpi is 32 bit: int and long, int32 are all 4 byte, long long and int64 are 8 byte
 *
 */

//#define DEBUG

#define VERSION 0.1
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "bcm_host.h"
#include "libavformat/avformat.h"
#include "libavutil/avutil.h"
#include "libavutil/mathematics.h"
#include "libavformat/avio.h"
#include <error.h>

#include "OMX_Video.h"
#include "OMX_Types.h"
#include "OMX_Component.h"
#include "OMX_Core.h"
#include "OMX_Broadcom.h"

#include <pthread.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/queue.h>
#include <fcntl.h>

#include <time.h>
#include <errno.h>

#include <unistd.h>
#include <signal.h>

/* Defined in OMX_Types.h */
static OMX_VERSIONTYPE SpecificationVersion = {
   .s.nVersionMajor = 1,   /* OMX_VERSION_MAJOR */
   .s.nVersionMinor = 1,   /* OMX_VERSION_MINOR */
   .s.nRevision     = 2,   /* OMX_VERSION_REVISION */
   .s.nStep         = 0    /* OMX_VERSION_STEP */
};

/* Hateful things: */
#define MAKEME(y, x)   do { \
            y = calloc(1, sizeof(x)); \
            y->nSize = sizeof(x); \
            y->nVersion = SpecificationVersion; \
         } while (0)


#define OERR(cmd)   do { \
            OMX_ERRORTYPE oerr = cmd; \
            if (oerr != OMX_ErrorNone) { \
               fprintf(stderr, #cmd " failed on line %d: %x\n", __LINE__, oerr); \
               exit(1); \
            } \
         } while (0)
/* ... but damn useful.*/

/* Hardware component names: */
#define ENCNAME "OMX.broadcom.video_encode"
#define DECNAME "OMX.broadcom.video_decode"
#define RSZNAME "OMX.broadcom.resize"
#define VIDNAME "OMX.broadcom.video_render"
#define SPLNAME "OMX.broadcom.video_splitter"
#define DEINAME "OMX.broadcom.image_fx"

/*
 * Portbase for the modules, could also be queried, but as the components
 * are broadcom/raspberry specific anyway...
 * These are input ports; output is always PORT+1
 */
#define PORT_RSZ  60   /* Image resize */
#define PORT_VID  90   /* Video render */
#define PORT_DEC 130   /* Video decode */
#define PORT_DEI 190   /* Image FX */
#define PORT_ENC 200   /* Video encode */
#define PORT_SPL 250   /* Video splitter: output on ports 251 - 254 */

/* Process states, set in context struct field state */
enum states {
   DECINIT,       /* Decoder: initialising */
   TUNNELSETUP,   /* Decoder initialised: Ready for tunnel setup */
   OPENOUTPUT,    /* Tunnel setup finished: ready for output file open */
   DECFAILED,
   RUNNING,       /* Encoder is running */
   DECEOF,        /* End of input file */
   ENCEOS,        /* Encoder: end of stream */
   QUIT,          /* User terminated the process with SIGINT (ctrl-c) */
};

/* Double linked list of non-video packets saved during decoder initialisation
 * See man 3 tailq_entry for details of tailq
 */
struct packetentry {
   TAILQ_ENTRY(packetentry) link;
   AVPacket *packet;
};
TAILQ_HEAD(packetqueue, packetentry);
static struct packetqueue packetq;

typedef struct {
   uint8_t *nalBuf;
   uint32_t nalBufSize; /* 32 bit: maximum buffer size 4GB! */
   off_t nalBufOffset;
   int64_t tick;
   int64_t pts;
   int64_t duration;
   AVRational fps;
} OMXTX_NAL_ENTRY;

static struct context {
   AVFormatContext *ic;    /* Input context for demuxer */
   AVFormatContext *oc;    /* Output context for muxer */
   OMXTX_NAL_ENTRY nalEntry;  /* Store for NAL header info */
   int      raw_fd;        /* File descriptor for raw output file */
   uint16_t userFlags;      /* User command line switch flags */
   uint64_t framesIn;
   volatile _Atomic uint64_t framesOut;
   volatile _Atomic uint64_t ptsDelta; /* Time difference in ms between output pts and omx tick */
   volatile _Atomic uint8_t componentFlags;
   volatile _Atomic int encBufferFilled;
   volatile _Atomic enum states state;
   OMX_BUFFERHEADERTYPE *encbufs;
   OMX_BUFFERHEADERTYPE *decbufs;
   volatile uint64_t encWaitTime;
   int      inVidStreamIdx;
   int      inAudioStreamIdx; /* <0 if there is no audio stream */
   int      userAudioStreamIdx;
   int64_t  audioPTS;      /* Input PTS */
   int64_t  videoPTS;      /* Input PTS */
   OMX_HANDLETYPE   dec, enc, rsz, dei, spl, vid;
   pthread_mutex_t decBufLock;
   AVBitStreamFilterContext *bsfc;
   int   bitrate;
   double omxFPS;          /* Output frame rate for display purposes */
   char  *iname;
   char  *oname;
   char  *oformat;
   AVRational omxtimebase; /* OMX time base is micro seconds */
   OMX_CONFIG_RECTTYPE *cropRect;
   int outputWidth;
   int outputHeight;
   int naluInputFormat;    /* 1 if input is h264 annexb, 0 otherwise */
   int minQuant;           /* Minimum allowed quantisation: lower is better quality but bigger file size */
   int maxQuant;           /* Maximum allowed quantisation */
   int interlaceMode;
   int dei_ofpf;           /* Deinterlacer: output 1 frame per field */
} ctx;

/* Command line option flags */
#define UFLAGS_VERBOSE       (uint16_t)(1U<<0)
#define UFLAGS_RESIZE        (uint16_t)(1U<<1)
#define UFLAGS_MONITOR       (uint16_t)(1U<<2)
#define UFLAGS_DEINTERLACE   (uint16_t)(1U<<3)
#define UFLAGS_RAW           (uint16_t)(1U<<4)
#define UFLAGS_SPARE         (uint16_t)(1U<<5)
#define UFLAGS_CROP          (uint16_t)(1U<<6)
#define UFLAGS_AUTO_SCALE_X  (uint16_t)(1U<<7)
#define UFLAGS_AUTO_SCALE_Y  (uint16_t)(1U<<8)
#define UFLAGS_MAKE_UP_PTS  (uint16_t)(1U<<9)

/* Component flags */
#define CFLAGS_RSZ       (uint8_t)(1U<<0)
#define CFLAGS_VID       (uint8_t)(1U<<1)
#define CFLAGS_DEC       (uint8_t)(1U<<2)
#define CFLAGS_DEI       (uint8_t)(1U<<3)
#define CFLAGS_ENC       (uint8_t)(1U<<4)
#define CFLAGS_SPL       (uint8_t)(1U<<5)

static OMX_BUFFERHEADERTYPE *allocbufs(OMX_HANDLETYPE h, int port);
static void requestStateChange(OMX_HANDLETYPE handle, enum OMX_STATETYPE rState, int wait);
static const char *mapComponent(struct context *ctx, OMX_HANDLETYPE handle);

/* Print some useful information about the state of the port: */
static void dumpport(OMX_HANDLETYPE handle, int port) {
   OMX_PARAM_PORTDEFINITIONTYPE   *portdef;
   MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
   portdef->nPortIndex = port;
   OERR(OMX_GetParameter(handle, OMX_IndexParamPortDefinition, portdef));
   fprintf(stderr, "%s port %d is %s, %s\n", mapComponent(&ctx, handle), portdef->nPortIndex,
      (portdef->eDir == 0 ? "input" : "output"),
      (portdef->bEnabled == 0 ? "disabled" : "enabled"));
   fprintf(stderr, "Wants %d bufs, needs %d, size %d, enabled: %d, pop: %d, aligned %d\n",
      portdef->nBufferCountActual,
      portdef->nBufferCountMin, portdef->nBufferSize,
      portdef->bEnabled, portdef->bPopulated,
      portdef->nBufferAlignment);

   switch (portdef->eDomain) {
   case OMX_PortDomainVideo:
      fprintf(stderr, "Video type is currently:\n"
         "\tMIME:\t\t%s\n"
         "\tNative:\t\t%p\n"
         "\tWidth:\t\t%d\n"
         "\tHeight:\t\t%d\n"
         "\tStride:\t\t%d\n"
         "\tSliceHeight:\t%d\n"
         "\tBitrate:\t%d\n"
         "\tFramerate:\t%d (%x); (%f)\n"
         "\tError hiding:\t%d\n"
         "\tCodec:\t\t%d\n"
         "\tColour:\t\t%d\n",
         portdef->format.video.cMIMEType,
         portdef->format.video.pNativeRender,
         portdef->format.video.nFrameWidth,
         portdef->format.video.nFrameHeight,
         portdef->format.video.nStride,
         portdef->format.video.nSliceHeight,
         portdef->format.video.nBitrate,
         portdef->format.video.xFramerate,
         portdef->format.video.xFramerate,
         ((float)portdef->format.video.xFramerate/(float)(1<<16)), /* Q16 format */
         portdef->format.video.bFlagErrorConcealment,
         portdef->format.video.eCompressionFormat,
         portdef->format.video.eColorFormat);
      break;
   case OMX_PortDomainImage:
      fprintf(stderr, "Image type is currently:\n"
         "\tMIME:\t\t%s\n"
         "\tNative:\t\t%p\n"
         "\tWidth:\t\t%d\n"
         "\tHeight:\t\t%d\n"
         "\tStride:\t\t%d\n"
         "\tSliceHeight:\t%d\n"
         "\tError hiding:\t%d\n"
         "\tCodec:\t\t%d\n"
         "\tColour:\t\t%d\n",
         portdef->format.image.cMIMEType,
         portdef->format.image.pNativeRender,
         portdef->format.image.nFrameWidth,
         portdef->format.image.nFrameHeight,
         portdef->format.image.nStride,
         portdef->format.image.nSliceHeight,
         portdef->format.image.bFlagErrorConcealment,
         portdef->format.image.eCompressionFormat,
         portdef->format.image.eColorFormat);
      break;
/* Feel free to add others. */
   default:
      fprintf(stderr, "This port is not defined in this program!\n");
      break;
   }
   free(portdef);
}

/* OMX_FreeBuffer to be called:
 * while the component is in the OMX_StateIdle state and the IL client has
 * already sent a request for the state transition to OMX_StateLoaded (e.g.,
 * during the stopping of the component)
 * i.e. request loaded state, but don't wait for the transition.
 */
static void freeBuffers(OMX_HANDLETYPE h, int port, OMX_BUFFERHEADERTYPE *omxBufs) {
   int i;
   OMX_BUFFERHEADERTYPE *buf;
   OMX_PARAM_PORTDEFINITIONTYPE *portdef;

   MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
   portdef->nPortIndex = port;
   OERR(OMX_GetParameter(h, OMX_IndexParamPortDefinition, portdef));
   buf=omxBufs;
   for (i = 0; i < portdef->nBufferCountActual; i++) {
      OERR(OMX_FreeBuffer(h, port, buf));
      buf = buf->pAppPrivate;
   }
   free(portdef);
}

/* Free all buffers:
 * Transition component to idle and wait for transition
 * Then request transition to loaded, but don't wait
 * Free the client assigned buffers
 * (tunnelled components will free buffers automatically on request for state loaded)
 * Wait for the transition to loaded for each component
 * Free handles
 * Call OMX_Deinit()
 */
static void cleanup(struct context *ctx) {

   requestStateChange(ctx->dec, OMX_StateIdle, 1);
   if (ctx->userFlags & UFLAGS_DEINTERLACE)
      requestStateChange(ctx->dei, OMX_StateIdle, 1);
   if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP)
      requestStateChange(ctx->rsz, OMX_StateIdle, 1);
   if (ctx->userFlags & UFLAGS_MONITOR) {
      requestStateChange(ctx->spl, OMX_StateIdle, 1);
      requestStateChange(ctx->vid, OMX_StateIdle, 1);
   }
   requestStateChange(ctx->enc, OMX_StateIdle, 1);

   requestStateChange(ctx->dec, OMX_StateLoaded, 0);
   if (ctx->userFlags & UFLAGS_DEINTERLACE)
      requestStateChange(ctx->dei, OMX_StateLoaded, 0);
   if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP)
      requestStateChange(ctx->rsz, OMX_StateLoaded, 0);
   if (ctx->userFlags & UFLAGS_MONITOR) {
      requestStateChange(ctx->spl, OMX_StateLoaded, 0);
      requestStateChange(ctx->vid, OMX_StateLoaded, 0);
   }
   requestStateChange(ctx->enc, OMX_StateLoaded, 0);
   freeBuffers(ctx->dec, PORT_DEC, ctx->decbufs);
   freeBuffers(ctx->enc, PORT_ENC+1, ctx->encbufs);

   /* Wait for state changes to loaded state after all buffers are de-allocated
    * Since handles were obtained for all components, unused ones will
    * already be in the loaded state.
    */
   requestStateChange(ctx->enc, OMX_StateLoaded, 2);
   requestStateChange(ctx->vid, OMX_StateLoaded, 2);
   requestStateChange(ctx->spl, OMX_StateLoaded, 2);
   requestStateChange(ctx->rsz, OMX_StateLoaded, 2);
   requestStateChange(ctx->dei, OMX_StateLoaded, 2);
   requestStateChange(ctx->dec, OMX_StateLoaded, 2);

   /* OMX_TeardownTunnel not defined on rpi */

   OERR(OMX_FreeHandle(ctx->dec));
   OERR(OMX_FreeHandle(ctx->dei));
   OERR(OMX_FreeHandle(ctx->rsz));
   OERR(OMX_FreeHandle(ctx->spl));
   OERR(OMX_FreeHandle(ctx->vid));
   OERR(OMX_FreeHandle(ctx->enc));
   OERR(OMX_Deinit());
}

static void exitHandler(void) {
   enum OMX_STATETYPE state;

   fprintf(stderr, "\n\nIn exit handler, after %lli frames:\n", ctx.framesOut);
   if (ctx.userFlags & UFLAGS_VERBOSE) {
      dumpport(ctx.dec, PORT_DEC);
      dumpport(ctx.dec, PORT_DEC+1);
      dumpport(ctx.enc, PORT_ENC+1);
   }

   OMX_GetState(ctx.dec, &state);
   fprintf(stderr, "Decoder state: %d\n", state);
   OMX_GetState(ctx.enc, &state);
   fprintf(stderr, "Encoder state: %d\n", state);
   cleanup(&ctx);
}

static int mapCodec(enum AVCodecID id) {
   fprintf(stderr, "Mapping codec ID %d (%x)\n", id, id);
   switch (id) {
      case AV_CODEC_ID_MPEG2VIDEO:
         return OMX_VIDEO_CodingMPEG2;
      case AV_CODEC_ID_H264:
         return OMX_VIDEO_CodingAVC;
      case AV_CODEC_ID_VP8:
         return OMX_VIDEO_CodingVP8;
      case AV_CODEC_ID_MJPEG:
         return OMX_VIDEO_CodingMJPEG;
      case AV_CODEC_ID_MPEG4:
         return OMX_VIDEO_CodingMPEG4;
      default:
         return -1;
   }
}

static int mapProfile(enum OMX_VIDEO_AVCPROFILETYPE id) {
   fprintf(stderr, "Mapping profile ID %d (%x)\n", id, id);
   switch (id) {
      case OMX_VIDEO_AVCProfileBaseline:
         return FF_PROFILE_H264_BASELINE;
      case OMX_VIDEO_AVCProfileMain:
         return FF_PROFILE_H264_MAIN;
      case OMX_VIDEO_AVCProfileExtended:
         return FF_PROFILE_H264_EXTENDED;
      case OMX_VIDEO_AVCProfileHigh:
         return FF_PROFILE_H264_HIGH;
      case OMX_VIDEO_AVCProfileHigh10:
         return FF_PROFILE_H264_HIGH_10;
      case OMX_VIDEO_AVCProfileHigh422:
         return FF_PROFILE_H264_HIGH_422;
      case OMX_VIDEO_AVCProfileHigh444:
         return FF_PROFILE_H264_HIGH_444;
      case OMX_VIDEO_AVCProfileConstrainedBaseline:
         return FF_PROFILE_H264_CONSTRAINED_BASELINE;
      default:
         return FF_PROFILE_UNKNOWN;
   }
}

static int mapLevel(enum OMX_VIDEO_AVCLEVELTYPE id) {
   fprintf(stderr, "Mapping level ID %d (%x)\n", id, id);
   switch (id) {
      case OMX_VIDEO_AVCLevel1:
         return 10;
      case OMX_VIDEO_AVCLevel1b:
         return 11;
      case OMX_VIDEO_AVCLevel11:
         return 11;
      case OMX_VIDEO_AVCLevel12:
         return 12;
      case OMX_VIDEO_AVCLevel13:
         return 13;
      case OMX_VIDEO_AVCLevel2:
         return 20;
      case OMX_VIDEO_AVCLevel21:
         return 21;
      case OMX_VIDEO_AVCLevel22:
         return 22;
      case OMX_VIDEO_AVCLevel3:
         return 30;
      case OMX_VIDEO_AVCLevel31:
         return 31;
      case OMX_VIDEO_AVCLevel32:
         return 32;
      case OMX_VIDEO_AVCLevel4:
         return 40;
      case OMX_VIDEO_AVCLevel41:
         return 41;
      case OMX_VIDEO_AVCLevel42:
         return 42;
      case OMX_VIDEO_AVCLevel5:
         return 50;
      case OMX_VIDEO_AVCLevel51:
         return 51;
      default:
         return FF_LEVEL_UNKNOWN;
   }
}

static int mapColour(enum OMX_COLOR_FORMATTYPE id) {
   fprintf(stderr, "Mapping colour ID %d (%x)\n", id, id);
   switch (id) {
      case OMX_COLOR_FormatYUV420PackedPlanar:
         return AV_PIX_FMT_YUV420P;
      case OMX_COLOR_FormatYUV420PackedSemiPlanar:
         return AV_PIX_FMT_NONE;       /* Can't match this up */
      case OMX_COLOR_Format16bitRGB565:
         return AV_PIX_FMT_RGB565LE;   /* This is little endian; AV_PIX_FMT_RGB565BE for big endian */
      case OMX_COLOR_Format24bitBGR888:
         return AV_PIX_FMT_BGR24;
      case OMX_COLOR_Format32bitABGR8888:
         return AV_PIX_FMT_ABGR;
      default:
         return AV_PIX_FMT_NONE;
   }
}

static const char *mapEvents(OMX_EVENTTYPE event) {
   switch (event) {
      case OMX_EventCmdComplete:
         return "OMX_EventCmdComplete"; /* component has sucessfully completed a command */
      break;
      case OMX_EventError:
         return "OMX_EventError"; /* component has detected an error condition */
      break;
      case OMX_EventMark:
         return "OMX_EventMark"; /* component has detected a buffer mark */
      break;
      case OMX_EventPortSettingsChanged:
         return "OMX_EventPortSettingsChanged"; /* component is reported a port settings change */
      break;
      case OMX_EventBufferFlag:
         return "OMX_EventBufferFlag"; /* component has detected an EOS */ 
      break;
      case OMX_EventResourcesAcquired:
         return "OMX_EventResourcesAcquired"; /* component has been granted resources and is automatically starting the state change from OMX_StateWaitForResources to OMX_StateIdle. */
      break;
      case OMX_EventComponentResumed:
         return "OMX_EventComponentResumed"; /**< Component resumed due to reacquisition of resources */
      break;
      case OMX_EventDynamicResourcesAvailable:
         return "OMX_EventDynamicResourcesAvailable"; /**< Component has acquired previously unavailable dynamic resources */
      break;
      case OMX_EventPortFormatDetected:
         return "OMX_EventPortFormatDetected"; /**< Component has detected a supported format. */
      break;
      case OMX_EventParamOrConfigChanged:
         return "OMX_EventParamOrConfigChanged"; 
      break;
      default:
         return "Unknown";
   }
}

/* Map OMX handle to component type, used for info in printf statements only */
static const char *mapComponent(struct context *ctx, OMX_HANDLETYPE handle) {
   if (handle == ctx->dec)
      return "Decoder";
   if (handle == ctx->enc)
      return "Encoder";
   if (handle == ctx->rsz)
      return "Resizer";
   if (handle == ctx->dei)
      return "Deinterlacer";
   if (handle == ctx->spl)
      return "Splitter";
   if (handle == ctx->vid)
      return "Render";
   return "Unknown";
}

/* oname is output filename, ctx->oname, idx is video stream index ctx->inVidStreamIdx
 * ic - input AVFormatContext; allocated by avformat_open_input() on input file open
 */
static AVFormatContext *makeOutputContext(AVFormatContext *ic, const char *oname, int idx, const OMX_PARAM_PORTDEFINITIONTYPE *prt, OMX_VIDEO_PARAM_PROFILELEVELTYPE *level) {
   const OMX_VIDEO_PORTDEFINITIONTYPE *viddef;
   AVFormatContext   *oc=NULL;
   AVStream          *iflow, *oflow;
   AVOutputFormat    *fmt;

   viddef = &prt->format.video; /* Decoder output format structure */

   /* allocate avformat context - avformat_free_context() can be used to free */

   if (ctx.oformat!=NULL)
      fmt = av_guess_format(ctx.oformat, NULL, NULL);

   avformat_alloc_output_context2(&oc, fmt, NULL, oname);
   if (!oc) {
      fprintf(stderr, "Failed to alloc outputcontext\n");
      exit(1);
   }

   iflow = ic->streams[ctx.inVidStreamIdx];
   oflow = avformat_new_stream(oc, NULL); /* Stream 0 */

   
   if (!oflow) {
      av_log(NULL, AV_LOG_ERROR, "Failed allocating output stream\n");
      exit(1);
   }
   oflow->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
   oflow->codecpar->codec_id = AV_CODEC_ID_H264;
      
   oflow->codecpar->width = viddef->nFrameWidth;   /* Set  AVCodecContext details to OMX_VIDEO reported values */
   oflow->codecpar->height = viddef->nFrameHeight;
   oflow->codecpar->bit_rate = ctx.bitrate;        /* User specified bit rate or default */
   oflow->codecpar->profile = mapProfile(level->eProfile);
   oflow->codecpar->level = mapLevel(level->eLevel);

   oflow->time_base = ctx.omxtimebase;             /* Set timebase hint for muxer: will be overwritten on header write depending on container format */
   oflow->codecpar->format = mapColour(viddef->eColorFormat);
   oflow->avg_frame_rate = ctx.nalEntry.fps;
   oflow->r_frame_rate = ctx.nalEntry.fps;
   oflow->start_time=0;

   if (ctx.userFlags & UFLAGS_RESIZE) {
      if ((ctx.userFlags & UFLAGS_AUTO_SCALE_X) || (ctx.userFlags & UFLAGS_AUTO_SCALE_Y)) {
         oflow->codecpar->sample_aspect_ratio.num = 1;
         oflow->codecpar->sample_aspect_ratio.den = 1;
         oflow->sample_aspect_ratio.num = 1;
         oflow->sample_aspect_ratio.den = 1;
      }
      else {   /* Set this to 'unknown' for now */
         oflow->codecpar->sample_aspect_ratio.num = 0;
         oflow->codecpar->sample_aspect_ratio.den = 1;
         oflow->sample_aspect_ratio.num = 0;
         oflow->sample_aspect_ratio.den = 1;
      }
   }
   else {   /* No resize requested: copy apect ratio from input */
      oflow->codecpar->sample_aspect_ratio.num = iflow->codecpar->sample_aspect_ratio.num;
      oflow->codecpar->sample_aspect_ratio.den = iflow->codecpar->sample_aspect_ratio.den;
      oflow->sample_aspect_ratio.num = iflow->codecpar->sample_aspect_ratio.num;
      oflow->sample_aspect_ratio.den = iflow->codecpar->sample_aspect_ratio.den;
   }

   fprintf(stderr, "\n\n\n");
   if (ctx.inAudioStreamIdx>0) {
      fprintf(stderr, "*** Mapping input video stream #%i to output video stream #%i ***\n", ctx.inVidStreamIdx, 0);
      fprintf(stderr, "*** Mapping input audio stream #%i to output audio stream #%i ***\n\n", ctx.inAudioStreamIdx, 1);
      iflow = ic->streams[ctx.inAudioStreamIdx];
      oflow = avformat_new_stream(oc, NULL); /* Stream 1 */
      if (avcodec_parameters_copy(oflow->codecpar, iflow->codecpar) < 0) /* This copies extradata */
         fprintf(stderr,"ERROR: Copying parameters for audio stream failed.\n");
      oflow->codecpar->codec_tag=0; /* Don't copy FOURCC: Causes problems when remuxing */
      oflow->start_time=0;
      oflow->time_base = iflow->time_base; /* Time base hint */
   }

   fprintf(stderr, "\nOutput:\n");
   av_dump_format(oc, 0, oname, 1);

   return oc;
}

static void writeAudioPacket(AVPacket *pkt) {
   int ret;
   pkt->stream_index=1;
   
   if (! (ctx.userFlags & UFLAGS_MAKE_UP_PTS) && pkt->dts > ctx.audioPTS)
      ctx.audioPTS=pkt->dts;
   else
      ctx.audioPTS+=pkt->duration; /* Use packet duration */

   pkt->duration = av_rescale_q(pkt->duration,
              ctx.ic->streams[ctx.inAudioStreamIdx]->time_base,
              ctx.oc->streams[1]->time_base);
   pkt->pts = av_rescale_q(ctx.audioPTS,
              ctx.ic->streams[ctx.inAudioStreamIdx]->time_base,
              ctx.oc->streams[1]->time_base);
   pkt->dts=pkt->pts; /* Audio packet: dts=pts */

   // fprintf(stderr,"Audio PTS %lld\n", pkt->pts);
   ret=av_interleaved_write_frame(ctx.oc, pkt);   /* This frees pkt */
   if (ret < 0) {
      fprintf(stderr, "ERROR:omxtx: Failed to write audio frame.\n");
   }
}

static int openOutput(struct context *ctx) {
   int i, ret;
   struct packetentry *packet, *next;

   fprintf(stderr,"Got SPS and PPS data: opening output file '%s'\n", ctx->oname);

   if (!(ctx->oc->oformat->flags & AVFMT_NOFILE)) {
     ret = avio_open(&ctx->oc->pb, ctx->oname, AVIO_FLAG_WRITE);
     if (ret < 0) {
         fprintf(stderr, "Could not open output file '%s'\n", ctx->oname);
         exit(1);
     }
   }
   /* init muxer, write output file header */
   ret = avformat_write_header(ctx->oc, NULL);
   if (ret < 0) {
     av_log(NULL, AV_LOG_ERROR, "Error occurred when opening output file\n");
     exit(1);
   }

   if (ctx->inAudioStreamIdx>0) {
      fprintf(stderr, "Writing saved audio packets out...");
      for (i = 0, packet = TAILQ_FIRST(&packetq); packet; packet = next) {
         next = TAILQ_NEXT(packet, link);
         writeAudioPacket(packet->packet);
         i++;
         TAILQ_REMOVE(&packetq, packet, link);
         av_packet_free(&packet->packet);
      }

      fprintf(stderr, "done.  Wrote %d frames.\n\n", i);
   }

   fprintf(stderr, "Press ctrl-c to abort\n");
   return 0;
}

OMX_ERRORTYPE genericEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {

   if (ctx->userFlags & UFLAGS_VERBOSE) {
      fprintf(stderr, "Got an unhandled event of type %s (%x) on %s (d1: %x, d2: %x)\n",
         mapEvents(event), event, mapComponent(ctx, handle), data1, data2);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE decEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   switch (event) {
      case OMX_EventPortSettingsChanged:
         ctx->state = TUNNELSETUP; /* Port setting identified (decoder needs some frames to setup port parameters) call configure() from main loop to set up tunnels now we have those settings */
      break;
      case OMX_EventError:
         fprintf(stderr, "ERROR:%s %p: %x\n", mapComponent(ctx, handle), handle, data1);
         if (data1==OMX_ErrorSameState && ctx->componentFlags&CFLAGS_DEC)
            ctx->componentFlags &= ~CFLAGS_DEC; /* Clear flag - component is already in requested state */
      break;
      case OMX_EventCmdComplete:
         if (ctx->componentFlags&CFLAGS_DEC) {
            ctx->componentFlags &= ~CFLAGS_DEC;
         }
      break;
      default:
         genericEventHandler(handle, ctx, event, data1, data2, eventdata);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE encEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   switch (event) {
      case OMX_EventError:
         fprintf(stderr, "ERROR:%s %p: %x\n", mapComponent(ctx, handle), handle, data1);
         if (data1==OMX_ErrorSameState && ctx->componentFlags&CFLAGS_ENC)
            ctx->componentFlags &= ~CFLAGS_ENC; /* Clear flag - component is already in requested state */
      break;
      case OMX_EventCmdComplete:
         if (ctx->componentFlags&CFLAGS_ENC)
            ctx->componentFlags &= ~CFLAGS_ENC;
      break;
      default:
         genericEventHandler(handle, ctx, event, data1, data2, eventdata);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE rszEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   switch (event) {
      case OMX_EventError:
         fprintf(stderr, "ERROR:%s %p: %x\n", mapComponent(ctx, handle), handle, data1);
         if (data1==OMX_ErrorSameState && ctx->componentFlags&CFLAGS_RSZ)
            ctx->componentFlags &= ~CFLAGS_RSZ; /* Clear flag - component is already in requested state */
      break;
      case OMX_EventCmdComplete:
         if (ctx->componentFlags&CFLAGS_RSZ) {
            ctx->componentFlags &= ~CFLAGS_RSZ;
         }
      break;
      default:
         genericEventHandler(handle, ctx, event, data1, data2, eventdata);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE deiEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   switch (event) {
      case OMX_EventError:
         fprintf(stderr, "ERROR:%s %p: %x\n", mapComponent(ctx, handle), handle, data1);
         if (data1==OMX_ErrorSameState && ctx->componentFlags&CFLAGS_DEI)
            ctx->componentFlags &= ~CFLAGS_DEI; /* Clear flag - component is already in requested state */
      break;
      case OMX_EventCmdComplete:
         if (ctx->componentFlags&CFLAGS_DEI)
            ctx->componentFlags &= ~CFLAGS_DEI;
      break;
      default:
         genericEventHandler(handle, ctx, event, data1, data2, eventdata);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE splEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   switch (event) {
      case OMX_EventError:
         fprintf(stderr, "ERROR:%s %p: %x\n", mapComponent(ctx, handle), handle, data1);
         if (data1==OMX_ErrorSameState && ctx->componentFlags&CFLAGS_SPL)
            ctx->componentFlags &= ~CFLAGS_SPL; /* Clear flag - component is already in requested state */
      break;
      case OMX_EventCmdComplete:
         if (ctx->componentFlags&CFLAGS_SPL)
            ctx->componentFlags &= ~CFLAGS_SPL;
      break;
      default:
         genericEventHandler(handle, ctx, event, data1, data2, eventdata);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE vidEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   switch (event) {
      case OMX_EventError:
         fprintf(stderr, "ERROR:%s %p: %x\n", mapComponent(ctx, handle), handle, data1);
         if (data1==OMX_ErrorSameState && ctx->componentFlags&CFLAGS_VID)
            ctx->componentFlags &= ~CFLAGS_VID; /* Clear flag - component is already in requested state */
      break;
      case OMX_EventCmdComplete:
         if (ctx->componentFlags&CFLAGS_VID)
            ctx->componentFlags &= ~CFLAGS_VID;
      break;
      default:
         genericEventHandler(handle, ctx, event, data1, data2, eventdata);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE genericBufferCallback(OMX_HANDLETYPE handle, struct context *ctx, OMX_BUFFERHEADERTYPE *buf) {
   fprintf(stderr, "Got a buffer event on %s %p, buf %p\n", mapComponent(ctx, handle), handle, buf);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE emptied(OMX_HANDLETYPE handle, struct context *ctx, OMX_BUFFERHEADERTYPE *buf) {
   #ifdef DEBUG
      fprintf(stderr, "Got a buffer emptied event on %s %p, buf %p\n", mapComponent(ctx, handle), handle, buf);
   #endif
   pthread_mutex_lock(&ctx->decBufLock);
   buf->nFilledLen = 0; /* Flag buffer emptied */
   pthread_mutex_unlock(&ctx->decBufLock);
   return OMX_ErrorNone;
}

/* This is a blocking call; encoder will not continue until this returns.
 * OMX_FillThisBuffer() *must* not be called until this function returns,
 * otherwise some kind of deadlock happens!
 */
OMX_ERRORTYPE filled(OMX_HANDLETYPE handle, struct context *ctx, OMX_BUFFERHEADERTYPE *buf) {
   #ifdef DEBUG
      fprintf(stderr, "Got a buffer filled event on %s %p, buf %p\n", mapComponent(ctx, handle), handle, buf);
   #endif
   ctx->encBufferFilled=1;
   return OMX_ErrorNone;
}

OMX_CALLBACKTYPE encEventCallback = {
   (void (*))encEventHandler,
   (void (*))genericBufferCallback,
   (void (*))filled
};

OMX_CALLBACKTYPE decEventCallback = {
   (void (*)) decEventHandler,
   (void (*)) emptied,
   (void (*)) genericBufferCallback
};

OMX_CALLBACKTYPE rszEventCallback = {
   (void (*)) rszEventHandler,
   (void (*)) genericBufferCallback,
   (void (*)) genericBufferCallback
};

OMX_CALLBACKTYPE deiEventCallback = {
   (void (*)) deiEventHandler,
   (void (*)) genericBufferCallback,
   (void (*)) genericBufferCallback
};

OMX_CALLBACKTYPE vidEventCallback = {
   (void (*)) vidEventHandler,
   (void (*)) genericBufferCallback,
   (void (*)) genericBufferCallback
};

OMX_CALLBACKTYPE splEventCallback = {
   (void (*)) splEventHandler,
   (void (*)) genericBufferCallback,
   (void (*)) genericBufferCallback
};

static void *fps(void *p) {
   uint64_t lastframe;

   while (ctx.state!=DECEOF) {
      lastframe = ctx.framesOut;
      if (sleep(1)>0) break;
      fprintf(stderr, "Frame %6lld (%5.2fs).  Frames last second: %lli   pts delta: %llims     \r",
         ctx.framesOut, (double)ctx.framesOut/ctx.omxFPS, ctx.framesOut-lastframe, ctx.ptsDelta);
      fflush(stderr);
   }
   fprintf(stderr, "\n");
   return NULL;
}

/* Allocated buffers are stored as a linked list in pAppPrivate:
 * each pAppPrivate points to the next OMX_BUFFERHEADER in the list
 * Final buffer in the list has pAppPrivate set to NULL.
 */
static OMX_BUFFERHEADERTYPE *allocbufs(OMX_HANDLETYPE h, int port) {
   int i;
   OMX_BUFFERHEADERTYPE *list = NULL, **end = &list;
   OMX_PARAM_PORTDEFINITIONTYPE *portdef;

   MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);
   portdef->nPortIndex = port;
   OERR(OMX_GetParameter(h, OMX_IndexParamPortDefinition, portdef));

   fprintf(stderr, "Allocate %i %s buffers of %d bytes\n", portdef->nBufferCountActual, mapComponent(&ctx, h), portdef->nBufferSize);
   for (i = 0; i < portdef->nBufferCountActual; i++) {
      //OMX_U8 *buf= vcos_malloc_aligned(portdef->nBufferSize, portdef->nBufferAlignment, "buffer");
      //OERR(OMX_UseBuffer(h, end, port, NULL, portdef->nBufferSize, buf));
      OERR(OMX_AllocateBuffer(h, end, port, NULL, portdef->nBufferSize));
      end = (OMX_BUFFERHEADERTYPE **) &((*end)->pAppPrivate);
   }

   free(portdef);
   return list;
}

/* Request a component to change state and optionally wait:
 * wait == 0 Send request but don't wait for change
 * wait == 1 Send request and wait for state change
 * wait == 2 Don't send request, wait for an earlier requested state change
 */
static void requestStateChange(OMX_HANDLETYPE handle, enum OMX_STATETYPE rState, int wait) {
   enum OMX_STATETYPE aState;
   int i=0;

   if (wait != 2)
      OERR(OMX_SendCommand(handle, OMX_CommandStateSet, rState, NULL));
   if (wait > 0) {
      do {
         usleep(100);
         OMX_GetState(handle, &aState);
         i++;
         fprintf(stderr, "State %s =%i\r", mapComponent(&ctx, handle), aState);
      } while (i<10000 && aState!=rState);   /* Timeout 1s */
      fprintf(stderr, "\n");
      if (aState!=rState) {
         fprintf(stderr,"ERROR: timeout waiting for state change: wanted %i, got %i\n",rState, aState);
         exit(1);
      }
   }
}

static void waitForEvents(OMX_HANDLETYPE handle, uint8_t cFlag) {
   int i=0;
   do {  /* Wait command to complete */
      usleep(100);
      i++;
   } while (i<10000 && (ctx.componentFlags&cFlag));   /* Timeout 1s */
   if (ctx.componentFlags&cFlag) {
      fprintf(stderr,"ERROR: %s timeout waiting for command to complete.\n", mapComponent(&ctx, handle));
      exit(1);
   }
}

static void sendCommand(OMX_HANDLETYPE handle, OMX_COMMANDTYPE command, OMX_U32 port, uint8_t cFlag, int wait) {
   ctx.componentFlags|=cFlag;
   OERR(OMX_SendCommand(handle, command, port, NULL));

   if (wait>0) waitForEvents(handle, cFlag);
}

static OMX_PARAM_PORTDEFINITIONTYPE *configureResizer(struct context *ctx, OMX_PARAM_PORTDEFINITIONTYPE *portdef) {
   /* Resize and/or crop:
    * Do using hardware resizer: set input port size to input video size, output port size to
    * required rescale size. Crop is applied to input using OMX_IndexConfigCommonInputCrop
    */
   OMX_VIDEO_PORTDEFINITIONTYPE *viddef;
   OMX_PARAM_PORTDEFINITIONTYPE *imgportdef;
   OMX_IMAGE_PORTDEFINITIONTYPE *imgdef;
   
   MAKEME(imgportdef, OMX_PARAM_PORTDEFINITIONTYPE);

   sendCommand(ctx->rsz, OMX_CommandPortDisable, PORT_RSZ, CFLAGS_RSZ, 1);
   sendCommand(ctx->rsz, OMX_CommandPortDisable, PORT_RSZ+1, CFLAGS_RSZ, 1);

   /* Setup input port parameters */
   imgportdef->nPortIndex = PORT_RSZ;
   OERR(OMX_GetParameter(ctx->rsz, OMX_IndexParamPortDefinition, imgportdef));
   imgdef = &imgportdef->format.image;
   viddef = &portdef->format.video;

   /* Set input port image parameters to be same as previous output port */
   imgdef->nFrameWidth = viddef->nFrameWidth;
   imgdef->nFrameHeight = viddef->nFrameHeight;
   imgdef->nStride = viddef->nStride;
   imgdef->nSliceHeight = viddef->nSliceHeight;
   imgdef->bFlagErrorConcealment = viddef->bFlagErrorConcealment;
   imgdef->eCompressionFormat = viddef->eCompressionFormat;
   imgdef->eColorFormat = viddef->eColorFormat;
   imgdef->pNativeWindow = viddef->pNativeWindow;
   OERR(OMX_SetParameter(ctx->rsz, OMX_IndexParamPortDefinition, imgportdef)); /* Set input port parameters */

   /* Set output port image parameters: same as input port except for output port size */
   if (ctx->userFlags & UFLAGS_CROP) {
      if ((ctx->cropRect->nLeft + ctx->cropRect->nWidth) <= imgdef->nFrameWidth
       && (ctx->cropRect->nTop + ctx->cropRect->nHeight) <= imgdef->nFrameHeight) {
         OERR(OMX_SetConfig(ctx->rsz, OMX_IndexConfigCommonInputCrop, ctx->cropRect));
         imgdef->nFrameWidth = ctx->cropRect->nWidth;   /* Set new output size */
         imgdef->nFrameHeight = ctx->cropRect->nHeight;
      }
      else
         fprintf (stderr,"ERROR: Crop rectangle outside of frame dimensions: ignoring crop\n");
   }

   if (ctx->userFlags & UFLAGS_AUTO_SCALE_X) {
      ctx->outputWidth = imgdef->nFrameWidth*ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->sample_aspect_ratio.num/ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->sample_aspect_ratio.den;
      ctx->outputWidth += 0x0f;
      ctx->outputWidth &= ~0x0f;
      ctx->outputHeight = imgdef->nFrameHeight;
   }

   if (ctx->userFlags & UFLAGS_AUTO_SCALE_Y) {
      ctx->outputHeight = imgdef->nFrameHeight*ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->sample_aspect_ratio.den/ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->sample_aspect_ratio.num;
      ctx->outputHeight += 0x0f;
      ctx->outputHeight &= ~0x0f;
      ctx->outputWidth = imgdef->nFrameWidth;
   }

   if (ctx->userFlags & UFLAGS_RESIZE) {
      imgdef->nFrameWidth = ctx->outputWidth;
      imgdef->nFrameHeight = ctx->outputHeight;
   }

   /* Force component to re-calculate these */
   imgdef->nStride = 0;
   imgdef->nSliceHeight = 0;

   imgportdef->nPortIndex = PORT_RSZ+1;
   OERR(OMX_SetParameter(ctx->rsz, OMX_IndexParamPortDefinition, imgportdef));
   
   /* Setup video port definition for next component */
   OERR(OMX_GetParameter(ctx->rsz, OMX_IndexParamPortDefinition, imgportdef));
   viddef->nFrameWidth = imgdef->nFrameWidth;
   viddef->nFrameHeight = imgdef->nFrameHeight;
   viddef->nStride = imgdef->nStride;
   viddef->nSliceHeight = imgdef->nSliceHeight;
   viddef->bFlagErrorConcealment = imgdef->bFlagErrorConcealment;
   viddef->eCompressionFormat = imgdef->eCompressionFormat;
   viddef->eColorFormat = imgdef->eColorFormat;
   viddef->pNativeWindow = imgdef->pNativeWindow;

   return portdef;
}

static OMX_PARAM_PORTDEFINITIONTYPE *configureDeinterlacer(struct context *ctx, OMX_PARAM_PORTDEFINITIONTYPE *portdef) {
   OMX_CONFIG_IMAGEFILTERPARAMSTYPE *image_filter;
   OMX_VIDEO_PORTDEFINITIONTYPE *viddef;
   OMX_PARAM_PORTDEFINITIONTYPE *imgportdef;
   OMX_IMAGE_PORTDEFINITIONTYPE *imgdef;

   /* Set input port parameters */
   sendCommand(ctx->dei, OMX_CommandPortDisable, PORT_DEI, CFLAGS_DEI, 1);

   /* omxplayer does this */
   OMX_PARAM_U32TYPE *extra_buffers;
   MAKEME(extra_buffers, OMX_PARAM_U32TYPE);
   extra_buffers->nU32 = -2;
   extra_buffers->nPortIndex = PORT_DEI;
   OERR(OMX_SetParameter(ctx->dei, OMX_IndexParamBrcmExtraBuffers, extra_buffers));

   /*portdef->nPortIndex = PORT_DEI;
   OERR(OMX_SetParameter(ctx->dei, OMX_IndexParamPortDefinition, portdef));*/

   /* Setup output port parameters */
   sendCommand(ctx->dei, OMX_CommandPortDisable, PORT_DEI+1, CFLAGS_DEI, 1);

   portdef->nPortIndex = PORT_DEI+1;
   OERR(OMX_SetParameter(ctx->dei, OMX_IndexParamPortDefinition, portdef));
   MAKEME(image_filter, OMX_CONFIG_IMAGEFILTERPARAMSTYPE);
   
   /* Setup filter parameters */
   image_filter->nPortIndex = PORT_DEI+1;
   image_filter->nNumParams = 4;
   image_filter->nParams[0] = ctx->interlaceMode; /* OMX_INTERLACETYPE: see OMX_Broadcom.h. "Modes 1 and 2 are not handled. The line doubler algorithm only takes values 3 or 4, whilst the other two accept 0, 3, 4, or 5."; omxplayer hard codes this to 3. */
   image_filter->nParams[1] = 0; /* default frame interval */
   image_filter->nParams[2] = ctx->dei_ofpf; /* Setting one frame per field: NOTE this will double the frame rate and screw up the pts! This will be corrected in emptyEncoderBuffers() by using the duration. */
   image_filter->nParams[3] = 1; /* use qpus - quad processing units in the gpu */
   image_filter->eImageFilter = OMX_ImageFilterDeInterlaceAdvanced; /* Options are OMX_ImageFilterDeInterlaceLineDouble, OMX_ImageFilterDeInterlaceAdvanced, and OMX_ImageFilterDeInterlaceFast; see OMX_IVCommon.h for available filters; omxplayer uses OMX_ImageFilterDeInterlaceAdvanced for < 720x576 */
   OERR(OMX_SetConfig(ctx->dei, OMX_IndexConfigCommonImageFilterParameters, image_filter));

   /* Setup video port definition for next component */
   MAKEME(imgportdef, OMX_PARAM_PORTDEFINITIONTYPE);
   imgportdef->nPortIndex = PORT_DEI+1;
   OERR(OMX_GetParameter(ctx->dei, OMX_IndexParamPortDefinition, imgportdef));

   imgdef = &imgportdef->format.image;
   viddef = &portdef->format.video;

   viddef->nFrameWidth = imgdef->nFrameWidth;
   viddef->nFrameHeight = imgdef->nFrameHeight;
   viddef->nStride = imgdef->nStride;
   viddef->nSliceHeight = imgdef->nSliceHeight;
   viddef->bFlagErrorConcealment = imgdef->bFlagErrorConcealment;
   viddef->eCompressionFormat = imgdef->eCompressionFormat;
   viddef->eColorFormat = imgdef->eColorFormat;
   viddef->pNativeWindow = imgdef->pNativeWindow;

   if (image_filter->nParams[2]==0)
      viddef->xFramerate*=2;

   return portdef;
}

static OMX_PARAM_PORTDEFINITIONTYPE *configureMonitor(struct context *ctx, OMX_PARAM_PORTDEFINITIONTYPE *portdef) {
   OMX_DISPLAYRECTTYPE vidRect;
   OMX_CONFIG_DISPLAYREGIONTYPE *vidConf;
   int i;

   for (i = 0; i < 5; i++)
      sendCommand(ctx->spl, OMX_CommandPortDisable, PORT_SPL+i, CFLAGS_SPL, 1);
   sendCommand(ctx->vid, OMX_CommandPortDisable, PORT_VID, CFLAGS_VID, 1);

   MAKEME(vidConf,OMX_CONFIG_DISPLAYREGIONTYPE);
   // TODO: correct aspect ratio
   /* Don't show video full screen: define size 512x288 */
   vidRect.x_offset=0;
   vidRect.y_offset=0;
   vidRect.width=512;
   vidRect.height=288;

   vidConf->nPortIndex=PORT_VID;
   vidConf->set=OMX_DISPLAY_SET_FULLSCREEN|OMX_DISPLAY_SET_DEST_RECT;
   vidConf->fullscreen=OMX_FALSE;
   vidConf->dest_rect=vidRect;
   OERR(OMX_SetConfig(ctx->vid, OMX_IndexConfigDisplayRegion, vidConf));

   portdef->nPortIndex = PORT_SPL; /* Input to splitter */
   OERR(OMX_SetParameter(ctx->spl, OMX_IndexParamPortDefinition, portdef));
   portdef->nPortIndex = PORT_SPL+1;
   OERR(OMX_SetParameter(ctx->spl, OMX_IndexParamPortDefinition, portdef));
   portdef->nPortIndex = PORT_SPL+2;
   OERR(OMX_SetParameter(ctx->spl, OMX_IndexParamPortDefinition, portdef));

   return portdef; /* Unchanged in this case: outputs are a copy of input */
}

static void configure(struct context *ctx) {
   OMX_VIDEO_PARAM_PROFILELEVELTYPE *level;
   OMX_VIDEO_PARAM_BITRATETYPE *bitrate;
   OMX_PARAM_PORTDEFINITIONTYPE *portdef;
   OMX_VIDEO_PORTDEFINITIONTYPE *viddef;
   OMX_HANDLETYPE prev; /* Used in setting pipelines: previous handle */
   OMX_PARAM_U32TYPE *minQuant;
   OMX_PARAM_U32TYPE *maxQuant;
   OMX_CONFIG_INTERLACETYPE *interlaceType;

   int pp;

   MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);

   /* Get type of interlacing used, if any */
   MAKEME(interlaceType, OMX_CONFIG_INTERLACETYPE);
   interlaceType->nPortIndex = PORT_DEC+1;
   OERR(OMX_GetConfig(ctx->dec, OMX_IndexConfigCommonInterlace, interlaceType));
   ctx->interlaceMode=interlaceType->eMode;
   switch (ctx->interlaceMode) {
      case OMX_InterlaceProgressive: /* mode 0: no need for de-interlacer */
         fprintf(stderr, "Progresive scan detected, de-interlacing not required.\n", ctx->interlaceMode);
         if (ctx->userFlags & UFLAGS_DEINTERLACE)
            ctx->userFlags ^= UFLAGS_DEINTERLACE;
      break;
      case OMX_InterlaceFieldSingleUpperFirst:   /* mode 1: The data is interlaced, fields sent separately in temporal order, with upper field first */
      case OMX_InterlaceFieldSingleLowerFirst:   /* mode 2: The data is interlaced, fields sent separately in temporal order, with lower field first */
         fprintf(stderr, "Unsupported interlace format %i detected (separate field per frame).\n", ctx->interlaceMode);
         if (ctx->userFlags & UFLAGS_DEINTERLACE) {
            fprintf(stderr, "Disabling deinterlacer.\n");
            ctx->userFlags ^= UFLAGS_DEINTERLACE;
         }
      break;
      default:
         fprintf(stderr, "*** Source material is interlaced! Interlace type: %i ***\n", interlaceType->eMode);
         fprintf(stderr, "*** Consider using the de-interlacer option -d ***\n");
         // TODO: Switch on interlacer automatically? If so, will need to consider carefully the frame rate: if input is interlaced and 50fps, but omx detects 25fps, it's probably one field per frame and should output with one field per frame.
      break;
   }

   fprintf(stderr, "Setting up encoder.\n");

   /* Get the decoder OUTPUT port state */
   portdef->nPortIndex = PORT_DEC+1;
   OERR(OMX_GetParameter(ctx->dec, OMX_IndexParamPortDefinition, portdef));

   if (ctx->userFlags & UFLAGS_DEINTERLACE) 
      portdef=configureDeinterlacer(ctx, portdef);

   if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP)
      portdef=configureResizer(ctx, portdef);

   if (ctx->userFlags & UFLAGS_MONITOR)
     portdef=configureMonitor(ctx, portdef);

   sendCommand(ctx->enc, OMX_CommandPortDisable, PORT_ENC, CFLAGS_ENC, 1);
   sendCommand(ctx->enc, OMX_CommandPortDisable, PORT_ENC+1, CFLAGS_ENC, 1);

   /* Setup the encoder input port: portdef points to output port definition of the last component */
   viddef = &portdef->format.video;
   portdef->nPortIndex = PORT_ENC;
   OERR(OMX_SetParameter(ctx->enc, OMX_IndexParamPortDefinition, portdef)); /* Copy portdef of last component */

   /* Setup the tunnel(s): */
   prev = ctx->dec;   /* Start of tunnel: the decoder */
   pp = PORT_DEC+1;   /* Start of tunnel: decoder output */

   if (ctx->userFlags & UFLAGS_DEINTERLACE) {
      OERR(OMX_SetupTunnel(prev, pp, ctx->dei, PORT_DEI));
      prev = ctx->dei;
      pp = PORT_DEI + 1;
   }
   if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP) {
      OERR(OMX_SetupTunnel(prev, pp, ctx->rsz, PORT_RSZ));
      prev = ctx->rsz;
      pp = PORT_RSZ+1;
   }
   if (ctx->userFlags & UFLAGS_MONITOR) {
      OERR(OMX_SetupTunnel(prev, pp, ctx->spl, PORT_SPL)); /* Connect previous output to input of splitter */
      /* Tunnel 2nd port from splitter to video render */
      OERR(OMX_SetupTunnel(ctx->spl, PORT_SPL+2, ctx->vid, PORT_VID));
      prev = ctx->spl;
      pp = PORT_SPL+1;   /* First output sent to next stage in pipeline */
   }

   OERR(OMX_SetupTunnel(prev, pp, ctx->enc, PORT_ENC)); /* Final destination of pipeline */
   /* Set the pipeline to idle (waiting for data); call after setting up pipelines to auto allocate correct buffers; only buffers left to define are input and output to the pipeline */

   /* Now transition components to idle - do this here after all resources aquired */
   if (ctx->userFlags & UFLAGS_DEINTERLACE)
      requestStateChange(ctx->dei, OMX_StateIdle, 1);

   if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP)
      requestStateChange(ctx->rsz, OMX_StateIdle, 1);

   if (ctx->userFlags & UFLAGS_MONITOR) {
      requestStateChange(ctx->spl, OMX_StateIdle, 1);
      requestStateChange(ctx->vid, OMX_StateIdle, 1);
   }

   requestStateChange(ctx->enc, OMX_StateIdle, 1);

   /* setup encoder output port  - viddef points to format.video of previous component output port */
   viddef->nBitrate = ctx->bitrate;
   viddef->eCompressionFormat = OMX_VIDEO_CodingAVC;
   portdef->nPortIndex = PORT_ENC+1;
   OERR(OMX_SetParameter(ctx->enc, OMX_IndexParamPortDefinition, portdef));

   MAKEME(bitrate, OMX_VIDEO_PARAM_BITRATETYPE);
   bitrate->nPortIndex = PORT_ENC+1;
   bitrate->eControlRate = OMX_Video_ControlRateVariable;
   bitrate->nTargetBitrate = viddef->nBitrate;
   OERR(OMX_SetParameter(ctx->enc, OMX_IndexParamVideoBitrate, bitrate));

   /* Allowed values for pixel aspect are: 1:1, 10:11, 16:11, 40:33, 59:54, and 118:81
    * Note that these aspect ratios do not include overscan.
    * Corresponding display aspect ratio (DVD):
    * NTSC 10:11 -> 4:3 DAR
    * NTSC 40:33 -> 16:9 DAR
    * PAL 59:54 -> 4:3 DAR (for ANALOGUE signals: won't produce an integer of 16)
    * PAL 16:11 -> 16:9 DAR
    * PAL 118:81 -> 16:9 DAR (for ANALOGUE signals: won't produce an integer of 16)
    */
   if (ctx->userFlags & UFLAGS_RESIZE) { /* Probably defaults to this anyway... */
      OMX_CONFIG_POINTTYPE *pixaspect; 
      MAKEME(pixaspect, OMX_CONFIG_POINTTYPE);
      pixaspect->nPortIndex = PORT_ENC+1;
      pixaspect->nX = 1;
      pixaspect->nY = 1;
      OERR(OMX_SetParameter(ctx->enc, OMX_IndexParamBrcmPixelAspectRatio, pixaspect));
   }

   /* Allocate buffers; state must be idle & port disabled
    * Buffer allocation occurs during transition to state enabled.
    * Encoder only requires 1 output buffer; the buffer size varies
    * depending on input image size, doesn't seem to change with output size.
    * Around 500k for dvd stream, 3.5M for 1080p h264 stream.
    * If it does ask for more buffers, allocate them to get a state change,
    * but the current code set-up won't use them.
    */
   if (portdef->nBufferCountActual>1)
      fprintf(stderr,"WARNING: Encoder wants more than 1 output buffer: extra buffers not used!\n");
   sendCommand(ctx->enc, OMX_CommandPortEnable, PORT_ENC+1, CFLAGS_ENC, 0);
   ctx->encbufs = allocbufs(ctx->enc, PORT_ENC+1);
   waitForEvents(ctx->enc, CFLAGS_ENC);

   /* Enable ports: For port enable to succeed, *BOTH* ends of the pipeline need to be enabled.
    * Therefore, don't wait for output ports to be enabled - just queue command.
    * When destination port on the final component is enabled (always the encoder input port)
    * then wait for the other ports to enable in reverse order (i.e from encoder to components
    * further up the pipeline)
    */
   sendCommand(ctx->dec, OMX_CommandPortEnable, PORT_DEC+1, CFLAGS_DEC, 0); /* Don't wait */

   if (ctx->userFlags & UFLAGS_DEINTERLACE) {
      sendCommand(ctx->dei, OMX_CommandPortEnable, PORT_DEI, CFLAGS_DEI, 1);
      sendCommand(ctx->dei, OMX_CommandPortEnable, PORT_DEI+1, CFLAGS_DEI, 0); /* Don't wait */
   }

   if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP) {
      sendCommand(ctx->rsz, OMX_CommandPortEnable, PORT_RSZ, CFLAGS_RSZ, 1);
      sendCommand(ctx->rsz, OMX_CommandPortEnable, PORT_RSZ+1, CFLAGS_RSZ, 0); /* Don't wait */
   }

   if (ctx->userFlags & UFLAGS_MONITOR) {
      sendCommand(ctx->vid, OMX_CommandPortEnable, PORT_VID, CFLAGS_VID, 1);
      sendCommand(ctx->spl, OMX_CommandPortEnable, PORT_SPL, CFLAGS_SPL, 1);
      sendCommand(ctx->spl, OMX_CommandPortEnable, PORT_SPL+1, CFLAGS_SPL, 0); /* Encoder - don't wait, encoder port not yet enabled */
      sendCommand(ctx->spl, OMX_CommandPortEnable, PORT_SPL+2, CFLAGS_SPL, 1); /* Video render */
   }

   sendCommand(ctx->enc, OMX_CommandPortEnable, PORT_ENC, CFLAGS_ENC, 1);
   /* Wait for port enable commands to complete
    * This shouldn't be neccessary as we wait for encoder above;
    * if encoder enable completes then all of these should also
    * have completed, but lets check anyway!
    */
   waitForEvents(ctx->rsz, CFLAGS_DEC);
   waitForEvents(ctx->rsz, CFLAGS_RSZ);
   waitForEvents(ctx->dei, CFLAGS_DEI);
   waitForEvents(ctx->spl, CFLAGS_SPL);

   /* Transition to state executing */
   if (ctx->userFlags & UFLAGS_DEINTERLACE)
      requestStateChange(ctx->dei, OMX_StateExecuting, 1);

   if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP)
      requestStateChange(ctx->rsz, OMX_StateExecuting, 1);

   if (ctx->userFlags & UFLAGS_MONITOR) {
      requestStateChange(ctx->spl, OMX_StateExecuting, 1);
      requestStateChange(ctx->vid, OMX_StateExecuting, 1);
   }

   requestStateChange(ctx->enc, OMX_StateExecuting, 1);

   /* Start encoding */
   OERR(OMX_FillThisBuffer(ctx->enc, ctx->encbufs));

   /* Dump current port states: */
   
   if (ctx->userFlags & UFLAGS_VERBOSE) {
      dumpport(ctx->dec, PORT_DEC);
      dumpport(ctx->dec, PORT_DEC+1);
      if (ctx->userFlags & UFLAGS_DEINTERLACE) {
         dumpport(ctx->dei, PORT_DEI);
         dumpport(ctx->dei, PORT_DEI+1);
      }
      if (ctx->userFlags & UFLAGS_RESIZE || ctx->userFlags & UFLAGS_CROP) {
         dumpport(ctx->rsz, PORT_RSZ);
         dumpport(ctx->rsz, PORT_RSZ+1);
      }
      dumpport(ctx->enc, PORT_ENC);
      dumpport(ctx->enc, PORT_ENC+1);
   }

   MAKEME(level, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
   level->nPortIndex = PORT_ENC+1;
   OERR(OMX_GetParameter(ctx->enc, OMX_IndexParamVideoProfileLevelCurrent, level));

   if (ctx->minQuant > 0) {
      MAKEME(minQuant, OMX_PARAM_U32TYPE);
      minQuant->nU32 = ctx->minQuant;
      minQuant->nPortIndex = PORT_ENC+1;
      OERR(OMX_SetParameter(ctx->enc, OMX_IndexParamBrcmVideoEncodeMinQuant, minQuant));
   }

   if (ctx->maxQuant > 0) {
      MAKEME(maxQuant, OMX_PARAM_U32TYPE);
      maxQuant->nU32 = ctx->maxQuant;
      maxQuant->nPortIndex = PORT_ENC+1;
      OERR(OMX_SetParameter(ctx->enc, OMX_IndexParamBrcmVideoEncodeMaxQuant, maxQuant));
   }

   /* Get the frame rate at the encoder output
    * This is detected and set by the decoder:
    * seems to be set from stream data if present or from omx ticks
    * Constant framerate is assumed.
    */
   if (portdef->format.video.xFramerate==0) {   /* If unknown use average fps from input */
      fprintf(stderr, "WARNING: frame rate unknown - setting rate from input. This may not be correct!\n");
      portdef->format.video.xFramerate=(ctx->ic->streams[ctx->inVidStreamIdx]->avg_frame_rate.num/ctx->ic->streams[ctx->inVidStreamIdx]->avg_frame_rate.den)*(1<<16);
   }

   ctx->nalEntry.fps.num=portdef->format.video.xFramerate;   /* Q16 format */
   ctx->nalEntry.fps.den=(1<<16);
   
   ctx->omxFPS=av_q2d(ctx->nalEntry.fps); /* Convert to double */
   ctx->nalEntry.duration=(double)ctx->omxtimebase.den/ctx->omxFPS;  /* Estimate frame duration in omx timebase units */

   /* Make an output context if output is not raw: */
   if ((ctx->userFlags & UFLAGS_RAW) == 0) {
      ctx->oc = makeOutputContext(ctx->ic, ctx->oname, ctx->inVidStreamIdx, portdef, level);
      if (!ctx->oc) {
         fprintf(stderr, "Create output AVFormatContext failed.\n");
         exit(1);
      }
   }

   ctx->state=OPENOUTPUT;
}

static OMX_BUFFERHEADERTYPE *configDecoder(struct context *ctx) {
   OMX_PARAM_PORTDEFINITIONTYPE *portdef;
   OMX_VIDEO_PORTDEFINITIONTYPE *viddef;
   OMX_BUFFERHEADERTYPE *decbufs;
   OMX_NALSTREAMFORMATTYPE *nalStreamFormat;

   MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);

   sendCommand(ctx->dec, OMX_CommandPortDisable, PORT_DEC, CFLAGS_DEC, 1);
   sendCommand(ctx->dec, OMX_CommandPortDisable, PORT_DEC+1, CFLAGS_DEC, 1);

   portdef->nPortIndex = PORT_DEC;
   OERR(OMX_GetParameter(ctx->dec, OMX_IndexParamPortDefinition, portdef));
   viddef = &portdef->format.video;
   viddef->nFrameWidth = ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->width;
   viddef->nFrameHeight = ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->height;
   viddef->eCompressionFormat = mapCodec(ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->codec_id);
   viddef->bFlagErrorConcealment = 0;
   /* It is NOT required to set xFramerate from ffmpeg avg_frame_rate.den; the encoder will be passed the detected frame rate (I assume from the timestamps / omxtick or from raw stream data). */
   OERR(OMX_SetParameter(ctx->dec, OMX_IndexParamPortDefinition, portdef));

/* TODO! */
   if (ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->codec_id==AV_CODEC_ID_H264) {
      if (ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->extradata==NULL) ctx->naluInputFormat=1;
      else if (ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->extradata_size<7) ctx->naluInputFormat=1;
      else if (*(ctx->ic->streams[ctx->inVidStreamIdx]->codecpar->extradata)!=1) ctx->naluInputFormat=1; // omxplayer: valid avcC atom data always starts with the value 1 (version), otherwise annexb
      else ctx->naluInputFormat=0;
      if (ctx->naluInputFormat==1) {
         fprintf(stderr, "** h264 annexb format detected\n");
         MAKEME(nalStreamFormat, OMX_NALSTREAMFORMATTYPE);

         nalStreamFormat->nPortIndex = PORT_DEC;
         nalStreamFormat->eNaluFormat = OMX_NaluFormatStartCodes;
         OERR(OMX_SetParameter(ctx->dec, OMX_IndexParamNalStreamFormatSelect, nalStreamFormat));
      }
   }

   /* Allocate the input buffers: port needs to be in idle state, and disabled.
    * Note that it doesn't actually enable until after all buffers allocated, so
    * wait for port enable after allocation.
    */
   requestStateChange(ctx->dec, OMX_StateIdle, 1);
   sendCommand(ctx->dec, OMX_CommandPortEnable, PORT_DEC, CFLAGS_DEC, 0);
   decbufs = allocbufs(ctx->dec, PORT_DEC); /* returns a linked list of buffers */
   waitForEvents(ctx->dec, CFLAGS_DEC);

   ctx->state = DECINIT;
   requestStateChange(ctx->dec, OMX_StateExecuting, 1);    /* Start decoder */

   free(portdef);
   return decbufs;
}

static void usage(const char *name) {
   fprintf(stderr, "Usage: %s [opts] <infile> <outfile>\n\n"
      "Where opts are:\n"
      "   -a[y] Auto scale the video stream to produce a sample aspect ratio (pixel aspect ratio)\n"
      "         of 1:1. By default the scaling is in the x-direction (image width); this usually\n"
      "         results in more (interpolated) pixels in the x-direction. If y is specified, the scaling\n"
      "         is done in the y-direction; this usually results in a reduction in resolution in the\n"
      "         y-direction.\n"
      "   -b n  Target bitrate n[k|M] in bits/second (default: 2Mb/s)\n"
      "   -c C  Crop: 'C' is specified in pixels as width:height:left:top\n"
      "   -d[0] Deinterlace: default is to output one frame per two interlaced fields\n"
      "         If 0 is specified, one frame per field will be output.\n"
      "   -f F  Format: 'F' overrides format else guessed from the file name, usefull if decoding to stdout\n"
      "   -i n  Select audio stream n.\n"
      "   -m    Monitor.  Display the decoder's output\n"
      "   -p    Make up pts based on encoder (constant) framerate. Default is based on input stream dts.\n"
      "   -q Q  Quantisation limits: 'Q' is specified as min=a:max=b where 0 < a < b < 52.\n"
      "         Defaults to a=20, b=50.\n"
      "   -r S  Resize: 'S' is in pixels specified as widthxheight\n"
      "   -v    Verbose: show input / output states of OMX components\n"
      "\n"
      "Output container is guessed based on filename. Use '.nal' for raw output.\n"
      "\n"
      "Input file must contain one of MPEG2, H.264, MPEG4 (H.263), MJPEG or vp8 video.\n"
      "\n", name);
   exit(1);
}

static int parsebitrate(const char *s, int defaultRate) {
   float rate;
   char specifier;
   int r;

   if (s!=NULL) {
      r = sscanf(s, "%f%c", &rate, &specifier);
      switch (r) {
      case 1:
         return (int)rate;
      case 2:
         switch (specifier) {
         case 'K':
         case 'k':
            return (int)(rate * 1024.0);
         case 'M':
         case 'm':
            return (int)(rate * 1024.0 * 1024.0);
         default:
            fprintf(stderr, "Unrecognised bitrate specifier!\n");
            exit(1);
            break;
         }
         break;
      default:
         fprintf(stderr, "Failed to parse bitrate!\n");
         exit(1);
         break;
      }
   }
   return defaultRate;
}

/* If the encoder isn't running, save any audio packets for remux after the output file has been opened.
 * The output file can't be opened until SPS and PPS information have been read into codec->extradata
 */
static AVPacket *getNextVideoPacket(struct context *ctx) {
   int rc;
   AVPacket *pkt=NULL;

   while(1) {
      pkt=av_packet_alloc();
      rc = av_read_frame(ctx->ic, pkt);   /* This allocates buf */
      if (rc != 0) {
         av_packet_free(&pkt);
         pkt=NULL;
         break;
      }
      if (pkt->stream_index == ctx->inVidStreamIdx)   /* Found a video packet: return it */
         break;

      if (pkt->stream_index == ctx->inAudioStreamIdx) { /* ctx->inAudioStreamIdx<0 if no audio stream */
         if (ctx->state == RUNNING)  /* Write out audio packet */
            writeAudioPacket(pkt);
         else { /* Encoder not running: save packet for remux when we open the output file */
            struct packetentry *entry;
            entry = malloc(sizeof(struct packetentry));
            if (entry!=NULL) {
               entry->packet = pkt; /* Take ref */
               TAILQ_INSERT_TAIL(&packetq, entry, link);
               continue;
            }
         }
      }
      av_packet_free(&pkt);          /* If discard packet */ 
   };

   return pkt;
}

static int setCropRectangle(struct context *ctx, const char *optarg) {
   int cropLeft, cropTop, cropWidth, cropHeight;

   if (optarg!=NULL) {
      if (sscanf(optarg, "%d:%d:%d:%d", &cropWidth, &cropHeight, &cropLeft, &cropTop) == 4) {
         cropTop += 0x04;  /* Interlaced material requires y offset is a multiple of 4 */ 
         cropTop &= ~0x04;
         cropWidth += 0x0f;
         cropWidth &= ~0x0f;
         cropHeight += 0x0f;
         cropHeight &= ~0x0f;
         if (cropWidth > 16 && cropHeight > 16) {
            MAKEME(ctx->cropRect,OMX_CONFIG_RECTTYPE);
            ctx->cropRect->nPortIndex = PORT_RSZ;
            ctx->cropRect->nLeft=cropLeft;
            ctx->cropRect->nTop=cropTop;
            ctx->cropRect->nWidth=cropWidth;
            ctx->cropRect->nHeight=cropHeight;
            return 0;
         }
      }
   }
   fprintf(stderr,"ERROR: Invalid crop parameters\n");
   return 1;
}

static int setOutputSize(struct context *ctx, const char *optarg) {
   int outputWidth, outputHeight;

   if (optarg!=NULL) {
      if (sscanf(optarg, "%dx%d", &outputWidth, &outputHeight) == 2) {
         outputWidth += 0x0f; /* round x up to next multiple of 16; if already a multiple of 16, x is unchanged */
         outputWidth &= ~0x0f;
         outputHeight += 0x0f; /* round y up to next multiple of 16 */
         outputHeight &= ~0x0f;
         if (outputWidth > 16 && outputHeight > 16) {
            ctx->outputWidth=outputWidth;
            ctx->outputHeight=outputHeight;
            return 0;
         }
      }
   }
   fprintf(stderr,"ERROR: Invalid resize parameters\n");
   return 1;
}

/* Quantisation options. Range 1 to 51, lower numbers are better quality / bigger file size
 * Default appears to be min=20, max=50. Lower BOTH numbers to increase quality
 * (setting minQuant to 1 roughly doubles the file size)
 */
static int setQuantLimits(struct context *ctx, const char *optarg) {
   int minq=0;
   int maxq=0;

   if (optarg!=NULL) {
      if (sscanf(optarg, "min=%d:max=%d", &minq, &maxq) != 2) {
         fprintf(stderr,"ERROR: Must specify 'min=a:max=b' where 0 < a < b < 52\n");
         return 1;
      }
   }
   if (minq > 0 && maxq > minq && maxq < 52) {
      printf("Setting quantisation limits to qmin=%d, qmax=%d\n", minq, maxq);
      ctx->minQuant=minq;
      ctx->maxQuant=maxq;
      return 0;
   }
   fprintf(stderr,"ERROR: Invalid quantisation parameters! Valid range: 0 < qmin < qmax < 52 \n");
   return 1;
}

/* WARNING: use is made of optag==NULL in parsing the input options. This is a gnu extension */
static void setupUserOpts(struct context *ctx, int argc, char *argv[]) {
   int opt, j;

   if (argc < 3)
      usage(argv[0]);

   ctx->bitrate = 2*1024*1024;   /* Default: 2Mb/s */
   ctx->userAudioStreamIdx=-1;   /* Default: guess audio stream */
   ctx->minQuant=0;              /* Default minimum quantisation: use 0 avoid resetting from firmware default (20?)*/
   ctx->maxQuant=0;              /* Default maximum quantisation: use 0 avoid resetting from firmware default (50?)*/
   ctx->dei_ofpf=1;              /* Deinterlace: set to 0 for one frame per two fields; set to 1 to use one frame per field */

   while ((opt = getopt(argc, argv, "a::b:c:d:f:i:mpq:r:v")) != -1) {
      switch (opt) {
      case 'a':
         ctx->userFlags |= UFLAGS_AUTO_SCALE_X;
         ctx->userFlags |= UFLAGS_RESIZE;
         if (optarg!=NULL && *optarg=='y')
            ctx->userFlags |= UFLAGS_AUTO_SCALE_Y;            
      break;
      case 'b':
         ctx->bitrate = parsebitrate(optarg, ctx->bitrate);
         if (ctx->bitrate < 128*1024)
            fprintf(stderr, "WARNING: bit rate too low; use at least 128kb/s!\n", strerror(errno));
      break;
      case 'c':
         if (setCropRectangle(ctx, optarg)==0)
            ctx->userFlags |= UFLAGS_CROP;
         else
            exit(1);
      break;
      case 'd':
         ctx->userFlags |= UFLAGS_DEINTERLACE;
         if (optarg!=NULL && *optarg=='0')
            ctx->dei_ofpf=0;
      break;
      case 'f':
         if (optarg!=NULL)
            ctx->oformat = optarg;
      break;
      case 'i':
         ctx->userAudioStreamIdx=atoi(optarg);
      break;
      case 'm':
         ctx->userFlags |= UFLAGS_MONITOR;
      break;
      case 'p':
         ctx->userFlags |= UFLAGS_MAKE_UP_PTS;
      break;
      case 'q':
         if (setQuantLimits(ctx, optarg)==1)
            exit(1);
      break;
      case 'r':
         if (setOutputSize(ctx, optarg)==0)
            ctx->userFlags |= UFLAGS_RESIZE;
         else
            exit(1);
      break;
      case 'v':
         ctx->userFlags |= UFLAGS_VERBOSE;
      break;
      default:
         usage(argv[0]);
      break;
      }
   }
   if (optind+1 >= argc) {
      fprintf(stderr, "Expected arguments after options\n");
      exit(1);
   }
   ctx->iname = argv[optind++];
   ctx->oname = argv[optind++];
   j = strlen(ctx->oname);
   if (strncmp(&(ctx->oname[j-4]), ".nal", 4) == 0 || strncmp(&(ctx->oname[j-4]), ".264", 4) == 0) {
      ctx->userFlags |= UFLAGS_RAW;
      ctx->raw_fd = open(ctx->oname, O_CREAT|O_TRUNC|O_WRONLY, 0666);
      if (ctx->raw_fd == -1) {
         fprintf(stderr, "Failed to open the output: %s\n", strerror(errno));
         exit(1);
      }
   }
}

static int openInputFile(struct context *ctx) {
   AVFormatContext *ic=NULL;   /* Input context */
   int err;

   av_register_all();

   if ((err = avformat_open_input(&ic, ctx->iname, NULL, NULL) != 0)) {
      fprintf(stderr, "Failed to open '%s': %s\n", ctx->iname, strerror(err));
      exit(1);
   }

   if (avformat_find_stream_info(ic, NULL) < 0) {
      fprintf(stderr, "Failed to find streams in '%s'\n", ctx->iname);
      avformat_close_input(&ic);
      exit(1);
   }

   ctx->inVidStreamIdx = av_find_best_stream(ic, AVMEDIA_TYPE_VIDEO, -1, -1, NULL, 0);
   if (ctx->inVidStreamIdx < 0) {
      fprintf(stderr, "Failed to find video stream in '%s'\n", ctx->iname);
      avformat_close_input(&ic);
      exit(1);
   }
   if (!(ctx->userFlags&UFLAGS_RAW)) {
      ctx->inAudioStreamIdx = av_find_best_stream(ic, AVMEDIA_TYPE_AUDIO, ctx->userAudioStreamIdx, -1, NULL, 0);
      if (ctx->inAudioStreamIdx < 0) {
         fprintf(stderr, "Failed to find audio stream in '%s'\n", ctx->iname);
      }
   }
   ctx->ic=ic;

   fprintf(stderr, "Input:\n");
   av_dump_format(ic, 0, ctx->iname, 0);

   return ic->streams[ctx->inVidStreamIdx]->codecpar->codec_id;   /* Return the codec ID for the video stream */
}

static int examineNAL(struct context *ctx) {
   if (ctx->nalEntry.nalBuf[0] == 0 && ctx->nalEntry.nalBuf[1] == 0 && ctx->nalEntry.nalBuf[2] == 0 && ctx->nalEntry.nalBuf[3] == 1)
      return ctx->nalEntry.nalBuf[4] & 0x1f;
   else
      return -1;
}

/* Transfer nal buffer to avpacket for writing to file
 * OMX_BUFFERFLAG_SYNCFRAME defined in IL/OMX_Core.h:
 * Sync Frame Flag: This flag is set when the buffer content contains a coded sync frame -
 *  a frame that has no dependency on any other frame information 
 * i.e this is an IDR-frame or keyframe used for indexing. Trouble is, it doesn't always
 * appear to actually be an IDR frame: check for IDR by looking at NAL unit type.
 */
static void writeVideoPacket(struct context *ctx, int nalType) {
   AVPacket pkt;
   int r=-1;
   av_init_packet(&pkt); /* pkt.data is set to NULL here */
   pkt.stream_index = 0;
   pkt.data = ctx->nalEntry.nalBuf;
   pkt.size = ctx->nalEntry.nalBufOffset;
   pkt.pts=av_rescale_q(ctx->nalEntry.pts, ctx->omxtimebase, ctx->oc->streams[0]->time_base); /* Transform omx pts to output timebase */
   pkt.dts=pkt.pts; /* Out of order b-frames not supported on rpi: so dts=pts */
   ctx->ptsDelta=(ctx->nalEntry.pts-ctx->nalEntry.tick)/1000;
   //fprintf(stderr, "pts:%lld, tick:%lld\n", ctx->nalEntry.pts, ctx->nalEntry.tick);

   if (nalType==5)   /* This is an IDR frame */
      pkt.flags |= AV_PKT_FLAG_KEY;

   r = av_interleaved_write_frame(ctx->oc, &pkt);
   if (r != 0) {
      char err[256];
      av_strerror(r, err, sizeof(err));
      fprintf(stderr,"\nFailed to write a video frame: %s (pts: %lld; nal: %i)\n", err, ctx->nalEntry.pts, nalType);
   }
   else ctx->framesOut++; /* This assumes 1 nalu is equivalent to 1 frame */
}

/* The h264 video data is organized into NAL units (annex b), each of which is effectively a packet
 * that contains part of, or a full, frame. The first byte of each H.264/AVC NAL unit is a
 * header byte that contains an indication of the type of data in the NAL unit.
 * Note that OMX_BUFFERFLAG_ENDOFNAL is defined in OMX_Broadcom.h and is broadcom specific.
 * It is possible that 1 NAL unit will not fit into 1 buffer, and may be split over several
 * buffers. Copy the buffer data into nalBuf, advancing the start location by nFilledLen for
 * each incomplete NAL: i.e. sum buf into nalBuf until end of NAL flag.
 * nalBuf points to a buffer of size nalEntry.nalBufSize; this is re-used and should not be freed here.
 * For mkv files:
 *    Extract extradata required *before* output file header can be written: first two nals from rpi
 *    are sps followed by pps. These must be known before the output file can be opened.
 */
static void emptyEncoderBuffers(struct context *ctx) {
   int nalType=-1;
   size_t curNalSize;
   int i;

   if (ctx->encBufferFilled==0) {
      ctx->encWaitTime++;
      return;  /* Buffer is empty - return to main loop */
   }

   if (ctx->userFlags & UFLAGS_RAW) {
      write(ctx->raw_fd, ctx->encbufs->pBuffer + ctx->encbufs->nOffset, ctx->encbufs->nFilledLen);
   }
   else {
      if (ctx->state==OPENOUTPUT && (ctx->encbufs->nFlags & OMX_BUFFERFLAG_CODECCONFIG)) {
         fprintf(stderr, "Examining extradata...\n");
         /* This code was copied from ffmpeg */
         if (av_reallocp(&ctx->oc->streams[0]->codecpar->extradata, ctx->oc->streams[0]->codecpar->extradata_size + ctx->encbufs->nFilledLen + AV_INPUT_BUFFER_PADDING_SIZE) == 0) {
            memcpy(ctx->oc->streams[0]->codecpar->extradata + ctx->oc->streams[0]->codecpar->extradata_size, ctx->encbufs->pBuffer + ctx->encbufs->nOffset, ctx->encbufs->nFilledLen);
            ctx->oc->streams[0]->codecpar->extradata_size += ctx->encbufs->nFilledLen;
            memset(ctx->oc->streams[0]->codecpar->extradata + ctx->oc->streams[0]->codecpar->extradata_size, 0, AV_INPUT_BUFFER_PADDING_SIZE); /* Add extra zeroed bytes to prevent certain optimised readers from reading past the end */
         }
         else  {
            fprintf(stderr, "\nERROR: Failed to allocate memory for extradata.\n");
            exit(1);
         }
         int nals[32] = { 0 };   /* Check that we have both sps and pps in extradata: newer versions of rpi libs put both in one buffer, older versions used separate buffers */
         for (i = 0; i + 4 < ctx->oc->streams[0]->codecpar->extradata_size; i++) {
             if (!ctx->oc->streams[0]->codecpar->extradata[i + 0] &&
                 !ctx->oc->streams[0]->codecpar->extradata[i + 1] &&
                 !ctx->oc->streams[0]->codecpar->extradata[i + 2] &&
                 ctx->oc->streams[0]->codecpar->extradata[i + 3] == 1) {
                 nals[ctx->oc->streams[0]->codecpar->extradata[i + 4] & 0x1f]++;
             }
         }
         if (nals[7] && nals[8]) {
            openOutput(ctx);
            ctx->state = RUNNING;
         }
      }
      else {
         curNalSize=ctx->nalEntry.nalBufOffset+ctx->encbufs->nFilledLen;
         if (curNalSize>ctx->nalEntry.nalBufSize) {
            fprintf(stderr, "\nERROR: nalBufSize exceeded.\n");
            exit(1);
         }
         memcpy(ctx->nalEntry.nalBuf + ctx->nalEntry.nalBufOffset, ctx->encbufs->pBuffer + ctx->encbufs->nOffset, ctx->encbufs->nFilledLen);
         ctx->nalEntry.nalBufOffset=curNalSize;
         ctx->nalEntry.tick=((((int64_t) ctx->encbufs->nTimeStamp.nHighPart)<<32) | ctx->encbufs->nTimeStamp.nLowPart);
         if (ctx->nalEntry.tick > ctx->nalEntry.pts)
            ctx->nalEntry.pts=ctx->nalEntry.tick; /* This is propagated through from the decoder */
         else
            ctx->nalEntry.pts+=ctx->nalEntry.duration; /* Something went wrong - make up pts based on detected framerate */

         if (ctx->encbufs->nFlags & OMX_BUFFERFLAG_ENDOFNAL) { /* At end of nal */
            nalType=examineNAL(ctx);
            if (ctx->state == RUNNING) writeVideoPacket(ctx, nalType);
            else if (ctx->state==OPENOUTPUT && nalType==5) {
               fprintf(stderr, "\nERROR: sps or pps or both missing from encoder stream.\n");
               exit(1);
            }
            ctx->nalEntry.nalBufOffset = 0;
         }
         else if ( ! ctx->encbufs->nFlags & OMX_BUFFERFLAG_EOS)
            fprintf(stderr, "\nWarning: End of NAL not found!\n");
      }
   }
   ctx->encBufferFilled=0;                /* Flag that the buffer is empty */
   ctx->encbufs->nFilledLen = 0;
   ctx->encbufs->nOffset = 0;
   if (ctx->encbufs->nFlags & OMX_BUFFERFLAG_EOS) /* This is the last buffer */
      ctx->state=ENCEOS;
   else
      OERR(OMX_FillThisBuffer(ctx->enc, ctx->encbufs)); /* Finished processing buffer - request buffer refill */
}

static void *sigHandler_thread(void *arg) {
   sigset_t *set = arg;
   int s, sig;

   for (;;) {
      s = sigwait(set, &sig);
      if (s != 0)
         perror("sigwait()");
      else
         ctx.state=QUIT;
   }
   
   return NULL;
}

OMX_BUFFERHEADERTYPE *getSpareDecBuffer(struct context *ctx) {
   OMX_BUFFERHEADERTYPE *spare;

   do {
      emptyEncoderBuffers(ctx); /* Empty filled encoder buffers as required */
      spare = ctx->decbufs;
      while (spare!=NULL && spare->nFilledLen != 0) /* Find a free buffer (indicated by nFilledLen == 0); if spare==NULL all buffers are used (pAppPrivate of last buffer is NULL) */
         spare = spare->pAppPrivate;
      usleep(10);
   } while (spare==NULL);
   return spare;
}

void fillDecBuffers(struct context *ctx, int i, AVPacket *p) {
   int offset;
   int size, nsize;
   OMX_BUFFERHEADERTYPE *spare;
   OMX_TICKS tick;
   int64_t omxTicks;

   /* From ffmpeg docs: pkt->pts can be AV_NOPTS_VALUE (-9223372036854775808) if the video format has B-frames, so it is better to rely on pkt->dts if you do not decompress the payload */
   if ( !(ctx->userFlags & UFLAGS_MAKE_UP_PTS) && p->dts != AV_NOPTS_VALUE)
      ctx->videoPTS=p->dts;
   else
      ctx->videoPTS+=p->duration; /* Use packet duration */

   omxTicks=av_rescale_q(ctx->videoPTS, ctx->ic->streams[ctx->inVidStreamIdx]->time_base, ctx->omxtimebase); /* Transform input timebase to omx timebase */

   //fprintf(stderr,"Input pts: %lld; timebase: %i/%i\n", ctx->videoPTS, ctx->ic->streams[ctx->inVidStreamIdx]->time_base.num, ctx->ic->streams[ctx->inVidStreamIdx]->time_base.den);
   tick.nLowPart = (uint32_t) (omxTicks & 0xffffffff);
   tick.nHighPart = (uint32_t) ((omxTicks & 0xffffffff00000000) >> 32);

   size = p->buf->size;
   offset = 0;
   while (size>0) {
      spare=getSpareDecBuffer(ctx);
      if (i==0) spare->nFlags=OMX_BUFFERFLAG_STARTTIME;
      else spare->nFlags=0;

      /* Fill the decoder buffer */
      if (size > spare->nAllocLen)  /* Frame is too big for one buffer */
         nsize = spare->nAllocLen;
      else {
         nsize = size;     /* Frame will fit in buffer */
         spare->nFlags |= OMX_BUFFERFLAG_ENDOFFRAME;
      }
      memcpy(spare->pBuffer, p->buf->data+offset, nsize);

      if (p->flags & AV_PKT_FLAG_KEY)
         spare->nFlags |= OMX_BUFFERFLAG_SYNCFRAME;

      spare->nTimeStamp = tick;
      pthread_mutex_lock(&ctx->decBufLock);
      spare->nFilledLen = nsize;
      pthread_mutex_unlock(&ctx->decBufLock);
      spare->nOffset = 0;
      OERR(OMX_EmptyThisBuffer(ctx->dec, spare));
      size -= nsize;
      offset += nsize;
   }
   ctx->framesIn++; /* This assumes 1 frame per buffer */
}

int main(int argc, char *argv[]) {
   int i, j;
   time_t start, end;
   AVPacket *p=NULL;
   OMX_BUFFERHEADERTYPE *spare;
   pthread_t fpst;
   pthread_attr_t fpsa;
   sigset_t set;
   pthread_t sigThread;

   setupUserOpts(&ctx, argc, argv);
   ctx.omxtimebase.num=1;
   ctx.omxtimebase.den=1000000; /* OMX timebase is in micro seconds */
   ctx.nalEntry.nalBufSize=1024*1024*2; /* 2MB buffer */
   ctx.nalEntry.nalBuf=av_malloc(ctx.nalEntry.nalBufSize);
   if (ctx.nalEntry.nalBuf==NULL) {
      fprintf(stderr,"ERROR: Can't allocate memory for nalBuf\n");
      exit(1);
   }
   ctx.nalEntry.nalBufOffset=0;
   ctx.nalEntry.pts=0;
   ctx.framesOut=0;
   ctx.framesIn=0;
   ctx.componentFlags=0;
   ctx.componentFlags=0;
   ctx.encBufferFilled=0;
   ctx.naluInputFormat=0;

   TAILQ_INIT(&packetq);

   /* Block SIGINT and SIGQUIT; other threads created by main()
    * will inherit a copy of the signal mask. */

   sigemptyset(&set);
   sigaddset(&set, SIGINT);
   sigaddset(&set, SIGQUIT);
   i=pthread_sigmask(SIG_BLOCK, &set, NULL);
   i+=pthread_create(&sigThread, NULL, &sigHandler_thread, (void *) &set);
   if (i!=0) {
      fprintf(stderr,"signal handling init failed; exit.\n");
      exit(1);
   }

   i=pthread_mutex_init(&ctx.decBufLock, NULL);
   if (i!=0) {
      fprintf(stderr,"mutex init failed; exit.\n");
      exit(1);
   }

   atexit(exitHandler); /* Not called if interrupted by a signal */
   bcm_host_init();
   OERR(OMX_Init());
   OERR(OMX_GetHandle(&ctx.dec, DECNAME, &ctx, &decEventCallback));
   OERR(OMX_GetHandle(&ctx.enc, ENCNAME, &ctx, &encEventCallback));
   OERR(OMX_GetHandle(&ctx.rsz, RSZNAME, &ctx, &rszEventCallback));
   OERR(OMX_GetHandle(&ctx.dei, DEINAME, &ctx, &deiEventCallback));
   OERR(OMX_GetHandle(&ctx.spl, SPLNAME, &ctx, &splEventCallback));
   OERR(OMX_GetHandle(&ctx.vid, VIDNAME, &ctx, &vidEventCallback));

   openInputFile(&ctx);
   ctx.decbufs=configDecoder(&ctx);
   /* If there is extradata send it to the decoder to have a look at */
   if (ctx.ic->streams[ctx.inVidStreamIdx]->codecpar->extradata!=NULL
         && ctx.ic->streams[ctx.inVidStreamIdx]->codecpar->extradata_size>0) {
      fprintf(stderr, "** Found extradata in video stream...\n");
      spare=ctx.decbufs;
      if (ctx.ic->streams[ctx.inVidStreamIdx]->codecpar->extradata_size < spare->nAllocLen) {
         spare->nFilledLen=ctx.ic->streams[ctx.inVidStreamIdx]->codecpar->extradata_size;
         memcpy(spare->pBuffer, ctx.ic->streams[ctx.inVidStreamIdx]->codecpar->extradata, spare->nFilledLen);
         spare->nFlags=OMX_BUFFERFLAG_CODECCONFIG | OMX_BUFFERFLAG_ENDOFFRAME;
         OERR(OMX_EmptyThisBuffer(ctx.dec, spare));
      }
      else
         fprintf(stderr,"WARNING: extradata too big for input buffer - ignoring...\n");
   }

   ctx.audioPTS=0;
   ctx.videoPTS=0;
   /* Feed the decoder frames until the parameters are identified and port 131 changes state */
   for (j=0; ctx.state!=TUNNELSETUP; j++) {
      p=getNextVideoPacket(&ctx);
      if (p!=NULL) {
         fillDecBuffers(&ctx,j,p);
         av_packet_free(&p);
      }
      else
         ctx.state = DECEOF;
      if (j==120)
         ctx.state=DECFAILED;
   }
   switch (ctx.state) {
      case DECFAILED:
         fprintf(stderr, "ERROR: Failed to set the parameters after %d video frames.  Giving up.\n", j);
         exit(1);
         break;
      case DECEOF:
         fprintf(stderr, "ERROR: End of file before parameters could be set.\n");
         exit(1);
         break;
      case TUNNELSETUP:
         fprintf(stderr, "Identified the parameters after %d video frames.\n", j);
         start = time(NULL);
         configure(&ctx);
  
         fprintf(stderr, "OMX detected %lf fps\n", ctx.omxFPS);
         pthread_attr_init(&fpsa);
         pthread_attr_setdetachstate(&fpsa, PTHREAD_CREATE_DETACHED);
         pthread_create(&fpst, &fpsa, fps, NULL); /* Run fps calculator in another thread */
         break;
      case QUIT:
         exit(1);
      default:
         fprintf(stderr, "ERROR: System in an unexpected state: %i.\n", ctx.state);
         exit(1);
   }
   
   /* Main loop */
   for (i = j+1; ctx.state != QUIT; i++) {
      p = getNextVideoPacket(&ctx);
      if (p == NULL) break;
      fillDecBuffers(&ctx,i,p);
      av_packet_free(&p);
   } /* End of main loop */

   ctx.state = DECEOF;  /* Signal fps thread to finish */
   avformat_close_input(&ctx.ic);
   spare=getSpareDecBuffer(&ctx);
   spare->nFilledLen=0;
   spare->nOffset = 0;
   spare->nFlags=OMX_BUFFERFLAG_ENDOFFRAME | OMX_BUFFERFLAG_EOS | OMX_BUFFERFLAG_TIME_UNKNOWN;
   OERR(OMX_EmptyThisBuffer(ctx.dec, spare));

   /* Wait for encoder to finish processing */
   while (ctx.state != ENCEOS) {
      emptyEncoderBuffers(&ctx);
      usleep(10);
   }
   
   end = time(NULL);

   fprintf(stderr, "\nDropped frames: %f\%\n",100*(ctx.framesIn-ctx.framesOut)/ctx.framesIn);
   fprintf(stderr, "Processed %lli frames in %d seconds; %llif/s\n\n\n", ctx.framesOut, end-start, (ctx.framesOut/(end-start)));
   fprintf(stderr, "Time waiting for encoder to finish: %.2lfs\n",(double)ctx.encWaitTime*1E-5);
   
   if (ctx.oc) {
      av_write_trailer(ctx.oc);
      avio_close(ctx.oc->pb);
   }
   else
      close(ctx.raw_fd);

   av_free(ctx.nalEntry.nalBuf);
   pthread_mutex_destroy(&ctx.decBufLock);
   return 0;
}
