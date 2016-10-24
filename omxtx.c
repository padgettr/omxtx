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
 * compatibility with ffmpeg 3.0.2
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
 *        If the muxer produces errors anout non-monotonically increasing pts /dts, try using
 *        the option -p to change how pts is derived.  The default is to base the pts on the
 *        duration stored in the input packets; I have found this the most reliable. Note that
 *        Some streams appear to reset the input pts to zero after the start credits: the muxer
 *        doesn't like this!
 *
 *        ffmpeg docs: https://www.ffmpeg.org/documentation.html
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
//#include "libavcodec/avcodec.h"
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

static OMX_VERSIONTYPE SpecificationVersion = {
   .s.nVersionMajor = 1,
   .s.nVersionMinor = 1,
   .s.nRevision     = 2,
   .s.nStep         = 0
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
   uint8_t *sps;
   uint8_t *pps;
   int spsSize;
   int ppsSize;
   uint8_t *nalBuf;
   uint32_t nalBufSize; /* 32 bit: maximum buffer size 4GB! */
   off_t nalBufOffset;
   int64_t pts;
} OMXTX_NAL_ENTRY;

static struct context {
   AVFormatContext *ic;    /* Input context for demuxer */
   AVFormatContext *oc;    /* Output context for muxer */
   OMXTX_NAL_ENTRY nalEntry;  /* Store for NAL header info */
   int      raw_fd;        /* File descriptor for raw output file */
   volatile uint64_t framecount;
   volatile uint16_t userFlags;      /* User command line switch flags */
   volatile uint8_t componentFlags;
   OMX_BUFFERHEADERTYPE *encbufs;
   OMX_BUFFERHEADERTYPE *decbufs;
   volatile uint64_t encWaitTime;
   volatile int encBufferFilled;
   volatile enum states   state;
   int      inVidStreamIdx;
   int      inAudioStreamIdx; /* <0 if there is no audio stream */
   int      userAudioStreamIdx;
   int      ptsOpt;
   int64_t  audioPTS;
   int64_t  videoPTS;
   OMX_HANDLETYPE   dec, enc, rsz, dei, spl, vid;
   pthread_mutex_t decBufLock;
   pthread_mutex_t encBufLock;
   pthread_mutex_t cmpFlagLock;
   AVBitStreamFilterContext *bsfc;
   int          bitrate;
   volatile float omxFPS;
   char     *iname;
   char     *oname;
   AVRational omxtimebase;
   OMX_CONFIG_RECTTYPE *cropRect;
   int outputWidth;
   int outputHeight;
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
   printf("%s port %d is %s, %s\n", mapComponent(&ctx, handle), portdef->nPortIndex,
      (portdef->eDir == 0 ? "input" : "output"),
      (portdef->bEnabled == 0 ? "disabled" : "enabled"));
   printf("Wants %d bufs, needs %d, size %d, enabled: %d, pop: %d, aligned %d\n",
      portdef->nBufferCountActual,
      portdef->nBufferCountMin, portdef->nBufferSize,
      portdef->bEnabled, portdef->bPopulated,
      portdef->nBufferAlignment);

   switch (portdef->eDomain) {
   case OMX_PortDomainVideo:
      printf("Video type is currently:\n"
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
      printf("Image type is currently:\n"
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
      fprintf(stderr,"This port is not defined in this program!\n");
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

   printf("\n\nIn exit handler, after %lli frames:\n", ctx.framecount);
   if (ctx.userFlags & UFLAGS_VERBOSE) {
      dumpport(ctx.dec, PORT_DEC);
      dumpport(ctx.enc, PORT_ENC+1);
   }

   OMX_GetState(ctx.dec, &state);
   printf("Decoder state: %d\n", state);
   OMX_GetState(ctx.enc, &state);
   printf("Encoder state: %d\n", state);
   cleanup(&ctx);
}

/* Map libavcodec IDs to OMX IDs */
static int mapCodec(enum AVCodecID id) {
   printf("Mapping codec ID %d (%x)\n", id, id);
   switch (id) {
      case   AV_CODEC_ID_MPEG2VIDEO:
      case   AV_CODEC_ID_MPEG2VIDEO_XVMC:
         return OMX_VIDEO_CodingMPEG2;
      case   AV_CODEC_ID_H264:
         return OMX_VIDEO_CodingAVC;
      case   8:
         return OMX_VIDEO_CodingMJPEG;
      case   13:
         return OMX_VIDEO_CodingMPEG4;
      default:
         return -1;
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
static AVFormatContext *makeOutputContext(AVFormatContext *ic, const char *oname, int idx, const OMX_PARAM_PORTDEFINITIONTYPE *prt) {
   const OMX_VIDEO_PORTDEFINITIONTYPE *viddef;
   AVFormatContext   *oc=NULL;
   AVStream          *iflow, *oflow;

   viddef = &prt->format.video; /* Decoder output format structure */

   /* allocate avformat context - avformat_free_context() can be used to free */

   avformat_alloc_output_context2(&oc, NULL, NULL, oname);
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
   oflow->codecpar->bit_rate = ctx.bitrate;   /* User specified bit rate or default */
   oflow->codecpar->profile = FF_PROFILE_H264_HIGH;
   oflow->codecpar->level = 41;   /* The profile level: hard coded here to 4.1 */

   oflow->time_base = iflow->time_base;               /* Set timebase hint for muxer: will be overwritten on header write depending on container format */
   oflow->codecpar->format = AV_PIX_FMT_YUV420P; // TODO: should match up with OMX formats!
   oflow->avg_frame_rate = iflow->avg_frame_rate; /* Set framerate of output stream */
   oflow->r_frame_rate = iflow->r_frame_rate;
   oflow->start_time=iflow->start_time;

   if ((ctx.userFlags & UFLAGS_RESIZE)==0) { /* If resizing use default 1:1 pixel aspect, otherwise copy apect ratio from input */
      oflow->codecpar->sample_aspect_ratio.num = iflow->codecpar->sample_aspect_ratio.num;
      oflow->codecpar->sample_aspect_ratio.den = iflow->codecpar->sample_aspect_ratio.den;
      oflow->sample_aspect_ratio.num = iflow->codecpar->sample_aspect_ratio.num;
      oflow->sample_aspect_ratio.den = iflow->codecpar->sample_aspect_ratio.den;
   }
   //if (oc->oformat->flags & AVFMT_GLOBALHEADER)
   //   oflow->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

   printf("\n\n\n");
   if (ctx.inAudioStreamIdx>0) {
      printf("*** Mapping input video stream #%i to output video stream #%i ***\n", ctx.inVidStreamIdx, 0);
      printf("*** Mapping input audio stream #%i to output audio stream #%i ***\n\n", ctx.inAudioStreamIdx, 1);
      iflow = ic->streams[ctx.inAudioStreamIdx];
      oflow = avformat_new_stream(oc, NULL); /* Stream 1 */
      oflow->codecpar->codec_type = iflow->codecpar->codec_type;
      
      oflow->codecpar->codec_id = iflow->codecpar->codec_id;
      oflow->codecpar->codec_tag = iflow->codecpar->codec_tag;
      oflow->codecpar->format = iflow->codecpar->format;
      oflow->codecpar->bit_rate = iflow->codecpar->bit_rate;
      oflow->codecpar->bits_per_coded_sample = iflow->codecpar->bits_per_coded_sample;
      oflow->codecpar->bits_per_raw_sample = iflow->codecpar->bits_per_raw_sample;
      oflow->codecpar->channel_layout = iflow->codecpar->channel_layout;
      oflow->codecpar->channels = iflow->codecpar->channels;
      oflow->codecpar->sample_rate = iflow->codecpar->sample_rate;
      oflow->codecpar->block_align = iflow->codecpar->block_align;
      oflow->codecpar->frame_size = iflow->codecpar->frame_size;
      oflow->codecpar->initial_padding = iflow->codecpar->initial_padding;
      oflow->codecpar->trailing_padding = iflow->codecpar->trailing_padding;
      oflow->codecpar->seek_preroll = iflow->codecpar->seek_preroll;
      oflow->time_base = iflow->time_base; /* Time base hint */
      oflow->start_time=iflow->start_time;

      //if (oc->oformat->flags & AVFMT_GLOBALHEADER)
      //   oflow->codecpar->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
   }

   printf("Input:\n");
   av_dump_format(ic, 0, ic->filename, 0);
   printf("\nOutput:\n");
   av_dump_format(oc, 0, oname, 1);


   return oc;
}

static void writeAudioPacket(AVPacket *pkt) {
   int ret;

   pkt->stream_index=1;

   switch (ctx.ptsOpt) {
      case 0: ctx.audioPTS=pkt->pts; break; /* Use packet pts */
      case 1: ctx.audioPTS=pkt->pts; break; /* Use packet pts: dts not used for audio */
      case 2: ctx.audioPTS+=pkt->duration; break; /* Use packet duration */
      default: ctx.audioPTS+=pkt->duration; break;
   }
   pkt->pts = av_rescale_q_rnd(ctx.audioPTS,
              ctx.ic->streams[ctx.inAudioStreamIdx]->time_base,
              ctx.oc->streams[1]->time_base,
              AV_ROUND_NEAR_INF|AV_ROUND_PASS_MINMAX);
   pkt->dts=pkt->pts; /* Audio packet: dts=pts */

   // fprintf(stderr,"Audio PTS %lld\n", pkt->pts);
   ret=av_interleaved_write_frame(ctx.oc, pkt);   /* This frees pkt */
   if (ret < 0) {
      fprintf(stderr, "Failed to write non-video frame.\n");
   }
}

static int openOutput(struct context *ctx, int nalType) {
   int i, ret;
   struct packetentry *packet, *next;
   AVFormatContext *oc;
   AVCodecParameters *cc;

   if (nalType == 7) {          /* This NAL is of type sequence parameter set */
      ctx->nalEntry.sps = ctx->nalEntry.nalBuf; /* Take ref */
      ctx->nalEntry.nalBuf=av_malloc(ctx->nalEntry.nalBufSize); /* Alloc new memory for nal data */
      if (ctx->nalEntry.nalBuf==NULL) {
         fprintf(stderr,"ERROR: Can't allocate memory for nalBuf\n");
         exit(1);
      }
      ctx->nalEntry.spsSize = ctx->nalEntry.nalBufOffset;
      printf("New SPS, length %d\n", ctx->nalEntry.spsSize);
   }
   else if (nalType == 8) {   /* This NAL is of type picture parameter set */
      ctx->nalEntry.pps = ctx->nalEntry.nalBuf;
      ctx->nalEntry.nalBuf=av_malloc(ctx->nalEntry.nalBufSize);
      if (ctx->nalEntry.nalBuf==NULL) {
         fprintf(stderr,"ERROR: Can't allocate memory for nalBuf\n");
         exit(1);
      }
      ctx->nalEntry.ppsSize = ctx->nalEntry.nalBufOffset;
      printf("New PPS, length %d\n", ctx->nalEntry.ppsSize);
   }

   if (ctx->nalEntry.pps==NULL || ctx->nalEntry.sps==NULL)
      return 1; /* Can't open output yet: no pps / sps data */

   printf("Got SPS and PPS data: opening output file '%s'\n", ctx->oname);
   oc = ctx->oc;
   cc = ctx->oc->streams[0]->codecpar;

   if (cc->extradata) { /* Delete any existing data */
      av_free(cc->extradata);
      cc->extradata = NULL;
      cc->extradata_size = 0;
   }
            
   cc->extradata_size = ctx->nalEntry.ppsSize + ctx->nalEntry.spsSize;
   cc->extradata = av_malloc(cc->extradata_size);
   memcpy(cc->extradata, ctx->nalEntry.sps, ctx->nalEntry.spsSize);
   memcpy(&cc->extradata[ctx->nalEntry.spsSize], ctx->nalEntry.pps, ctx->nalEntry.ppsSize);
         
   if (!(oc->oformat->flags & AVFMT_NOFILE)) {
     ret = avio_open(&oc->pb, ctx->oname, AVIO_FLAG_WRITE);
     if (ret < 0) {
         fprintf(stderr, "Could not open output file '%s'\n", ctx->oname);
         exit(1);
     }
   }
   /* init muxer, write output file header */
   ret = avformat_write_header(oc, NULL);
   if (ret < 0) {
     av_log(NULL, AV_LOG_ERROR, "Error occurred when opening output file\n");
     exit(1);
   }
   
   av_free(ctx->nalEntry.sps);
   av_free(ctx->nalEntry.pps);
   ctx->nalEntry.spsSize=0;
   ctx->nalEntry.ppsSize=0;

   if (ctx->inAudioStreamIdx>0) {
      printf("Writing saved audio packets out...");
      for (i = 0, packet = TAILQ_FIRST(&packetq); packet; packet = next) {
         next = TAILQ_NEXT(packet, link);
         writeAudioPacket(packet->packet);
         i++;
         TAILQ_REMOVE(&packetq, packet, link);
         av_packet_free(&packet->packet);
      }

      printf("done.  Wrote %d frames.\n\n", i);
   }
   printf("Press ctrl-c to abort\n");
   return 0;
}

OMX_ERRORTYPE genericEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {

   if (ctx->userFlags & UFLAGS_VERBOSE) {
      printf("Got an unhandled event of type %s (%x) on %s (d1: %x, d2: %x)\n",
         mapEvents(event), event, mapComponent(ctx, handle), data1, data2);
   }
   return OMX_ErrorNone;
}

OMX_ERRORTYPE decEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   pthread_mutex_lock(&ctx->cmpFlagLock);
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
   pthread_mutex_unlock(&ctx->cmpFlagLock);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE encEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   pthread_mutex_lock(&ctx->cmpFlagLock);
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
   pthread_mutex_unlock(&ctx->cmpFlagLock);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE rszEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   pthread_mutex_lock(&ctx->cmpFlagLock);
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
   pthread_mutex_unlock(&ctx->cmpFlagLock);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE deiEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   pthread_mutex_lock(&ctx->cmpFlagLock);
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
   pthread_mutex_unlock(&ctx->cmpFlagLock);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE splEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   pthread_mutex_lock(&ctx->cmpFlagLock);
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
   pthread_mutex_unlock(&ctx->cmpFlagLock);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE vidEventHandler(OMX_HANDLETYPE handle, struct context *ctx, OMX_EVENTTYPE event, OMX_U32 data1, OMX_U32 data2, OMX_PTR eventdata) {
   pthread_mutex_lock(&ctx->cmpFlagLock);
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
   pthread_mutex_unlock(&ctx->cmpFlagLock);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE genericBufferCallback(OMX_HANDLETYPE handle, struct context *ctx, OMX_BUFFERHEADERTYPE *buf) {
   printf("Got a buffer event on %s %p, buf %p\n", mapComponent(ctx, handle), handle, buf);
   return OMX_ErrorNone;
}

OMX_ERRORTYPE emptied(OMX_HANDLETYPE handle, struct context *ctx, OMX_BUFFERHEADERTYPE *buf) {
   #ifdef DEBUG
      printf("Got a buffer emptied event on %s %p, buf %p\n", mapComponent(ctx, handle), handle, buf);
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
      printf("Got a buffer filled event on %s %p, buf %p\n", mapComponent(ctx, handle), handle, buf);
   #endif
   pthread_mutex_lock(&ctx->encBufLock);
   ctx->encBufferFilled=1;
   pthread_mutex_unlock(&ctx->encBufLock);
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
      lastframe = ctx.framecount;
      sleep(1);
      printf("Frame %6lld (%5.2fs).  Frames last second: %lli     \r",
         ctx.framecount, (float)ctx.framecount/ctx.omxFPS, ctx.framecount-lastframe);
      fflush(stdout);
   }
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

   printf("Allocate %i %s buffers of %d bytes\n", portdef->nBufferCountActual, mapComponent(&ctx, h), portdef->nBufferSize);
   for (i = 0; i < portdef->nBufferCountActual; i++) {
      //OMX_U8 *buf= vcos_malloc_aligned(portdef->nBufferSize, portdef->nBufferAlignment, "buffer");
      //OERR(OMX_UseBuffer(h, end, port, NULL, portdef->nBufferSize, buf));
      OERR(OMX_AllocateBuffer(h, end, port, NULL, portdef->nBufferSize));
      end = (OMX_BUFFERHEADERTYPE **) &((*end)->pAppPrivate);
   }

   free(portdef);
   return list;
}

static int dofiltertest(AVPacket *rp) {

   if (!(rp->buf->data[0] == 0x00 && rp->buf->data[1] == 0x00 && rp->buf->data[2] == 0x00 && rp->buf->data[3] == 0x01)) {
      printf("h264 input data required to be in annex b format. Use, e.g:\n");
      printf("\tffmpeg -i INPUT.mp4 -codec copy -bsf:v h264_mp4toannexb OUTPUT.ts\n");
      printf("to convert before processing.\n");
      return 1;
   }

   return 0;
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
   pthread_mutex_lock(&ctx.cmpFlagLock);
   ctx.componentFlags|=cFlag;
   pthread_mutex_unlock(&ctx.cmpFlagLock);
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
   image_filter->nParams[0] = 3;
   image_filter->nParams[1] = 0; // default frame interval
   image_filter->nParams[2] = 1; // half framerate
   image_filter->nParams[3] = 1; // use qpus
   image_filter->eImageFilter = OMX_ImageFilterDeInterlaceFast;
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
   int pp;

   MAKEME(portdef, OMX_PARAM_PORTDEFINITIONTYPE);

   printf("Decoder has changed settings.  Setting up encoder.\n");

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
   if (ctx->bitrate == 0)
      viddef->nBitrate = (2*1024*1024);
   else
      viddef->nBitrate = ctx->bitrate;
   ctx->bitrate = viddef->nBitrate;

   viddef->eCompressionFormat = OMX_VIDEO_CodingAVC;
   viddef->nStride = viddef->nSliceHeight = viddef->eColorFormat = 0;
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

   if (ctx->userFlags & UFLAGS_VERBOSE) {
      MAKEME(level, OMX_VIDEO_PARAM_PROFILELEVELTYPE);
      level->nPortIndex = PORT_ENC+1;
      OERR(OMX_GetParameter(ctx->enc, OMX_IndexParamVideoProfileLevelCurrent, level));
      printf("OMX H264 Encoder: Current level:\t\t%x\nCurrent profile:\t%x\n", level->eLevel, level->eProfile);
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

   /* Make an output context if output is not raw: */
   if ((ctx->userFlags & UFLAGS_RAW) == 0) {
      ctx->oc = makeOutputContext(ctx->ic, ctx->oname, ctx->inVidStreamIdx, portdef);
      if (!ctx->oc) {
         fprintf(stderr, "Create output AVFormatContext failed.\n");
         exit(1);
      }
   }
   ctx->omxFPS=(float)portdef->format.video.xFramerate/(float)(1<<16);
   ctx->state=OPENOUTPUT;
}

static OMX_BUFFERHEADERTYPE *configDecoder(struct context *ctx) {
   OMX_PARAM_PORTDEFINITIONTYPE   *portdef;
   OMX_VIDEO_PORTDEFINITIONTYPE   *viddef;
   OMX_BUFFERHEADERTYPE   *decbufs;

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

   OERR(OMX_SetParameter(ctx->dec, OMX_IndexParamPortDefinition, portdef));

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
   fprintf(stderr, "Usage: %s [-a[y]] [-b bitrate] [-c crop] [-d] [-m] [-r size] [-x] <infile> <outfile>\n\n"
      "Where:\n"
      "   -a[y] Auto scale the video stream to produce a sample aspect ratio (pixel aspect ratio)\n"
      "         of 1:1. By default the scaling is in the x-direction (image width); this usually\n"
      "         results more (interpolated) pixels in the x-direction. If y is specified, the scaling\n"
      "         is done in the y-direction; this usually results in a reduction in resolution in the\n"
      "         y-direction.\n"
      "   -b n  Target bitrate n[k|M] in bits/second (default: 2Mb/s)\n"
      "   -c c  Crop: 'c' is specified in pixels as width:height:left:top\n"
      "   -d    Deinterlace\n"
      "   -i n  Select audio stream n.\n"
      "   -m    Monitor.  Display the decoder's output\n"
      "   -p n  Select how the timestamps are calculated:\n"
      "            n=0: use input frame pts\n"
      "            n=1: use input frame dts\n"
      "            n=2: use input frame duration (default)\n"
      "   -r s  Resize: 's' is in pixels specified as widthxheight\n"
      "   -v    Verbose\n"
      "\n"
      "Output container is guessed based on filename.  Use '.nal' for raw output.\n"
      "\n"
      "Input file must contain one of MPEG 2, H.264, or MJPEG video\n"
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

static void setupUserOpts(struct context *ctx, int argc, char *argv[]) {
   int opt, j;

   if (argc < 3)
      usage(argv[0]);

   ctx->bitrate = 2*1024*1024;   /* Default: 2Mb/s */
   ctx->userAudioStreamIdx=-1;   /* Default: guess audio stream */
   ctx->ptsOpt=2;                /* Default: Use duration to determine pts */
   while ((opt = getopt(argc, argv, "a::b:c:di:mp:r:v")) != -1) {
      switch (opt) {
      case 'a':
         if (optarg==NULL)
            ctx->userFlags |= UFLAGS_AUTO_SCALE_X;
         else if (*optarg=='y')
            ctx->userFlags |= UFLAGS_AUTO_SCALE_Y;
         else
            ctx->userFlags |= UFLAGS_AUTO_SCALE_X;
         ctx->userFlags |= UFLAGS_RESIZE;
      break;
      case 'b':
         ctx->bitrate = parsebitrate(optarg, ctx->bitrate);
      break;
      case 'c':
         if (setCropRectangle(ctx,optarg)==0)
            ctx->userFlags |= UFLAGS_CROP;
         else
            exit(1);
      break;
      case 'd':
         ctx->userFlags |= UFLAGS_DEINTERLACE;
      break;
      case 'i':
         ctx->userAudioStreamIdx=atoi(optarg);
      break;
      case 'm':
         ctx->userFlags |= UFLAGS_MONITOR;
      break;
      case 'p':
         ctx->ptsOpt=atoi(optarg);
      break;
      case 'r':
         if (setOutputSize(ctx,optarg)==0)
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
  //pkt.buf = av_buffer_create(pkt.data, pkt.size, av_buffer_default_free, NULL, 0); // TODO: ctx->nalEntry.nalBuf can't be used here because it gets freed in av_interleaved_write_frame()
   pkt.pts = av_rescale_q(ctx->nalEntry.pts, ctx->omxtimebase, ctx->oc->streams[0]->time_base); /* Transform omx pts to output timebase */
   pkt.dts=pkt.pts; /* Out of order b-frames not supported on rpi: so dts=pts */
   //fprintf(stderr,"pts: pkt:%lld, OMX:%lld\n",pkt.pts, ctx->nalEntry.pts);

   if (nalType==5)   /* This is an IDR frame */
      pkt.flags |= AV_PKT_FLAG_KEY;
   //if (pkt.buf != NULL)
      r = av_interleaved_write_frame(ctx->oc, &pkt);
   if (r != 0) {
      char err[256];
      av_strerror(r, err, sizeof(err));
      fprintf(stderr,"\nFailed to write a video frame: %s (pts: %lld; nal: %i)\n", err, ctx->nalEntry.pts, nalType);
   }
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
   int nalType;
   size_t curNalSize;
   int i;

   pthread_mutex_lock(&ctx->encBufLock);
   i=ctx->encBufferFilled;
   pthread_mutex_unlock(&ctx->encBufLock);
   if (i==0) {
      ctx->encWaitTime++;
      return;  /* Buffer is empty - return to main loop */
   }
   if (ctx->encbufs->nFlags & OMX_BUFFERFLAG_EOS) {
      ctx->state=ENCEOS;
      return;
   }

   if (ctx->userFlags & UFLAGS_RAW) {
      write(ctx->raw_fd, &ctx->encbufs->pBuffer[ctx->encbufs->nOffset], ctx->encbufs->nFilledLen);
   }
   else {
      curNalSize=ctx->nalEntry.nalBufOffset+ctx->encbufs->nFilledLen;
      if (curNalSize>ctx->nalEntry.nalBufSize) {
         fprintf(stderr, "\nERROR: nalBufSize exceeded.\n");
         exit(1);
      }
      memcpy(ctx->nalEntry.nalBuf + ctx->nalEntry.nalBufOffset, ctx->encbufs->pBuffer + ctx->encbufs->nOffset, ctx->encbufs->nFilledLen);
      ctx->nalEntry.nalBufOffset=curNalSize;
      ctx->nalEntry.pts=((((int64_t) ctx->encbufs->nTimeStamp.nHighPart)<<32) | ctx->encbufs->nTimeStamp.nLowPart);

      if (ctx->encbufs->nFlags & OMX_BUFFERFLAG_ENDOFNAL) { /* At end of nal */
         nalType=examineNAL(ctx);

         if (ctx->state==OPENOUTPUT) {
            if (openOutput(ctx, nalType)==0) /* returns 0 if both sps and pps are known */
               ctx->state = RUNNING;
         }
         else /* Output file is open */
            writeVideoPacket(ctx, nalType);
         ctx->nalEntry.nalBufOffset = 0;  /* Flag that the buffer is empty */
      }
   }
   pthread_mutex_lock(&ctx->encBufLock); /* Lock not really required as we haven't called OMX_FillThisBuffer() yet and only one buffer, but just in case */
   ctx->encBufferFilled=0;
   pthread_mutex_unlock(&ctx->encBufLock);
   ctx->encbufs->nFilledLen = 0;
   ctx->encbufs->nOffset = 0;
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

int main(int argc, char *argv[]) {
   int i, j;
   time_t start, end;
   int offset;
   AVPacket *p=NULL, *rp=NULL;
   int AV_videoCodecID;
   int filtertest;
   int size, nsize;
   OMX_BUFFERHEADERTYPE *spare;
   OMX_TICKS tick;
   int64_t omt;
   pthread_t fpst;
   pthread_attr_t fpsa;
   sigset_t set;
   pthread_t sigThread;

   setupUserOpts(&ctx, argc, argv);
   ctx.omxtimebase.num=1;
   ctx.omxtimebase.den=1000000; /* OMX timebase is in micro seconds */
   ctx.nalEntry.sps=NULL;
   ctx.nalEntry.pps=NULL;
   ctx.nalEntry.spsSize=0;
   ctx.nalEntry.ppsSize=0;
   ctx.nalEntry.nalBufSize=1024*1024*2; /* 2MB buffer */
   ctx.nalEntry.nalBuf=av_malloc(ctx.nalEntry.nalBufSize);
   if (ctx.nalEntry.nalBuf==NULL) {
      fprintf(stderr,"ERROR: Can't allocate memory for nalBuf\n");
      exit(1);
   }
   ctx.nalEntry.nalBufOffset=0;
   ctx.framecount=0;
   ctx.componentFlags=0;
   ctx.componentFlags=0;
   ctx.encBufferFilled=0;

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
   i+=pthread_mutex_init(&ctx.encBufLock, NULL);
   i+=pthread_mutex_init(&ctx.cmpFlagLock, NULL);
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

   AV_videoCodecID=openInputFile(&ctx);
   ctx.decbufs=configDecoder(&ctx);
   
   filtertest = (AV_videoCodecID==AV_CODEC_ID_H264); /* Check for annex b input stream */

   ctx.audioPTS=ctx.ic->streams[ctx.inVidStreamIdx]->start_time;
   ctx.videoPTS=ctx.ic->streams[ctx.inVidStreamIdx]->start_time;
   switch (ctx.ptsOpt) {
      case 0:
         printf("Using frame pts for timestamps\n");
      break;
      case 1:
         printf("Using frame dts for timestamps\n");
      break;
      case 2:
         printf("Using frame duration to calculate timestamps\n");
      break;
      default:
         printf("Invalid selection for option -p: using defaults (frame duration)\n");
         ctx.ptsOpt=2;
      break;
   }

   for (i = j = 0; ctx.state != ENCEOS; i++, j++) {
      if (ctx.state!=DECEOF) {
         if (ctx.state!=QUIT)
            rp = getNextVideoPacket(&ctx);
         if (rp == NULL) {         /* If end of file, keep sending zeroed out packets until EOS on encoder */
            ctx.state = DECEOF;
            avformat_close_input(&ctx.ic);
            if (av_buffer_is_writable(p->buf)==1)
               memset(p->buf->data, 0, p->buf->size); /* Clear packet data; send OMX_BUFFERFLAG_EOS with this empty packet (below) */
            else
               fprintf(stderr, "Can't zero last packet buffer for flush!\n");
         }
         else {
            av_packet_free(&p); /* Free previous packet from decoder */
            p = rp;
            rp=NULL; /* Ref is passed to p */
            if (filtertest) { /* If input is h264, expecting annex B */
               filtertest = 0; // Don't do this next time round
               if(dofiltertest(p)==1) exit(1);
            }
            
            /* From ffmpeg docs: pkt->pts can be AV_NOPTS_VALUE (-9223372036854775808) if the video format has B-frames, so it is better to rely on pkt->dts if you do not decompress the payload */
            switch (ctx.ptsOpt) {
               case 0: ctx.videoPTS=p->pts; break; /* Use packet pts */
               case 1: ctx.videoPTS=p->dts; break; /* Use packet dts */
               case 2: ctx.videoPTS+=p->duration; break; /* Use packet duration */
               default: ctx.videoPTS+=p->duration; break;
            }
            omt = av_rescale_q(ctx.videoPTS, ctx.ic->streams[ctx.inVidStreamIdx]->time_base, ctx.omxtimebase); /* Transform input timebase to omx timebase */
            //fprintf(stderr,"Input pts: %lld; timebase: %i/%i\n", ctx.videoPTS, ctx.ic->streams[ctx.inVidStreamIdx]->time_base.num, ctx.ic->streams[ctx.inVidStreamIdx]->time_base.den);
         }
      }

/* Encoder setup when stream identified, indicated by a state change on port 131, then configure() sets up pipelines */
      switch (ctx.state) {
      case TUNNELSETUP:
         printf("Identified the parameters after %d video frames.\n", i);
         start = time(NULL);
         configure(&ctx);
         pthread_attr_init(&fpsa);
         pthread_attr_setdetachstate(&fpsa, PTHREAD_CREATE_DETACHED);
         pthread_create(&fpst, &fpsa, fps, NULL); /* Run fps calculator in another thread */
         break;
      case DECINIT:
         if (i < 120) /* Keep going if number of frames < 120; otherwise bail; port 131 has not changed state; decoder doesn't like it */
            break;
         ctx.state = DECFAILED;
         /* Drop through */
      case DECFAILED:
         fprintf(stderr, "Failed to set the parameters after %d video frames.  Giving up.\n", i);
         exit(1);
         break;
      default:
         break;   /* Shuts the compiler up */
      }

      tick.nLowPart = (uint32_t) (omt & 0xffffffff);
      tick.nHighPart = (uint32_t) ((omt & 0xffffffff00000000) >> 32);

      size = p->buf->size;
      ctx.framecount++;
      offset = 0;
      while (size>0) {
         do {
            emptyEncoderBuffers(&ctx);
            spare = ctx.decbufs;
            while (spare!=NULL && spare->nFilledLen != 0) /* Find a free buffer (indicated by nFilledLen == 0); if spare==NULL all buffers are used (pAppPrivate of last buffer is NULL) */
               spare = spare->pAppPrivate;
            usleep(10);
         } while (spare==NULL);

         if (ctx.state == ENCEOS) /* Got EOS at encoder output: finished! */
            break;
         spare->nFlags = i == 0 ? OMX_BUFFERFLAG_STARTTIME : 0; /* If it is first frame set start time flag, otherwise clear flags */
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
         if (ctx.state==DECEOF)
             spare->nFlags|=OMX_BUFFERFLAG_EOS;
         spare->nTimeStamp = tick;
         pthread_mutex_lock(&ctx.decBufLock);
         spare->nFilledLen = nsize;
         pthread_mutex_unlock(&ctx.decBufLock);
         spare->nOffset = 0;
         OERR(OMX_EmptyThisBuffer(ctx.dec, spare));
         size -= nsize;
         offset += nsize;
      }
   } /* End of main loop */

   av_packet_free(&p);
   end = time(NULL);

   printf("\nProcessed %lli frames in %d seconds; %llif/s\n\n\n", ctx.framecount, end-start, (ctx.framecount/(end-start)));
   printf("Number of wasted encoder cycles: %lli\n",ctx.encWaitTime);
   
   if (ctx.oc) {
      av_write_trailer(ctx.oc);
      avio_close(ctx.oc->pb);
   }
   else
      close(ctx.raw_fd);

   av_free(ctx.nalEntry.nalBuf);
   pthread_mutex_destroy(&ctx.decBufLock);
   pthread_mutex_destroy(&ctx.encBufLock);
   pthread_mutex_destroy(&ctx.cmpFlagLock);
   return 0;
}
