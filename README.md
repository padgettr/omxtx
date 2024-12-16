omxtx
=====
UPDATE 16-12-2024:
This project has not been tested for a long time and will likely not work on recent versions of linux for the raspberry pi (rpi). Instead, I suggest the ffmpeg interface for the video for linux drivers recently added to the rpi kernel. For example:
ffmpeg -y -i input.mkv -r 25 -filter:v crop=720:416:0:80,scale=1024x416 -c:v h264_v4l2m2m -num_capture_buffers 128 -qmin 24 -qmax 31 -b:v 10M -c:a copy out.mp4

Note that only some output containers will work (mkv reported errors, mp4 was OK) and only certain video sizes are valid (encodes, but corrupted). From my experiments (end of 2022) the following applies:
        Scaled widths:
            1024x416: works (aspect 2.4:1)
            992x416:  doesn't work - corrupted and green background
            960x416:  works (aspect 2.3:1)
            928x416: doesn't work
            896x416: works
            *** So it looks like width/32 must be an even number ***
            As for height - any even height works. If a non even height is used, ffmpeg corrects to previous even number without warning.
            
            
Output to mkv doesn't work: Failed to set gop size: Invalid argument. Could not write header for output file #0 (incorrect codec parameters ?): Invalid data found when processing input

*** End of update ***

This project is forked from github: dickontoo/omxtx "a simple OpenMAX transcoder for the Raspberry Pi".
Thanks go to dickontoo for the concept and original code!

See the warnings in the comments at the top of omxtx.c

This version has been tested on a raspberry PI 3 running Arch Linux, ffmpeg version:

```
ffmpeg version n4.1 Copyright (c) 2000-2018 the FFmpeg developers
  built with gcc 8.2.0 (GCC)
  configuration: --prefix=/usr --disable-debug --disable-static --disable-stripping --enable-fontconfig --enable-gmp --enable-gnutls --enable-gpl --enable-ladspa --enable-libass --enable-libbluray --enable-libdrm --enable-libfreetype --enable-libfribidi --enable-libgsm --enable-libiec61883 --enable-libjack --enable-libmodplug --enable-libmp3lame --enable-libopencore_amrnb --enable-libopencore_amrwb --enable-libopenjpeg --enable-libopus --enable-libpulse --enable-libsoxr --enable-libspeex --enable-libssh --enable-libtheora --enable-libv4l2 --enable-libvidstab --enable-libvorbis --enable-libvpx --enable-libwebp --enable-libx264 --enable-libx265 --enable-libxcb --enable-libxml2 --enable-libxvid --enable-omx --enable-shared --enable-version3 --host-cflags='"-fPIC"'
  libavutil      56. 22.100 / 56. 22.100
  libavcodec     58. 35.100 / 58. 35.100
  libavformat    58. 20.100 / 58. 20.100
  libavdevice    58.  5.100 / 58.  5.100
  libavfilter     7. 40.101 /  7. 40.101
  libswscale      5.  3.100 /  5.  3.100
  libswresample   3.  3.100 /  3.  3.100
  libpostproc    55.  3.100 / 55.  3.100
Hyper fast Audio and Video encoder
usage: ffmpeg [options] [[infile options] -i infile]... {[outfile options] outfile}...

Use -h to get full help or, even better, run 'man ffmpeg'

```
Check the Makefile for CFLAGS to set for older versions of ffmpeg!

Also, some new functionality has been added: ability to crop the image using hardware, and an
auto-scaling function. This version is primarily aimed at converting dvd material,
and as such has only received significant testing for mpeg files ripped from dvd, and a target
format of h264 in a matroska container, e.g:

```
./omxtx test.vob -ay -c 720:480:0:80 -o test.mkv
```
Run omxtx -h for a full list of options and usage.

I used this as a project to learn some openmax, so the code has been changed from the original a fair bit to aid
my understanding.

To try out vp8 decoding requires:
```
            start_file=start_x.elf
            fixup_file=fixup_x.dat
```
in /boot/config.txt

This version has an enhanced deinterlace capability, and can output 1 frame per field as a user option.
It has been tested with mpeg2 avi/mkv, divx avi, mjpeg avi input all to h264 mkv output.
WARNING: Some audio desync was noted with pcm audio input streams! All AC3 was OK.

Dr. R. Padgett, December 2019

Copyright
=========

omxtx - OpenMAX transcoder for the Raspberry Pi
Copyright (C) 2012, 2013 Dickon Hood <dickon@fluff.org>

This version 2014 - 2019 Rodney Padgett <rod_padgett@hotmail.com>

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
