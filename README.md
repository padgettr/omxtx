omxtx
=====

This is a simple OpenMAX transcoder for the Raspberry Pi.

See the warnings in the comments at the top of omxtx.c

This version has been tested on a raspberry PI 3 running Arch Linux, ffmpeg version:

```
ffmpeg version 3.2.2 Copyright (c) 2000-2016 the FFmpeg developers
  built with gcc 6.2.1 (GCC) 20160830
  configuration: --prefix=/usr --disable-debug --disable-static --disable-stripping --enable-avisynth --enable-avresample --enable-fontconfig --enable-gmp --enable-gnutls --enable-gpl --enable-ladspa --enable-libass --enable-libbluray --enable-libfreetype --enable-libfribidi --enable-libgsm --enable-libiec61883 --enable-libmodplug --enable-libmp3lame --enable-libopencore_amrnb --enable-libopencore_amrwb --enable-libopenjpeg --enable-libopus --enable-libpulse --enable-libschroedinger --enable-libsoxr --enable-libspeex --enable-libssh --enable-libtheora --enable-libv4l2 --enable-libvidstab --enable-libvorbis --enable-libvpx --enable-libwebp --enable-libx264 --enable-libx265 --enable-libxvid --enable-netcdf --enable-shared --enable-version3 --enable-x11grab --host-cflags='"-fPIC"'
  libavutil      55. 34.100 / 55. 34.100
  libavcodec     57. 64.101 / 57. 64.101
  libavformat    57. 56.100 / 57. 56.100
  libavdevice    57.  1.100 / 57.  1.100
  libavfilter     6. 65.100 /  6. 65.100
  libavresample   3.  1.  0 /  3.  1.  0
  libswscale      4.  2.100 /  4.  2.100
  libswresample   2.  3.100 /  2.  3.100
  libpostproc    54.  1.100 / 54.  1.100
Hyper fast Audio and Video encoder
usage: ffmpeg [options] [[infile options] -i infile]... {[outfile options] outfile}...

Use -h to get full help or, even better, run 'man ffmpeg'
```

Also, some new functionality has been added: ability to crop the image using hardware, and an
auto-scaling function. This version is primarily aimed at converting dvd material,
and as such has only received significant testing for mpeg files ripped from dvd, and a target
format of h264 in a matroska container, e.g:

```
./omxtx test.vob -ay -c 720:480:0:80 test.mkv
```

I used this as a project to learn some openmax, so the code has been changed from the original a fair bit to aid
my understanding.

To try out vp8 decoding requires:
```
            start_file=start_x.elf
            fixup_file=fixup_x.dat
```
in /boot/config.txt


Dr. R. Padgett, November 2016

Copyright
=========

omxtx - OpenMAX transcoder for the Raspberry Pi
Copyright (C) 2012, 2013 Dickon Hood <dickon@fluff.org>

This version 2014 - 2016 Rodney Padgett <rod_padgett@hotmail.com>

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
