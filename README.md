omxtx
=====

This is a simple proof-of-concept OpenMAX transcoder for the Raspberry Pi.

See the warnings in the comments at the top of omxtx.c

This version has been tested on a raspberry PI 2 running Arch Linux, ffmpeg version:

ffmpeg version 3.0.2 Copyright (c) 2000-2016 the FFmpeg developers
  built with gcc 5.3.0 (GCC)
  configuration: --prefix=/usr --disable-debug --disable-static --disable-stripping --enable-avisynth --enable-avresample --enable-fontconfig --enable-gnutls --enable-gpl --enable-ladspa --enable-libass --enable-libbluray --enable-libdcadec --enable-libfreetype --enable-libfribidi --enable-libgsm --enable-libiec61883 --enable-libmodplug --enable-libmp3lame --enable-libopencore_amrnb --enable-libopencore_amrwb --enable-libopenjpeg --enable-libopus --enable-libpulse --enable-libschroedinger --enable-libsoxr --enable-libspeex --enable-libssh --enable-libtheora --enable-libv4l2 --enable-libvidstab --enable-libvorbis --enable-libvpx --enable-libwebp --enable-libx264 --enable-libx265 --enable-libxvid --enable-netcdf --enable-shared --enable-version3 --enable-x11grab --host-cflags='"-fPIC"'
  libavutil      55. 17.103 / 55. 17.103
  libavcodec     57. 24.102 / 57. 24.102
  libavformat    57. 25.100 / 57. 25.100
  libavdevice    57.  0.101 / 57.  0.101
  libavfilter     6. 31.100 /  6. 31.100
  libavresample   3.  0.  0 /  3.  0.  0
  libswscale      4.  0.100 /  4.  0.100
  libswresample   2.  0.101 /  2.  0.101
  libpostproc    54.  0.100 / 54.  0.100
Hyper fast Audio and Video encoder
usage: ffmpeg [options] [[infile options] -i infile]... {[outfile options] outfile}...

Use -h to get full help or, even better, run 'man ffmpeg'

Also, some new functionality has been added: ability to crop the image using hardware, and an
auto-scaling function. This version is primarily aimed at converting dvd material,
and as such has only received significant testing for mpeg files ripped from dvd, and a target
format of h264 in a matroska container.
I used this as a project to learn some openmax, so the code has been changed a fair bit to aid
my understanding.

Dr. R. Padgett, May 2016

Copyright
=========

omxtx - OpenMAX transcoder for the Raspberry Pi
Copyright (C) 2012, 2013 Dickon Hood <dickon@fluff.org>

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
