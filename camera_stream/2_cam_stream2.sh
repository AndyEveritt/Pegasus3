#!/bin/bash -x

gst-launch-1.0 tee name=stream v4l2src device=/dev/video1 \
  ! image/jpeg,width=800,height=600,framerate=30/1 ! jpegparse \
  ! rtpjpegpay ! udpsink host=192.168.1.16 port=11112 #.\
  #! v4l2src device=/dev/video1 \
  #! image/jpeg,width=800,height=600,framerate=30/1 ! jpegparse ! rtpjpegpay \
  #! udpsink host=192.168.1.12 port=11111 stream.
