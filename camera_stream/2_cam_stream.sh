#!/bin/bash -x
#
# Script to start RPi Compute Module streaming over RTP (RFC3984)
# from both cameras
#
FPS=15                          # Frames per second
WIDTH=640                      # Image width
HEIGHT=480                      # Image height
UPLINK_HOST=192.168.1.100       # Receiving host
PORT=1234                       # UDP port
#
# TESTING WITH ONE CAMERA ONLY FOR THE MOMENT
#
function start_streaming
{
  gst-launch-1.0 -ve videomixer name=mixer \
    sink_00::xpos=0 sink_01::xpos=$WIDTH \
  v4l2src device=/dev/cameraARM \
  ! image/jpeg,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 \
  ! rtpjpegpay pt=96 \
  ! queue ! mixer.
  v4l2src device=/dev/cameraMAST \
  ! image/jpeg,format=MJPG,width=$WIDTH,height=$HEIGHT,framerate=$FPS/1 \
  ! rtpjpegpay pt=96 \
  ! queue ! mixer. \
  ! queue \
  ! udpsink host=$UPLINK_HOST port=$PORT 
}

# Start streaming on both cameras simultaneously
echo Image size: $WIDTH x $HEIGHT
echo Frame rate: $FPS
echo Starting cameras 0 and 1 streaming to $UPLINK_HOST:$PORT
start_streaming

# Wait until everything has finished
wait

exit 0
