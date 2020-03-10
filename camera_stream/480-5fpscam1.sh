gst-launch-1.0 -v v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480,framerate=5/1 \
  ! rtpjpegpay ! udpsink host=192.168.1.12 port=11111
