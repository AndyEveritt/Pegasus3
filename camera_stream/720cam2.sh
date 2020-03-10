gst-launch-1.0 -v v4l2src device=/dev/video1 ! "image/jpeg,width=1280, height=720,framerate=10/1" ! rtpjpegpay ! udpsink host=192.168.1.11 port=9001
