gst-launch-1.0 -v openni2src ! "image/jpeg,width=1280, height=720,framerate=15/1" ! rtpjpegpay ! udpsink host=192.168.1.16 port=9001
