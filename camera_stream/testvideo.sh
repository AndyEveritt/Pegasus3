gst-launch-1.0 -v videotestsrc ! video/x-yuyv,width=640,height=480 ! udpsink host=192.168.1.12 port=9001
