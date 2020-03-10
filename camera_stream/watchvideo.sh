gst-launch-1.0 -e -v udpsrc port=9001 ! application/x-rtp, encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegdec ! autovideosink

