## Gstreamer Commands

```bash
# Decode mp4 using vaapi
gst-launch-1.0 filesrc location=/home/lei/Downloads/sample-5s.mp4 ! qtdemux ! vaapidecodebin ! autovideosink

gst-launch-1.0 filesrc location=/home/lei/Downloads/sample-5s.mp4 ! qtdemux ! vaapih264dec ! videoconvert ! autovideosink

# Decode mp4 using nvdec
gst-launch-1.0 filesrc location=/home/lei/Downloads/sample-5s.mp4 ! qtdemux ! nvh264dec ! videoconvert ! autovideosink

# Convert to Raw
ffmpeg -i ~/Downloads/sample-5s.mp4 -c:v rawvideo -pix_fmt yuv420p ~/Downloads/test.yuv

# Encode to mp4 using vaapi
gst-launch-1.0 -m filesrc location=~/Downloads/test.yuv ! videoparse width=1920 height=1080 framerate=30/1 format=i420 ! vaapih264enc ! filesink location=~/Downloads/test.mp4

gst-launch-1.0 -m filesrc location=~/Downloads/test.yuv ! videoparse width=1920 height=1080 framerate=30/1 format=i420 ! vaapih264enc rate-control=cbr tune=high-compression ! filesink location=~/Downloads/test.mp4

# Encode to mp4 using nvenc
gst-launch-1.0 -m filesrc location=~/Downloads/test.yuv ! videoparse width=1920 height=1080 framerate=30/1 format=i420 ! nvh264enc ! filesink location=~/Downloads/test.mp4
```