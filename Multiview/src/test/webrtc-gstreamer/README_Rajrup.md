# webrtc-gstreamer

Original repo: <https://github.com/ethand91/webrtc-gstreamer>

Modifed:

- Replaced webcam with `videotestsrc` with flying ball pattern.
- Replaced video playback in `public/index.html` file to fix playback error on the browser.

## Build

```bash
mkdir build && cd build
cmake ..
make
```

## Run

```bash
# In terminal 1
./webrtc_server 

# In terminal 2
cd public
python3.8 -m http.server 9999
```

Open browser and go to `http://localhost:9999`. Reload the page if the video is not playing. VSCode port forwarding doesn't work for this project. Login remotely using teamviewer or anydesk, then open a browser and go to `http://localhost:9999`. You might need to reload the page if the video is not playing. 