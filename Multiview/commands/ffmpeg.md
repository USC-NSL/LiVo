## Calculate bitrate

- The videos dumped in standalone mode doesn't have duration and birate parameter. So we need to calculate packet size and duration using `ffprobe` to calculate bitrate.

```bash
ffprobe -v quiet -select_streams v -show_entries packet=size -of compact=p=0:nk=1 color_nvenc_10000k_c.mp4 | awk '{s+=$1} END {print s}'            # Number of total bytes
ffprobe -v quiet -select_streams v -show_entries packet=duration_time -of compact=p=0:nk=1 color_nvenc_10000k_c.mp4 | awk '{s+=$1} END {print s}'   # Duration in seconds
# OR
ffprobe -show_entries stream=r_frame_rate,nb_read_frames -select_streams v -count_frames -of compact=p=0:nk=1 -v 0 color_nvenc_10000k_c.mp4         # prints fps | number of frames

# Bitrate = (Number of total bytes * 8) / Duration in seconds

ffprobe -v quiet -select_streams v -show_entries stream=duration -of compact=p=0:nk=1 color_nvenc_10000k_c.mp4                                      # Doesn't work
``` 