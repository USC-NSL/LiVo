# LiVo
LiVo: Toward Bandwidth-adaptive Fully-Immersive Volumetric Video Conferencing, CoNEXT 2025

## Clone the repository
Clone this repository with submodules.

```bash
git clone --recurse-submodules https://github.com/USC-NSL/LiVo.git
```

## Dependencies

Following are some dependencies required for this project:

- Cmake (>=3.14.0): Build from source
  - `wget https://cmake.org/files/v3.20/cmake-3.20.1.tar.gz`
  - `tar -xvf cmake-3.20.1.tar.gz`
  - `cd cmake-3.20.1`
  - `./configure`
  - Add `cmake/bin` to `$PATH`
  - Check the version using `cmake --version`.
- Ninja: `ninja-build`
- OpenMP: `libomp-dev`
- OpenCV: `libopencv-dev`
- OpenCV: Build from source
  ```bash
  $ git clone --recurse-submodules https://github.com/opencv/opencv.git
  $ mv opencv/ opencv-3.4.20
  $ cd opencv-3.4.20
  $ git checkout tags/3.4.20 -b origin/3.4
  $ cd ..
  $ mkdir opencv-3.4.20_install
  $ git clone --recurse-submodules https://github.com/opencv/opencv_contrib.git
  $ mv opencv_contrib/ opencv_contrib-3.4.20/
  $ cd opencv_contrib-3.4.20
  $ git checkout tags/3.4.20 -b origin/3.4
  $ cd ../opencv-3.4.20
  $ mkdir build && cd build
  $ cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/home/lei/opencv/opencv-3.4.20_install \
    -D INSTALL_C_EXAMPLES=ON \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/home/lei/opencv/opencv_contrib-3.4.20/modules \
    -D BUILD_EXAMPLES=ON \
    -D WITH_FFMPEG=OFF ..
  $ ccmake ..
  ```
  - Turn these flags:
    - `BUILD_CUDA_STUBS` to `ON`
    - `BUILD_JPEG` to `ON`
    - `BUILD_PNG` to `ON`
    - `ENABLE_CXX11` to `ON`
    - `INSTALL_BIN_EXAMPLES` to `ON`
    - `WITH_CUDA` to `ON`
    - `WITH_FFMPEG` to `OFF` (ffmpeg didn't work for me)
    - `WITH_OPENGL` to `ON`
    - `WITH_OPENMP` to `ON`
    - `WITH_JPEG` to `ON`
    - `WITH_PNG` to `ON`
    - `WITH_VA` to `ON`
    - `WITH_VA_INTEL` to `ON`
  - `make -j8`
  - `make install`
  - Add to OpenCV install directory path to `CMAKE_PREFIX_PATH`.
- Python: `python3.8-dev`, `python3-pip`

- Rust and cargo: `curl https://sh.rustup.rs -sSf | sh`
  - Add `$HOME/.cargo/bin` to `$PATH`
  - Use other rustc version `1.69.0` or `1.70.0`:
    - After the above installation run: `rustup install 1.69.0`, `rustup override set 1.69.0`
    - Install specific version on `cargo-c`: `cargo install --version 0.9.19+cargo-0.70 cargo-c`
  - Current `cargo-c` has an issue:
    - Check this [Issue](https://github.com/lu-zero/cargo-c/issues/332)
    - `git clone https://github.com/lu-zero/cargo-c.git`
    - `cd cargo-c`
    - `cargo install --path .`

- Latest GStreamer and other dependencies
  - Install meson: `sudo python3.8 -m pip install meson`
  - Install tomli: `sudo python3.8 -m pip install tomli`
  - For python build: 
    - Install gobject-introspection: `sudo apt install python3-gi gobject-introspection gir1.2-gtk-3.0`
    - Install glib >= 2.74: [Instructions](https://gitlab.gnome.org/GNOME/glib/-/blob/main/INSTALL.md)
  - Add `$HOME/.local/bin` to `$PATH` for `meson`
  - Dependencies: `flex`, `bison`, `libssl-dev`
  - `git clone https://gitlab.freedesktop.org/gstreamer/gstreamer.git`
  - `cd gstreamer && meson subprojects/`
  - Check options using `meson configure`. Go inside subprojects and run `meson configure` for finer options. Use option `-Drs:examples=disabled` to disable compiling examples for rust plugins.
  - `meson -Dgpl=enabled -Dgood=enabled -Dbad=enabled -Drs=enabled build` - if rs failed, just remove it and start again
  - `ninja -C build`
  - `meson install -C build`
  - `python3.8 -m pip install cython==0.29.34`
  - `python3.8 -m pip install cython==1.24.3`
  - `python3.8 -m pip install websockets pycairo` (might have to install libcairo2-dev)
  - `sudo python3.8 -m pip install PyGObject` - make sure to install gi under system default dirs
  - Build `subprojects/gst-plugins-rs`
    - `cargo cbuild`
    - Add `subprojects/gst-plugins-rs/target/x86_64-unknown-linux-gnu/debug` to `GST_PLUGIN_PATH`
  - Build `subprojects/gst-python`
    - `meson build -Dpython=python3.8 -Dpygi-overrides-dir=/usr/local/lib/python3.8/dist-packages/gi/overrides`
    - If `pygobject-3.0` not found, then install `sudo apt install python-gi-dev`
    - `ninja -C build`
    - `meson install -C build`
    - Add `/usr/local/lib/python3.8/dist-packages` to `PYTHONPATH` and `/usr/local/lib/x86_64-linux-gnu/girepository-1.0/` to `GI_TYPELIB_PATH`
  - Might need to build `subprojects/gstreamer-vaapi` with meson:
    - Dependencies: `libudev-dev`, `build-essential`, `libgtk-3-dev`
    - Build `gst-plugins-base`: `cd gst-plugins-base/ && meson build && ninja -C build && meson install -C build`
    - Build `gstreamer-vaapi`: `cd subprojects/gstreamer-vaapi/ && meson build && ninja -C build && meson install -C build`
    - Check: `gst-inspect-1.0 vaapi`, `gst-inspect-1.0 vaapih264enc`
    - If nothing works, check these: (LINK1)[https://www.intel.com/content/www/us/en/developer/articles/technical/gstreamer-vaapi-media-sdk-installation-environment.html], (LINK2)[`https://gitlab.freedesktop.org/gstreamer/gstreamer-vaapi/-/issues/194`] and check for dependencies in the output of `meson build` command.
  - In each step, if the configuration fails, usually it's because a lib is missing (and should be installed through `apt`)

- VTK7: `libvtk7-dev, libvtk7-qt-dev, libvtk7.1, libvtk7.1-qt`

- Eigen3 (>=3.3.9): Build from source.
  - `wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz`
  - `tar -xvf eigen-3.3.9.tar.gz`
  - `cd eigen-3.3.9`
  - `mkdir build && cd build`
  - `cmake -DCMAKE_BUILD_TYPE=Release ..`
  - `make`
  - `sudo make install`

- flann 1.9.1: `libflann-dev`
- turbojpeg : `libturbojpeg`, `libturbojpeg0-dev`

- PCL 1.12.0: [Build from Source](https://pcl.readthedocs.io/projects/tutorials/en/latest/compiling_pcl_posix.html).
  - Use CMake to build PCL in Release mode using `cmake -DCMAKE_BUILD_TYPE=Release ..`
  - Switch `ON` or `OFF` for flags using `ccmake -DCMAKE_BUILD_TYPE=Release ..`
  - Some required flags as `ON`: `BUILD_CUDA, BUILD_GPU, PCL_COMMON, PCL_OCTREE, PCL_FILTERS, PCL_GEOMETRY, PCL_IO, PCL_SEGMENTATION, PCL_VISUALIZATION, PCL_REGISTRATION, PCL_APPS, PCL_CLOUD_COMPOSER`

- [Azure Kinect SDK](https://github.com/Rajrup/Azure-Kinect-Sensor-SDK/tree/origin/release/1.4.x) (Custom Build):
  - `git clone --recurse-submodules https://github.com/Rajrup/Azure-Kinect-Sensor-SDK.git`
  - `git checkout origin/release/1.4.x`
  - `git pull origin origin/release/1.4.x`
  - Check if all the [dependencies](https://github.com/Rajrup/Azure-Kinect-Sensor-SDK/blob/4526a0234879186ea77a695af94a6f0d5a7ec964/scripts/docker/setup-ubuntu.sh#L37) are installed.
  - [Build Instructions](https://github.com/Rajrup/Azure-Kinect-Sensor-SDK/blob/origin/release/1.4.x/docs/building.md)
  - Remember to build Azure Kinect SDK in `Release` mode.
  - Copy `libdepthengine.so` (**Essential Step**):
    - `cp Azure-Kinect-Sensor-SDK/libk4a1.4/libdepthengine.so.2.0 Azure-Kinect-Sensor-SDK/build/bin/`
    - `chmod 777 Azure-Kinect-Sensor-SDK/build/bin/libdepthengine.so.2.0`
  - FFmpeg: 
    - `libasound2-dev`, `libsdl2-dev`
    - `ffmpeg`, `libavfilter-dev`, `libavcodec-dev`, `libavformat-dev`, `libavutil-dev`, `libswscale-dev`, `libavdevice-dev`
  - Change the path of the `Azure Kinect SDK` pointing to your local path in `Multiview/CMakeLists.txt` file.
  - Change the path to the data folder in the macro defined in `Multiview/src/Multiview.cpp` file.

- Open3D [Link](http://www.open3d.org/docs/release/compilation.html):
  - cmake 3.20 or higher
  - Cuda 10.2+ 
  - `git clone --recurse-submodules https://github.com/intel-isl/Open3D.git`
  - `cd Open3D`
  - `git checkout tags/v0.17.0-1fix6008 -b origin/master`
  - `sudo apt-get install libc++-7-dev libc++abi-7-dev xorg-dev libglu1-mesa-dev`
  - `./util/install_deps_ubuntu.sh`
  - `conda create -n KinectStream python=3.8` (Optional for python build)
  - `conda activate KinectStream`  (Optional for python build)
  - `mkdir build && cd build`
  - `cmake -DCMAKE_BUILD_TYPE=Release -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_SHARED_LIBS=ON -DBUILD_PYTHON_MODULE=ON -DBUILD_EXAMPLES=OFF -DBUILD_GUI=ON -DBUILD_CUDA_MODULE=ON -DCMAKE_INSTALL_PREFIX=${HOME}/open3d_install ..`
  - `make install -j12`
  - `make install-pip-package` (Optional for python build)
  - Check python build: `python -c "import open3d"`

- Ccache for faster recompilations (Optional) [link](http://www.open3d.org/docs/release/compilation.html#ubuntu-18-04-20-04)
  - `git clone https://github.com/ccache/ccache.git`
  - `cd ccache`
  - `git checkout v4.6 -b 4.6`
  - `mkdir build && cd build`
  - `cmake -DZSTD_FROM_INTERNET=ON -DHIREDIS_FROM_INTERNET=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${HOME} ..`
  - `make -j$(nproc)`
  - `make install -j$(nproc)`
  - `echo "PATH=${HOME}/bin:${PATH}" >> ~/.bashrc`
  - `source ~/.bashrc`
  - `which ccache`
  - `ccache --version`
- Boost:
  -  `cd Multiview/lib/boost && git checkout 32da69a && ./bootstrap.sh && sudo ./b2 install`
- flatbuffers
  - Has been added to CMakeLists.txt file. Use `python3.8 -m pip install flatbuffers` to install the python version.
  - Compile schema using:
    - C++: `flatc -o Multiview/include/ --cpp Multiview/src/test/frameInfo.fbs`
    - Python: `cd Multiview/src/test/; flatc --python frameInfo.fbs`
  - Kinect frame is shared over `mmap`, while `socket` is used for synchronization.
- For Ptcl based streaming:
  - zstd (v1.5.2): Build from source. Follow the steps [here](https://github.com/facebook/zstd/tree/dev/build/cmake).
    - `git clone https://github.com/facebook/zstd.git`
    - `cd zstd`
    - `git checkout tags/v1.5.2 -b origin/dev`
    - `cd build/cmake/`
    - `mkdir build && cd build`
    - `cmake -DCMAKE_BUILD_TYPE=Release ..`
    - `make`
    - `sudo make install`
  - draco:
    - `git clone --recurse-submodules https://github.com/google/draco.git`
    - `cd draco`
    - `git checkout tags/1.5.6 -b origin/master`
    - `mkdir build && cd build`
    - `cmake -DCMAKE_BUILD_TYPE=Release ..`
    - `make`
    - `sudo make install`
    - Check other build options using `ccmake ..`
  - gpcc:
    - `git clone --recurse-submodules https://github.com/MPEGGroup/mpeg-pcc-tmc13.git`
    - `cd mpeg-pcc-tmc13/`
    - `git checkout tags/release-v23.0-rc2 -b origin/master`
    - `mkdir build && cd build`
    - `cmake -DCMAKE_BUILD_TYPE=Release ..`
    - `make`
- Time Synchronization:
  - Chrony:
    - `sudo apt install chrony`
    - Change IP address of server and client IP in `config/chrony_server.conf` and `config/chrony_client.conf` files.
    - `cp config/chrony_server.conf /etc/chrony/chrony.conf`  (for server)
    - `cp config/chrony_client.conf /etc/chrony/chrony.conf`  (for client)
    - `sudo systemctl enable chrony`
    - `sudo systemctl restart chrony`
    - `sudo systemctl status chrony`
    - `chronyc sources` (Server: for checking time sources)
    - `chronyc tracking` (Client: for tracking time synchronization)
    - `chronyc sources -v` (Client: for checking time sources)
    - `chronyc sourcestats -v` (Client: for checking time sources in detail)

## Build the Project

Use CMake to build the project with Release mode.

- `mkdir build && cd build`
- `cmake -DCMAKE_BUILD_TYPE=Release ..`
- `make`

## Common Problems:

- SQLite Error: `/usr/lib/libgdal.so.20: undefined reference to 'sqlite3_column_origin_name'`
  - (Refer)[https://stackoverflow.com/questions/48169718/libgdal-so-1-undefined-symbol-sqlite3-column-table-name]
  - `wget https://www.sqlite.org/2023/sqlite-autoconf-3410200.tar.gz`
  - `tar -xvf sqlite-autoconf-3410200.tar.gz`
  - `mv sqlite-autoconf-3410200 sqlite`
  - `cd sqlite`
  - `CFLAGS="-DSQLITE_ENABLE_COLUMN_METADATA=1" ./configure`
  - `make && sudo make install`
- Network Configuration:
  - The server and client should have a 1Gbps NIC. Check with command `sudo lshw -C network` and check `size` and `capacity` of ethernet interface to be `1Gbps`.
  - Check the network speed using `iperf3` tool. Install using `sudo apt install iperf3`. Run `iperf3 -s` on the server and `iperf3 -c [server_ip]` on the client.
  - If the speed is less than 1Gbps and NIC supports >=1Gbps, run the following commands:
    ```bash
    sudo ethtool -s [interface] autoneg off
    sudo ethtool -s [interface] speed 1000 duplex full
    sudo systemctl restart network-manager.service
    ```
  - Add this command to `/etc/rc.local` to run on boot. This file is run by Linux OS after the boot process is complete.
- Packet loss:
  - We have turned on packet retransmissions in Gstreamer WebRTC. This should cover packet losses.
  - There can be significant packet losses in Linux systems at higher endcoder bitrates such as 300 Mbps. Linux systems have fixed UDP buffer size of 213 KB both for sender and receiver. This means if either sender or receiver buffer size overflows due to overwhelming amount of packets, the packets are dropped. This happens at higher bitrates. 
  - Check UDP send buffer size by `sysctl net.core.wmem_max` and `sysctl net.core.wmem_default`.
  - Check UDP receive buffer size by `sysctl net.core.rmem_max` and `sysctl net.core.rmem_default`.
  - Increase buffers to 2 MB by adding the following lines to `/etc/sysctl.conf`:
    ```bash
    net.core.wmem_max=2097152
    net.core.wmem_default=2097152
    net.core.rmem_max=2097152
    net.core.rmem_default=2097152
    ```

## Run the project (on two machines):

### Client-side rate limiting:

- On server machine, allow binding to non-local addresses:
  - `sudo ufw allow 8080/tcp`
  - `sudo ufw allow 5252/tcp`
- On client mahcine, setup packet forwarding rules:
  - Run the following commands every time you restart the machine:
    - `sudo sysctl -w net.ipv4.ip_forward=1` -> enable mahimahi
    - `sudo iptables --table filter --policy FORWARD ACCEPT`
    - `sudo iptables -t nat -A PREROUTING -s [SERVER_IP] -j DNAT --to-destination 100.64.0.2`
    - `sudo iptables -A FORWARD -s [SERVER_IP] -d 100.64.0.2 -j ACCEPT`
    - The above 4 commands are available in `cd Scripts && bash setup_mahimahi_client.sh` script.
  - Start mahimahi:
    - `cd Scripts`
    - `bash run_mahimahi.sh`
    - `exit` - to exit mahimahi shell

- Run our pipeline:
  - On client machine, start Multiview Client and WebRTC Receiver:
    - `cd KinectStream/Scripts && bash run_client_moonbaby1.sh` -> Inside or outside mhaimahi shell for rate limiting or without rate limiting.
  - On server machine, start Multiview Server and WebRTC Sender (Start server once client waits for connection to server):
    - `cd KinectStream/Scripts && bash run_server_moonbaby1.sh`
    - Check parameters of the server and client in the scripts. For example, start frame number, bitrates, abr mode, etc.
  
### Server-side rate limiting:

- On server machine, allow binding to non-local addresses:
  - `sudo ufw allow 8080/tcp`
  - `sudo ufw allow 5252/tcp`
- On server machine, setup packet forwarding rules:
  - Run the following commands every time you restart the machine:
    - `sudo sysctl -w net.ipv4.ip_forward=1` -> enable mahimahi
    - `sudo iptables --table filter --policy FORWARD ACCEPT`
    - `sudo iptables -t nat -A PREROUTING -s [CLIENT_IP] -j DNAT --to-destination 100.64.0.2`
    - `sudo iptables -A FORWARD -s [CLIENT_IP] -d 100.64.0.2 -j ACCEPT`
    - The above 6 commands are available in `cd Scripts && bash setup_mahimahi_server.sh` script.
  - Start mahimahi:
    - `cd Scripts`
    - `bash run_mahimahi_server.sh`
    - `exit` - to exit mahimahi shell
  - Running outside mahimahi, requires us to delete packet forwarding rules, since the client needs to directly connect to `SERVER_IP` rather than forward to `100.64.0.2`.
    - `sudo iptables -D FORWARD -s [CLIENT_IP] -d 100.64.0.2 -j ACCEPT`
    - `sudo iptables -t nat -D PREROUTING -s [CLIENT_IP] -j DNAT --to-destination 100.64.0.2`
    - The above 2 commands are available in `cd Scripts && bash remove_setup_mahimahi_server.sh` script.

### Previous Instructions (for 2 machines):

- Setup networks:
  - Client-side rate limiting:
    - On server machine, allow binding to non-local addresses:
      - `sudo ufw allow 8080/tcp`
      - `sudo ufw allow 5252/tcp`
    - On client mahcine, setup packet forwarding rules:
      - `sudo sysctl -w net.ipv4.ip_forward=1` - enable mahimahi
      - `sudo iptables --table filter --policy FORWARD ACCEPT`    
      - `sudo iptables -t nat -A PREROUTING -s [SERVER_IP] -j DNAT --to-destination 100.64.0.2`
      - `sudo iptables -A FORWARD -s [SERVER_IP] -d 100.64.0.2 -j ACCEPT`
  - On client machine, run mahimahi shell:
    - mm-link [up-trace] [down-trace]
    - Note that this should be the only mahimahi shell on the machine, because we're hard coding the packet forwarding rules.
  - On server machine, start Multiview Server and WebRTC Sender:
    - `cd KinectStream/Webrtc && bash run_server.sh`
  - On client machine, start Multiview Client and WebRTC Receiver:
    - `cd KinectStream/Webrtc && bash run_client.sh`

## Run the project (on single machine, needs to update static addresses):
  - WebRTC agents rely on coroutine which occupies CPU without limitation (and race the Multiview Server/Client). We use taskset to limit the resource usage. This is the setup on a machine with 12 cores.
  - Start shell1 and shell2 with `mm-link`. Note that the starting order matters.
  - You might need to run `sudo iptables --table filter --policy FORWARD ACCEPT` to enable packet forwarding to mahi shell.
  - Start Multiview Server: 
    - `cd Multiview/build && taskset --cpu-list 0-1 ./Multiview/MultiviewServer`. Currently it will preload all frames into memory
  - Start Multiview Client: 
    - `cd Multiview/build && taskset --cpu-list 2-3 ./Multiview/MultiviewClient`
  - Start WebRTC sender: 
    - In shell1, run `cd WebRTC && taskset --cpu-list 4-7 python3.8 sender.py -v 0`
  - Start WebRTC receiver: 
    - In shell2, run`cd WebRTC && taskset --cpu-list 8-11 python3.8 receiver.py -v 0`

If you want to run this in **headless mode (over ssh-session)**, you can use the following command:

- `export DISPLAY=:[your screen number]` ([OpenGL context](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/810) required by the SDK). The number after `:` is the screen number, which can be obtained by the command `who`. If no screen is available, then you should login to the machine using teamviewer or any other remote desktop software. Run `who` again to get the display number against your `username`.

## Notes

- Current active folder is `Multiview`.
- Previous point cloud based approach is under `Calibration`. I will soon refactor this part of the code. Sorry for the misnomer.

## Panoptic

Modify the paths in `src/panoptic.cpp` file. You will require cmake build and run the `PanopticServerClient` executable. This equivalent to the `MultiviewServerClient` executable, which runs without the WebRTC part.

To Do:

- Add WebRTC support for Panoptic.
- Either create `PanopticServer.cpp` and `PanopticClien.cpp` separately or add a dataset flag to `server.cpp` and `client.cpp` files to run Panoptic and Kinect. Later seems to be a better option as it will reduce code duplication.

## Mmap (moved above)

- Use Python 3.8 for using `mmap` support from `multiprocessing` package.
- Library named [`flatbuffers`](https://google.github.io/flatbuffers/) is used for serialization and deserialization of metadata.
- `flatbuffers` has been added to CMakeLists.txt file. Use `pip install flatbuffers` to install the python version.
- Compile schema using:
  - C++: `flatc -o Multiview/include/ --cpp Multiview/src/test/frameInfo.fbs`
  - Python: `cd Multiview/src/test/; flatc --python frameInfo.fbs`
- Test codes are located under `Multiview/src/test` - `mmap_server.cpp` and `mmap_client.py`.
- Kinect frame is shared over `mmap`, while `socket` is used for synchronization.
- Run:
  - Start python client: `cd Multiview/src/test/; python mmap_client.py`

## Depth to Color

- Depth to RGB and RGB to depth is implemented here `Multiview/src/test/colorized_depth.cpp`.
- APIs:
  - Depth to color: `depth_to_color()`
  - RGB to depth: `color_to_depth()`
  - Check comments for function parameters.
- Build `colorized_depth_test` using cmake.
- Run: `build/Multiview/colorized_depth_test` to test.
- First, it will show the color map where left is nearest depth and right is farthest depth. Press a key to proceed to the next step.
- Second, it will show original depth image. Press a key to proceed to the next step.
- Third, it will show colorized depth image. Press a key to proceed to the next step.
- Fourth, it will show recovered depth image. Press a key to proceed to the next step.

## Required files

- A `data` folder containing the RGB+D images and calibration files of the cameras.
- Assuming there are 4 cameras, then the `data` folder structure should be like this:

```
data
├── 1_color_1.png
├── 1_color_2.png
├── 1_color_3.png
├── 1_depth_0.png
├── 1_depth_1.png
├── 1_depth_2.png
├── 1_depth_3.png
.
.
.
├── devices.txt
├── intrinsics_<camID0>.txt or intrinsics_ColorDownscaled_<cam0SerialID>.txt (for using Kinect Cameras using our capturer)
├── intrinsics_<camID1>.txt or intrinsics_ColorDownscaled_<cam1SerialID>.txt
├── intrinsics_<camID2>.txt or intrinsics_ColorDownscaled_<cam2SerialID>.txt
├── intrinsics_<camID3>.txt or intrinsics_ColorDownscaled_<cam3SerialID>.txt
├── extrinsics_<camID0>.txt
├── extrinsics_<camID1>.txt
├── extrinsics_<camID2>.txt
└── extrinsics_<camID3>.txt
```

- `devices.txt` contains the mapping of the camera IDs to the device IDs. For example, if the camera IDs are 0, 1, 2, 3 and the device IDs are 1, 2, 3, 4, then the `devices.txt` file should contain the following:

```
0 1
1 2
2 3
3 4
```

- `intrinsics_color_<camID>.txt` contains the camera intrinsics for the color camera of the camera with ID `<camID>`. For example, the file should contain the following:

```
0 w
1 h
2 cx
3 cy
4 fx
5 fy
```

- `extrinsics_<camID>.txt` contains the camera extrinsics for the camera with ID `<camID>`. For example, the file should contain the following:

```
-0.0455333 -0.07516 -0.880633   # Translation vector T
0.638706 0.0690572 -0.766346    # Rotation matrix R row 1
0.0747437 -0.996823 -0.0275313  # Rotation matrix R row 2
-0.765812 -0.0396951 -0.641838  # Rotation matrix R row 3
0                               # Marker ID
1                               # 1 is calibration is successful, 0 otherwise
```

- Last 2 lines are important if `Azure Kinect` cameras are used. For other cameras or datasets, you can use 0 for marker ID and 1 for calibration status.


## TODO

- Build Docker for the entire project
