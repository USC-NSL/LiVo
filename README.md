# LiVo

LiVo is the artifact for **“LiVo: Toward Bandwidth-adaptive Fully-Immersive
Volumetric Video Conferencing,” CoNEXT 2025**.

- Paper: <https://dl.acm.org/doi/10.1145/3768981>
- The active programs are `MultiviewServerPoolNew` and
  `MultiviewClientPoolNew`.
- The canonical paper-reproduction launchers are under
  `Scripts/e2e_quality/`.

## Description

- LiVo uses two machines: one runs the server and the other runs the client.
- Mahimahi replays bandwidth traces to emulate a time-varying network link for controlled end-to-end experiments. The current pipeline obtains its available bandwidth from the selected `Mahimahi` trace.
- The server reads synchronized RGB-D frames from disk, culls content according to the receiver's viewing frustum, and tiles each frame.
- Two independent `H.265` encoders encode the tiled output: one for color and one for depth.
- `GStreamer WebRTC` transports both encoded streams to the client. Separate control channels carry calibration, frustum, mask, and bitrate-allocation information.
- The client decodes both streams, combines the calibrated color and depth data into a 3D point cloud, and displays it using `Open3D`.
- Prerecorded user viewing-frustum traces under `data/user_log_updated/` drive viewport-dependent culling and predictive-culling experiments.
- The current data loader supports RGB-D sequences from the `CMU Panoptic` dataset.
- Frames are loaded into `Azure Kinect SDK` image and calibration constructs. Live `Azure Kinect` support can therefore be added by replacing the disk-backed frame-fetching stage without redesigning the downstream processing, encoding, transport, or rendering pipeline.

## Supported platform

The installation scripts target Ubuntu 18.04, 20.04, and 22.04 on x86-64.
Ubuntu 20.04 is the closest match to the original testbed.

The user must install a compatible NVIDIA driver and CUDA 11.3 before building
LiVo. The scripts do not install or modify CUDA.

Important compatibility notes:

- CUDA 11.3 officially supports Ubuntu 18.04 and 20.04.
- On Ubuntu 22.04, CUDA 11.3 is outside NVIDIA's supported host matrix. Use GCC/G++ 10 for CUDA compilation and validate the resulting binaries.
- The GPU must support `H.265` encoding and decoding, including the format used by LiVo's `Y444_16LE` depth stream.
- `Azure Kinect SDK` 1.4 is archived. LiVo uses the pinned research fork and its matching depth engine.

## Installation model

Installation is split into five phases:

1. Install Ubuntu system packages and build tools.
2. Build, locally install, and validate LiVo's library dependencies.
3. Build only LiVo source code.
4. Configure data paths, machine addresses, traces, and outputs.
5. Configure Mahimahi and run the two-machine pipeline.

The third phase never compiles third-party libraries. CMake first looks under
`Multiview/lib/<library>/install` and falls back to an already-installed system
package only when the local installation is absent.

## Dependency versions

The source revisions are pinned by gitlinks in `.gitmodules` and duplicated in
`install/versions.lock` for validation:

- CMake 3.31.12
- CUDA 11.3, supplied by the user
- OpenCV 3.4.20 with opencv_contrib 3.4.20
- Open3D 0.18.0
- PCL 1.12.1
- GStreamer 1.24.13
- usrsctp 0.9.5.0 for GStreamer WebRTC data-channel support
- Azure Kinect SDK custom release-1.4.x fork
- Eigen 3.4.1
- Boost 1.80.0
- FlatBuffers 25.12.19
- zstd 1.5.7
- Draco 1.5.7
- Abseil LTS 20250814.2
- gflags 2.3.0
- nlohmann_json 3.12.0

## 1. Install system dependencies

Clone LiVo recursively:

```bash
git clone --recursive https://github.com/USC-NSL/LiVo.git
cd LiVo
```

Install the Ubuntu packages:

```bash
./install/install-deps.sh
```

The script:

- detects Ubuntu 18.04, 20.04, or 22.04;
- installs compiler, multimedia, GUI, networking, and SDK prerequisites,
  including Chrony and Mahimahi;
- reports the detected CMake;
- asks whether CMake 3.31.12 should be installed under `$HOME/.local`;
- creates a private Python environment for Meson and Ninja under
  `install/tools`.

If CMake is installed by this script, ensure the following appears before the system's `PATH`:

```bash
export PATH="$HOME/.local/bin:$PATH"
```

`GStreamer` 1.24 requires `Python` 3.8 or newer. Ubuntu 18.04 users must make a `Python` 3.8+ interpreter available before running `install-deps.sh`.

## 2. Build and validate LiVo dependencies

Verify that all submodules match the locked revisions:

```bash
./install/fetch.sh
```

Build all dependencies:

```bash
./install/build-all.sh
```

The source, build, and install locations for a dependency named `foo` are:

```text
Multiview/lib/foo/                 # submodule source
Multiview/lib/foo/build-livo/      # out-of-source build
Multiview/lib/foo/install/         # installation directory
```

Build or rebuild one dependency:

```bash
./install/build-one.sh flatbuffers
./install/build-one.sh gstreamer --rebuild
```

Control parallelism if needed:

```bash
LIVO_BUILD_JOBS=8 ./install/build-all.sh
```

Validate existing installations without rebuilding:

```bash
./install/verify.sh --strict-local
```

Build logs are written under `install/logs/`. Successful strict validation
writes `install/dependencies.ready`.

### GStreamer isolation

LiVo's needs GStreamer, but is installed locally under `Multiview/lib/gstreamer`.

Check the private GStreamer installation:
```bash
./install/livo-env.sh -- gst-inspect-1.0 --version
./install/verify-gstreamer.sh
```

When the private installation exists, `install/livo-env.sh`:

- clears the system GStreamer plugin path;
- selects the private plugin scanner;
- uses a private registry cache;
- exposes only the private GStreamer pkg-config and library paths.

Do not copy the private GStreamer variables into `.bashrc`; mixing system and private plugins can cause symbol errors or registry crashes in Ubuntu.

## 3. Build LiVo only

LiVo is built in `Release` mode for performance. After dependency installation succeeds:

```bash
./install/build-livo.sh
```

Equivalent manual commands are:

```bash
source install/livo-env.sh
cmake -S . -B build -GNinja \
  -DCMAKE_BUILD_TYPE=Release \
  -DLIVO_DEPS_ROOT="$PWD/Multiview/lib"
cmake --build build --parallel "$(nproc)"
```

If a package is missing, configuration stops and asks the user to run `build-one.sh` or `build-all.sh` to build it.

The principal binaries are:

```text
build/Multiview/MultiviewServerPoolNew
build/Multiview/MultiviewClientPoolNew
build/Multiview/PanopticGT
build/Multiview/PanopticPtclDracoOutputGeneration
```

## 4. Configure the machines and data

Create a site configuration on both machines:

```bash
cp Scripts/livo-site.example.sh Scripts/livo-site.sh
```

Edit `Scripts/livo-site.sh`:

```bash
export LIVO_SERVER_HOST="<server_IP_address>"
export LIVO_CLIENT_HOST="<client_IP_address>"
export LIVO_MAHIMAHI_HOST="<mahimahi_IP_address>" # This is the IP address of the machine after running Mahimahi shell.
export LIVO_DATA_ROOT="/path/to/panoptic_data"
export LIVO_USER_TRACE_ROOT="/path/to/LiVo/data"
export LIVO_OUTPUT_ROOT="/path/to/livo-output"
export LIVO_TRACE_ROOT="/path/to/mahimahi-traces"
```

Next, edit the sequence JSON used by the run script, for example
`Multiview/config/panoptic_160906_band2.json`. Set:

- `panoptic_path`: parent directory containing the Panoptic sequences and
  `FrameIndices.json`;
- `user_trace_path` and `user_trace_folder`;
- `log_id`; This is the log ID of the user viewing-frustum trace. For example, there are 4 user traces under `data/user_log_updated/160906_band2`. So the log_id is 0,1, 2, and 3.
- `qrcode_folder`; This is the folder containing the QR codes to frame stamp the RGB-D frames.
- `seq_name`, which is the trace/index name without `_with_ground`.

For band2, the intentionally different names are:

```text
Command-line sequence: 160906_band2_with_ground
JSON sequence key:     160906_band2
```

The RGB-D sequence layout is:

```text
<panoptic_path>/160906_band2_with_ground/
├── devices.txt
├── extrinsics_0.txt ... extrinsics_9.txt
├── intrinsics_0.txt ... intrinsics_9.txt
├── color/<frame>_color_<camera>.png
└── depth/<frame>_depth_<camera>.png
```

Color images are BGRA and depth images are single-channel16-bit millimeter values. The
client-side adaptive split requires the matching ground-truth view directory.

Runtime flags override site defaults where needed:

```text
--server_host=<server LAN/public address>
--mahimahi_host=100.64.0.2
--config_file=<sequence JSON>
--seq_name=<disk sequence>
--output_dir=<writable output root>
```

## 5. Run LiVo with server-side Mahimahi

### Network preparation

LiVo uses these TCP/WebSocket endpoints:

- 8080: calibration and receiver frustum
- 8082: client bitrate split
- 8083: optional point-cloud channel
- 5252: color WebRTC signaling
- 5253: depth WebRTC signaling

WebRTC media uses ICE-negotiated UDP ports in addition to these fixed TCP signaling ports. Permit the negotiated UDP traffic between the two hosts.

### Synchronize the two machines with Chrony

LiVo compares server send timestamps with client receive, render, and display timestamps when calculating end-to-end latency. The two machines must therefore use synchronized clocks. `install/install-deps.sh` installs `Chrony`.

Configure the server first:

1. Open `Multiview/config/chrony_server.conf`.
2. Change its final `allow` directive to the LAN subnet containing both machines. For example:

   ```text
   allow <server_IP_address>/24
   ```

3. Install the configuration and start Chrony:

   ```bash
   sudo cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.livo-backup
   sudo cp Multiview/config/chrony_server.conf /etc/chrony/chrony.conf
   sudo systemctl enable --now chrony
   sudo systemctl restart chrony
   sudo chronyc makestep
   ```

Then configure the client:

1. Open `Multiview/config/chrony_client.conf`.
2. Change its final `pool` directive to the server's reachable physical IP address, not the Mahimahi address. For example:

   ```text
   pool <server_IP_address> iburst prefer
   ```

3. Install the configuration and start Chrony:

   ```bash
   sudo cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.livo-backup
   sudo cp Multiview/config/chrony_client.conf /etc/chrony/chrony.conf
   sudo systemctl enable --now chrony
   sudo systemctl restart chrony
   sudo chronyc makestep
   ```

Allow inbound UDP port 123 from the client if the server firewall is enabled.
For UFW, run this on the server:

```bash
sudo ufw allow from 68.181.32.215 to any port 123 proto udp
```

Verify synchronization on both machines before starting LiVo:

```bash
chronyc tracking
chronyc sources -v
```

On the client, the server should appear with `^*` in `chronyc sources -v`.
The `System time` offset reported by `chronyc tracking` should be small enough for the desired latency measurement accuracy. Chrony remains running in the background; these steps do not need to be repeated for every LiVo run.

Check the link before running:

```bash
# server
iperf3 -s

# client
iperf3 -c "$LIVO_SERVER_HOST"
```

On the server:

```bash
./Scripts/setup_mahimahi_server.sh # Need to be run once for setting the server to client packet forwarding.
cd Scripts
./run_mahimahi_server.sh
```

The second command enters the Mahimahi shell and writes `mm_time` in its
working directory.

On the client, start the waiting receiver:

```bash
cd Scripts
./run_client_band2.sh
```

Inside the server's Mahimahi shell:

```bash
./run_server_band2.sh
```

### Stop an experiment

To stop an experiment before it finishes, use separate terminals so that the
terminals running the client and server launchers can remain attached to their
processes.

On the client:

```bash
cd /path/to/LiVo
./Scripts/stop_client.sh
```

On the server:

```bash
cd /path/to/LiVo
./Scripts/stop_server.sh
```

After the server has stopped, return to its Mahimahi terminal and leave the Mahimahi shell:

```bash
exit
```

If you want to remove the server's Mahimahi packet-forwarding rules from a normal shell:

```bash
cd /path/to/LiVo
./Scripts/setup_remove_mahimahi_server.sh
```

The top-level band2 scripts are example launchers. For more experiments of the paper, the scripts are
under:

```text
Scripts/e2e_quality/livo/
Scripts/e2e_quality/livo_nocull/
Scripts/e2e_quality/starline/
Scripts/e2e_quality/draco/
```

For example:

```bash
# server: enter the matching method directory before starting mm-link
cd Scripts/e2e_quality/livo
./run_mahimahi_server.sh

# client
cd Scripts/e2e_quality/livo
./run_client_band2.sh

# server, inside mm-link
./run_server_band2.sh
```

## PointSSIM

MATLAB installation and licensing are left to the user.

1. Generate or locate the ground-truth PLY files using `PanopticGT` and the
   scripts under `Scripts/ptcl_gt_generation/`.
2. Locate the distorted PLY files written by the matching E2E client script.
3. Open the sequence script under
   `Metrics/pointssim/scripts/e2e_quality/`.
4. Set `CONFIG_FILE`, `base_path`, `pipeline_method`, `culling`, `mm_type`, and
   `logID` to match the run.
5. Add PointSSIM to MATLAB's path and execute the script:

```matlab
addpath(genpath('/path/to/LiVo/Metrics/pointssim'));
run('/path/to/LiVo/Metrics/pointssim/scripts/e2e_quality/calc_pssim_band2_tracep1.m');
```

The result is a `3D_pssim.csv` file. No additional MATLAB voxelization is
required for LiVo output.

## Paper design and artifact behavior

Several artifact choices are intentional:

- Application bandwidth currently follows the Mahimahi trace. Integrating GCC estimates is future work.
- Distortion-based bitrate adaptation runs on the client because the original server lacked sufficient free CPU cores. It therefore reads ground-truth `RGB-D` views on the client.
- Predictive culling uses a ten-frame lookahead. Experiments found this to be a good estimate of end-to-end delay.
- Additional receiver voxelization is unnecessary because the point cloud used by this pipeline is already voxelized at the required resolution.
- The pipeline uses `Azure Kinect SDK` constructs and data formats. Live capture requires an `RGB-D` source adapter that replaces disk reads with synchronized `Kinect` `RGB-D` fetches; the downstream pipeline does not need redesign.

## Troubleshooting

### Dependency accidentally resolves from the system

Run:

```bash
source install/livo-env.sh
./install/verify.sh --strict-local
ldd build/Multiview/MultiviewServerPoolNew
```

The dependency report printed during CMake configuration shows every selected
origin. Delete `build/CMakeCache.txt` after changing dependency prefixes.

### GStreamer plugin or ABI errors

```bash
rm -f install/cache/gstreamer/registry-*.bin
./install/verify-gstreamer.sh
```

Do not expose `/usr/local` or Ubuntu GStreamer plugins while using the private
runtime.

### High-rate packet loss

The default Linux UDP buffers may be too small for LiVo. Inspect them with:

```bash
sysctl net.core.wmem_max net.core.wmem_default
sysctl net.core.rmem_max net.core.rmem_default
```

The original artifact used 2 MiB send and receive defaults:

```text
net.core.wmem_max=2097152
net.core.wmem_default=2097152
net.core.rmem_max=2097152
net.core.rmem_default=2097152
```

Apply system-wide network changes only after reviewing them with the machine
administrator.

### Headless visualization

Open3D/PCL visualization requires an available OpenGL display. Set `DISPLAY`
to an active session or use a desktop session. Disabling visualization with
the existing viewer flag is preferable for unattended metric generation.

## Removing private dependencies

No dependency installer writes to `/usr/local`. To remove a private dependency,
delete only its generated directories:

```bash
rm -rf Multiview/lib/<library>/build-livo
rm -rf Multiview/lib/<library>/install
```

System packages installed through apt are managed normally by Ubuntu.

## Citation

If you use LiVo, please cite:

```bibtex
@article{ghosh2025livo,
  title={LiVo: Toward bandwidth-adaptive fully-immersive volumetric video conferencing},
  author={Ghosh, Rajrup and Shin, Christina Suyong and Zhang, Lei and Ye, Muyang and Jin, Tao and Madhyastha, Harsha V and Netravali, Ravi and Ortega, Antonio and Rao, Sanjay and Rowe, Anthony and others},
  journal={Proceedings of the ACM on Networking},
  volume={3},
  number={CoNEXT4},
  pages={1--25},
  year={2025},
  publisher={ACM New York, NY, USA}
}
```
