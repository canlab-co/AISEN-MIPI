# AISEN-MIPI

# CANLAB AISEN driver for Raspberry Pi 5

Raspberry Pi 5 (RP1 CFE) V4L2 driver for the AISEN MIPI CSI-2 camera
(640x480 UYVY, VC0, continuous clock, 4-lane or 2-lane).

- aisen channel: 640x480 UYVY -> `rp1-cfe-csi2_ch0` (/dev/videoN)
- Lanes: 4 (default) or 2 (`2lane` option, must match the firmware flashed on the camera)

Distributed as source + DKMS: the module is rebuilt automatically on the
target for the running kernel, including after kernel upgrades
(`AUTOINSTALL="yes"`), so no per-kernel binary releases are needed.

## Install

```
sudo apt install -y git
sudo apt install -y --no-install-recommends dkms
cd ~
git clone https://github.com/canlab-co/AISEN-MIPI.git
cd AISEN-MIPI/
sudo ./setup.sh
```

The repository root must contain exactly one driver source (`aisen.c`) and
one overlay source (`canlab-aisen-overlay.dts`); the Makefile and `setup.sh`
pick up `*.c` / `*-overlay.dts` automatically.

## Boot configuration

Edit `/boot/firmware/config.txt`:

1. Set `camera_auto_detect` to `0`:

```
camera_auto_detect=0
```

2. Add under the `[all]` section:

```
[all]
dtoverlay=canlab-aisen
```

Reboot for changes to take effect.

## dtoverlay options

### default
Base overlay with no options — `dtoverlay=canlab-aisen` — uses:

| setting | value                  |
| ------- | ---------------------- |
| port    | Use CAM1 port          |
| lanes   | 4 data lanes           |

### options
Add these parameters to change the defaults:

| option  | description                                   |
| ------- | --------------------------------------------- |
| `cam0`  | Use CAM0 port instead of CAM1                 |
| `2lane` | Use 2 data lanes (sensor and CSI `data-lanes` become `<1 2>`) |

Options can be combined: `dtoverlay=canlab-aisen,2lane,cam0`

The lane setting must match the firmware actually flashed on the camera
(4-lane vs 2-lane firmware).

## Runtime pipeline setup

Make the script executable before first use:

```
chmod a+x setup-aisen.sh
```

After every boot, configure the media pipeline (link + formats) before
streaming:

```
sudo ./setup-aisen.sh
```

The script finds the `rp1-cfe` media device, the `aisen 11-0033` subdevice and
the video node behind `rp1-cfe-csi2_ch0`, then sets `UYVY8_1X16/640x480` along
the path. It prints the video node to use.

Verify the driver probed:

```
# aisen 11-0033: aisen: VC0 640x480 (pad0/ch0) UYVY, 4 lanes, 200000000 Hz continuous
```

To confirm the lane option reached the final device tree:

```
dtc -I fs /proc/device-tree 2>/dev/null | grep -A8 "aisen@33"
```

## Streaming

GStreamer is not installed by default on Raspberry Pi OS. Install it first:

```
sudo apt install -y gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good
```

Use the video node printed by `setup-aisen.sh`:

```
gst-launch-1.0 \
  v4l2src device=/dev/videoN ! video/x-raw,format=UYVY,width=640,height=480 ! xvimagesink sync=false
```

## Uninstall

```
sudo dkms remove canlab-rpi-dkms/1.0.0 --all
sudo rm /boot/firmware/overlays/canlab-aisen.dtbo
# remove the dtoverlay line from config.txt
```
