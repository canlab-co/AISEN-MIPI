#!/bin/bash
# Runtime media-ctl setup for the single-channel CANLAB source:
#   canlab pad0 -> csi2 sink0 -> csi2 source4 -> rp1-cfe-csi2_ch0 (/dev/videoN)
# Run after every boot (media-ctl state is not persistent).
set -e

FMT="UYVY8_1X16/640x480"

MDEV=""
for m in /dev/media*; do
    if media-ctl -d "$m" -p 2>/dev/null | grep -q "model *rp1-cfe"; then
        MDEV=$m
        break
    fi
done
[ -n "$MDEV" ] || { echo "rp1-cfe media device not found"; exit 1; }

CAMSD=$(media-ctl -d "$MDEV" -p | grep -oE "aisen [0-9]+-0033" | head -n1)
[ -n "$CAMSD" ] || { echo "aisen subdev not found on $MDEV"; exit 1; }

# video node behind csi2_ch0
VDEV=$(media-ctl -d "$MDEV" -e "rp1-cfe-csi2_ch0")
[ -n "$VDEV" ] || { echo "rp1-cfe-csi2_ch0 node not found"; exit 1; }

echo "media: $MDEV  sensor: '$CAMSD'  video: $VDEV"

# Only one link needed: csi2 source4 -> ch0 (VC1 fan-out ch2 is not used).
media-ctl -d "$MDEV" -l "'csi2':4 -> 'rp1-cfe-csi2_ch0':0 [1]"

# Same format at every point on the path (sensor pad0, csi2 sink0, csi2 src4).
# csi2 sink1/sink2 are unlinked with a single-pad sensor, so no mirroring.
media-ctl -d "$MDEV" -V "'$CAMSD':0 [fmt:$FMT field:none]"
media-ctl -d "$MDEV" -V "'csi2':0 [fmt:$FMT field:none]"
media-ctl -d "$MDEV" -V "'csi2':4 [fmt:$FMT field:none]"

v4l2-ctl -d "$VDEV" --set-fmt-video=width=640,height=480,pixelformat=UYVY

echo "done. test:"
echo "  gst-launch-1.0 v4l2src device=$VDEV ! video/x-raw,format=UYVY,width=640,height=480 ! xvimagesink"
