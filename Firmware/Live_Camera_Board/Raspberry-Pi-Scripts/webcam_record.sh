#!/bin/bash

SAVE_DIR="/home/pi/webcam"
mkdir -p "$SAVE_DIR"

echo "Waiting for camera to initialize..."

sleep 5

while true; do
        if [ ! -e /dev/video0 ]; then
                echo  "No Camera Detected, retryimg in 5 seconds"
                sleep 5
                continue
        fi

        FILENAME="$SAVE_DIR/$(date +%Y-%m-%d_%H-%M-%S).mp4"
        echo "Recording Started: $FILENAME"

        ffmpeg \
                -f v4l2 \
                -input_format mjpeg \
                -framerate 20 \
                -video_size 1280x720 \
                -i /dev/video0 \
                -filter_complex "[0:v]split=2[rec][mon]; [rec]format=yuv420p[recout]; [mo>
                -map "[recout]" -c:v h264_v4l2m2m -b:v 4M -r 20 \
                -f segment -segment_time 120 -segment_format mp4 -strftime 1 \
                "$SAVE_DIR/%Y-%m-%d_%H-%M-%S.mp4" \
                -map "[monout]" -f fbdev /dev/fb0

        echo "Recording saved: $FILENAME"
done