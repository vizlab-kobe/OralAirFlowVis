#!/bin/sh
OUTPUT_DIR="Output/Process0000"
for i in `seq 0 26`
do
 index=$(printf "%06d" $i)
 ffmpeg -framerate 60 -pattern_type glob -i "${OUTPUT_DIR}/output_*_${index}.bmp" -vcodec libx264 -pix_fmt yuv420p -r 60 output_${index}.mp4
done
