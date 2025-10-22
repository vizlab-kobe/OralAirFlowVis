#!/bin/sh

./VideoCreate 5 0 0.75 0.25 0
ffmpeg -framerate 20 -pattern_type glob -i "../realistic-s3/ex_Output/Output/output_*_999999_*.bmp" -vcodec libx264 -pix_fmt yuv420p -r 60 40*40_5_0_0.75_0.25_0.mp4