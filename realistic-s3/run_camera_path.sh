#!/bin/sh
PROGRAM=rhoPimpleFoam

if [ -z $WM_DIR ]; then
    . ../etc/bashrc
fi

unset FOAM_SIGFPE
#export FOAM_SIGFPE=false
mpirun -n 8 ../${PROGRAM}_CameraPath/$PROGRAM -parallel

#if type "ffmpeg" > /dev/null 2>&1; then
#    ffmpeg -r 60 -start_number 000001 -i Output/output_%06d.bmp -vcodec libx264 -pix_fmt yuv420p -r 60 output.mp4
#fi
