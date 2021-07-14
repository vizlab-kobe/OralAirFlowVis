#!/bin/sh
PROGRAM=rhoPimpleFoam

if [ -z $WM_DIR ]; then
    . ../etc/bashrc
fi

mpirun -n 8 ../$PROGRAM/$PROGRAM -parallel
