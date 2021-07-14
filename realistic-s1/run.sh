#!/bin/sh
PROGRAM=rhoPimpleFoam

if [ -z $WM_DIR ]; then
    . ../etc/bashrc
fi

mpirun -n 4 ../$PROGRAM/$PROGRAM -parallel
