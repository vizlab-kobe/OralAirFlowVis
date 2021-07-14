#!/bin/sh
PROGRAM=rhoPimpleFoam

if [ -z $WM_DIR ]; then
    . ../etc/bashrc
fi

mpirun -n 48 ../$PROGRAM/$PROGRAM -parallel
