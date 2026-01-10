#!/bin/sh
PROGRAM=rhoPimpleFoam

if [ -z $WM_DIR ]; then
    . ../etc/bashrc
fi

wclean

if [ -e $PROGRAM ]; then
    rm $PROGRAM
fi
