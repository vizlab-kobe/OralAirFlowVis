#!/bin/sh
PS_COUNT=`ps aux | grep rhoPimpleFoam | grep -v grep | wc -l`
if [ $PS_COUNT -gt 0 ]; then
    killall rhoPimpleFoam
fi

OUTPUT_DIR=Output
if [ -e $OUTPUT_DIR ]; then
    rm -rf $OUTPUT_DIR
    rm -rf processor?/0.0*
    rm -rf processor?/*e-*
fi
