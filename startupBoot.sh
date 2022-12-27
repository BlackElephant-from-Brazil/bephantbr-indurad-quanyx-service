#!/bin/bash
mkdir -p /tmp/runtime-root
chmod 777 -R /tmp/runtime-root/
export XDG_RUNTIME_DIR=/tmp/runtime-root
export RUNLEVEL=3
export XAUTHORITY=/run/user/1000/gdm/Xauthority
export DISPLAY=:0
xhost
cd /usr/bephantbr-indurad-quanyx-service/src/build/
./SERVICE
