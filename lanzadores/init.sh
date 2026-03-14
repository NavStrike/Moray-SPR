#!/bin/bash
cd /home/pi/spr 
source ./venv/bin/activate
# export QT_QPA_GENERIC_PLUGINS=evdevtouch:/dev/input/event5
python ./stableVersion/main.py