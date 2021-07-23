#!/bin/bash
SESSION=$USER

pkill -SIGINT roslaunch
gnome-terminal -e kill-session -t $SESSION
gnome-terminal -e kill-session -a