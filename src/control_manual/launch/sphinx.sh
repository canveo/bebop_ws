# #!/bin/bash

# gnome-terminal --window-with-profile=scripting sudo systemctl start firmwared.service | sudo firmwared

# # gnome-terminal --window-with-profile=scripting -e roscore
# # sleep 3
# # RUN firmware script in second
# # gnome-terminal --window-with-profile=scripting -e sh /home/bebop_ws/src/control_manual/launch/firmwared/firmwared_terminal.sh

# sleep 5
# # RUN gazebo in third

# gnome-terminal --window-with-profile=scripting -e sudo sphinx /opt/parrot-sphinx/usr/share/sphinx/worlds/empty.world  /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone::with_front_cam=false

# sleep 5
# # RUN Bebop ros node in fourth
# gnome-terminal --window-with-profile=scripting -e roslaunch bebop_driver bebop_sphinx.launch

# # RUN Roscore in first terminal
# gnome-terminal --window-with-profile=scripting -e echo "hola, $USER"
# sleep 3
# # RUN firmware script in second
gnome-terminal --window-with-profile=scripting -e sh /home/bebop_ws/src/control_manual/launch/firmwared/firmwared_terminal.sh

sleep 5
# RUN gazebo in third
gnome-terminal --window-with-profile=scripting -e sudo sphinx /opt/parrot-sphinx/usr/share/sphinx/worlds/empty.world /opt/parrot-sphinx/usr/share/sphinx/drones/bebop2.drone::with_front_cam=false
# RUN Bebop ros node in fourth
gnome-terminal --window-with-profile=scripting -e roslaunch bebop_driver bebop_sphinx.launch