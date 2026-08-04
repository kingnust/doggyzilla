#!/bin/bash
xhost +

docker run -it  \
--privileged=true \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
--security-opt apparmor:unconfined \
-v /dev/ttyAMA1:/dev/ttyAMA1 \
-v /dev/ttyAMA0:/dev/ttyAMA0 \
-v /dev/video0:/dev/video0 \
-v /dev/input:/dev/input \
-v /dev/myspeech:/dev/myspeech \
-v /dev/bus/usb/:/dev/bus/usb/ \
yahboomtechnology/ros-humble:3.8 /bin/bash 

