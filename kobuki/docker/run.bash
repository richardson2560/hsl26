#!/bin/bash

# Written by Nikolay Dema <ndema2301@gmail.com>, Jun 2025

KOBUKI_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

xhost +local:docker > /dev/null || true

IMG_NAME="kobuki_100625_sol"
CTR_NAME="kobuki"


### DOCKER RUN ----------------------------------------------------------- #

docker run  -d -ti --rm                            \
            -e "DISPLAY"                           \
            -e "QT_X11_NO_MITSHM=1"                \
            -e XAUTHORITY                          \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw    \
            -v /etc/localtime:/etc/localtime:ro    \
            -v ${KOBUKI_ROOT}/workspace:/workspace \
            -v /dev:/dev                           \
            --net=host                             \
            --privileged                           \
            --name ${CTR_NAME} ${IMG_NAME}         \
            > /dev/null
