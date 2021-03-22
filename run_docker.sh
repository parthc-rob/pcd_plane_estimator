#!/bin/bash

if [ "$1" = "" ]
then
    HOST_DIR=/home/$USER/shared_dir/pcd_plane_estimator
else
    HOST_DIR=$1
fi

dockerUserName="user"


SHARED_DIR=/home/$dockerUserName/shared_dir/pcd_plane_estimator
##############
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

docker run --rm -it \
        --volume=$XSOCK:$XSOCK:rw \
        --volume=$XAUTH:$XAUTH:rw \
        --env="XAUTHORITY=${XAUTH}" \
        --env="DISPLAY" \
        --env="UID=`id -u $who`" \
        --env="UID=`id -g $who`" \
        --device /dev/nvidia0 --device /dev/nvidiactl -e DISPLAY=$DISPLAY \
        --gpus all \
        --privileged \
        --net=host\
        --volume=/dev:/dev \
        user/pcl:test \
        bash 
#--volume=$HOST_DIR:$SHARED_DIR:rw \
