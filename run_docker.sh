#!/bin/bash

if [ "$1" = "" ]
then
    HOST_DIR="$PWD"
else
    HOST_DIR=$1
fi

while getopts 'g:' OPTION; do
  case "$OPTION" in
    g)
      echo "Using GPUs (need to be NVIDIA)"
      GPU="gpus=all"
      ;;
    ?)
      echo "Using CPU only"
      GPU=""
      #echo "script usage: $(basename \$0) [-g]" >&1
      #exit 1
      ;;
  esac
done
shift "$(($OPTIND -1))"

dockerUserName="user"

SHARED_DIR=/code-sample

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
        $GPU \
        --privileged \
        --net=host\
        --volume=/dev:/dev \
        --volume=$HOST_DIR:$SHARED_DIR \
        parthc/minimal_pcl_cpp_linux:develop \
        bash 
