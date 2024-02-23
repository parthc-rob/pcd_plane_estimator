#!/bin/bash

echo "Specify your own docker tag name as first argument"
if [ "$1" = "" ]
then
    DOCKER_TAG=parthc/minimal_pcl_cpp_linux:develop
else
    DOCKER_TAG=$1
fi
docker build -t $DOCKER_TAG .
