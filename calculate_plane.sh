#!/bin/bash

unset DEBUG
unset VISUALIZE_PCL
while getopts 'dvb:' OPTION; do
  case "$OPTION" in
    d)
        export DEBUG=1
      ;;
    v)
        export VISUALIZE_PCL=1
      ;;
    b)
        ./build_cpp.sh
      ;;
    ?)
      echo "------ NOTE: Turn on visualizations with -v, debug prints with -d, recompilation with -b ------\n"
      #echo "script usage: $(basename \$0) [-g] [-c]" >&1
      ;;
  esac
done
shift "$(($OPTIND -1))"

#gdb --args ./build/pcl_plane_calculation ./data/learn11_plane.pcd
./build/pcl_plane_calculation ./data/learn11_plane.pcd
