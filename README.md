Single Plane Estimator
-------------------------

This package estimates the equation of a single plane from a given `.pcd`
pointcloud file.

The outputs are a normal pointing from a given camera distance towards the
plane, a coordinate frame attached to the plane, and a cuboid representing a
camera view such that the coordinate frame always appears with +Y axis as "up". 

Full dataset for objects to test on can be found [here](https://github.com/PointCloudLibrary/data/tree/master/segmentation/mOSD)

To run -

1. Clone, `cd` into folder, do `chmod +x *.sh`
2. `./dockerbuild.sh` - single script to build docker container, package and binaries. \n
         Alternatively use `./pull_docker.sh` to get the pre-built docker image from DockerHub.
3. `./run_docker.sh` to open docker, be able to calculate planes for different .pcd files
4. `./build_cpp.sh` to build the code.
5. `calculate_plane.sh` - single script to run and visualize plane calculation on sample PCD file with a single sparse plane.
        Append flags `-v` to enable visualization, `-d` to enable debug prints.

4. (optional) `run_tests.sh` to run unit tests.

Note : Due to OpenGL incompatibility with docker, pcl_viewer won't run inside docker - no visualization for plane inside docker.

For on-host plane visualization, optionally `git checkout test`, run `deps.sh` to install dependencies in host, `build_cpp.sh` to build package, then `calculate_plane.sh`.

**Updated 2024-02-23 :** This would require a linux host - visualization is currently not functional on MacOS M1 (arm64) docker host machines.

------------
Sample output planes

![learn11_plane.pcd](https://github.com/parthc-rob/pcd_plane_estimator/blob/no_visualize_with_docker/output_images/plane_estimate_learn11_plane.png?raw=true)

![object_template2.pcd](https://github.com/parthc-rob/pcd_plane_estimator/blob/no_visualize_with_docker/output_images/object_template2_plane_detection.png?raw=true)
