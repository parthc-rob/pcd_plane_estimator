Single Plane Estimator
-------------------------

This package estimates the equation of a single plane from a given `.pcd`
pointcloud file.

The outputs are a normal pointing from a given camera distance towards the
plane, a coordinate frame attached to the plane, and a cuboid representing a
camera view such that the coordinate frame always appears with +Y axis as "up". 

Full dataset for objects to test on can be found [here](https://github.com/PointCloudLibrary/data/tree/master/segmentation/mOSD)

To run -

1. Clone, `cd` into folder, do `chmod +x *.sh` and `./deps.sh` to install dependencies
2. `build_cpp.sh` - single script to build the package and binaries
3. `calculate_plane.sh` - single script to run and visualize plane calculation on sample PCD file with a single sparse plane.

4. (optional) `run_tests.sh` to run unit tests.
