Single Plane Estimator
-------------------------

This package estimates the equation of a single plane from a given `.pcd`
pointcloud file.

The outputs are a normal pointing from a given camera distance towards the
plane, a coordinate frame attached to the plane, and a cuboid representing a
camera view such that the coordinate frame always appears with +Y axis as "up". 

Dataset for objects to test on can be found [here](https://github.com/PointCloudLibrary/data/tree/master/segmentation/mOSD)
