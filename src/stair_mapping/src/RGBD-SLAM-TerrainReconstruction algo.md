TODO: RGBD-SLAM-TerrainReconstruction pipeline

1. Get color and depth image
2. Estimate the transform between two consecutive images
    2.1. Normal estimation of consecutive RGBD images
    2.2. Transform the input image with initial guess from legged odometry 
    2.3. Projective pairing of two images
    2.3. Calculate transform with ICP (point to plane metric or colored ICP)
3. Transform input image with the result
4. Run global optimization with loop closure and IMU correction
5. Integrate the RGBD images with transforms to build a patch/a TSDF volume

In-door slam and navigation
    1. Speed improvement at path following
Out-door slam and navigation
    1. Relocation
