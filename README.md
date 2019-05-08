# Stereo-Visual-Odometry
This is the implementation of Visual Odometry using the stereo image sequence from the KITTI dataset

Visual Odometry in action (Click on image to play video): 
[![Watch the video](https://github.com/akshay-iyer/Stereo-Visual-Odometry/blob/master/tn.png)](https://www.youtube.com/watch?v=B-6oqZwLLEs&t=4s)

Link to dataset - https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_28_drive_0001/2011_09_28_drive_0001_sync.zip

Visual Odometry is the process of incrementally estimating the pose of a vehicle using the images obtained from the onboard cameras. Its applications include, but are not limited to, robotics, augmented reality, wearable computing, etc. In this work, we implement stereo visual odometry using images obtained from the KITTI Vision Benchmark Suite and present the results the approache. We implement stereo visual odometry using 3D-2D feature correspondences. We find that between frames, using a combination of feature matching and feature tracking is better than implementing only feature matching or only feature tracking. Also, we find that stereo odometry is able a reliable trajectory without the need of an absolute scale as expected.

## Problem Formulation:

### Input
Our input consists of a stream of gray scale or color images obtained from a pair of cameras. This data is obtained from the KITTI Vision Benchmark Suite. Let the pair of images captured at time k and k+1 be (Il,k, Ir,k) and (Il,k+1, Ir,k+1 ) respectively. The intrinsic and extrinsic parameters of the cameras are obtained via any of the available stereo camera calibration algorithms or the dataset

### Output
For every stereo image pair we receive after every time step we need to find the rotation matrix R and translation vector t, which together describes the motion of the vehicle between two consecutive frames. 


## Algorithm Outline:
1. Read left (Il,0) and right (Ir,0) images of the initial car position
2. Match features between the pair of images 
3. Triangulate matched feature keypoints from both images
4. Iterate:
  a. Track keypoints of Il,k in Il,k+1
  b. Select only those 3D points formed from Il,k and Ir,k  which correspond to keypoints tracked in Il,k+1
  c. Calculate rotation and translation vectors using PNP from the selected 3D points and tracked feature keypoints in Il,k+1
  d. Calculate inverse transformation matrix, inverse rotation and inverse translation vectors to obtain coordinates of camera      with respect to world
  e. The inverse rotation and translation vectors give the current pose of the vehicle in the initial world coordinates. Plot the elements of the inverse translation vector as the current position of the vehicle
  f. Create new features for the next frame:
     i.  Read left (Il,k+1) and right (Ir,k+1) images 
     ii. Match features between the pair of images 
     iii.Triangulate matched feature keypoints from both images
     iv. Multiply the triangulated points with the inverse transform calculated in step (d) and form new triangulated points
     v.  Repeat from step 4.a 



