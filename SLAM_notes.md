# Graph structure

* Nodes ($x_i$): Represents the state of the robot (position $x, y, z$ and orientation quaternion) at time $i$.

* Edges ($z_{ij}$): Constraints connecting nodes.

    * Odometry Edges: Connect $x_i$ to $x_{i+1}$. These have low uncertainty locally but accumulate drift.

    * Loop Closure Edges: Connect $x_{current}$ to $x_{old}$. These happen when the vision system recognizes a place.

# Visual feature extraction from RGB images
RTAB-MAP uses a Bag-of-Words (BoW) approach with visual features (like SURF, SIFT, or ORB) from your D435 RGB images.

Feature Extraction: Convert image $I_t$ into a set of descriptors.

Quantization: Map descriptors to a "Visual Vocabulary." 才能分類到一些已知的類別(descriptors)中

Likelihood: It calculates a probability score that the current image $I_t$ is the same as a past image $I_j$. 讓robot知道回到同個地方了

ORB: 傳統古典的方法，先試試，太爛的話可以換deep learning方法，文獻指出降低誤差一個數量級

https://www.emergentmind.com/topics/superpoint-slam3


# 3. Implementation Strategy: Your Hardware Setup
You have a hybrid setup (LiDAR + Depth Camera). This allows you to use RGB-D for Loop Closure and LiDAR for the Occupancy Grid.


Step 1: Sensor Roles
* Wheel Odometry: Provides the base odom $\rightarrow$ base_link tf. This is your "guess" for movement.
* RealSense D435:
    * RGB: Used for Loop Closure Detection. It captures features to recognize "I have been here before."
    * Depth: Used for Proximity Detection (to ensure the loop closure is geometrically valid, not just visually similar).
* LiDAR:
    * Used for metric correctness. While the camera handles the graph logic, the LiDAR creates the 2D occupancy grid (the black and white map you use for navigation).
    * Optional: You can use ICP (Iterative Closest Point) with the LiDAR to refine the odometry edges (Scan Matching).

Step 2: TF Tree (Crucial)

You must ensure your TF tree is broadcast correctly. map $\rightarrow$ odom $\rightarrow$ base_link $\rightarrow$ camera_link / lidar_link

RTAB-MAP publishes the map $\rightarrow$ odom transform. Your base driver publishes odom $\rightarrow$ base_link.

Step 3: Launch Configuration (ROS 2 Example Logic)

You will likely use the rtabmap_ros package. You need to set specific parameters to utilize your hardware:
