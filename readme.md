Minimalistic example how to subscribe to PointCloud2 messages, transform it (shift+rotate), and publish it back

Installation and sample run:
1. Clone this repo in your catkin workspase/src folder (e.g. /home/user/catkin_ws/src/)
2. Compile by calling "catkin_make" from "/home/user/catkin_ws"
3. Run by calling "rosrun transform_point_cloud_simple node"
4. Input data should be published in topic "/camera/depth_registered/points" (default path for Intel Realsense cameras launched by rs_rgbd.launch
4. Check result in RViz in topic "/point_cloud_transformed"
