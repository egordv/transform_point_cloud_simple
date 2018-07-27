#include <iostream>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>


//Global publisher variable
ros::Publisher point_cloud_transformed_pub;

// Ros msg callback
void processCloud(const sensor_msgs::PointCloud2 msg)
{
        std::cout<<"Point cloud arrived" << std::endl;

        // Retrieve cloud msg and convert it to usable cloud for ROS.
        pcl::PCLPointCloud2 msg_cloud;

        //Convert sensor_msgs::PointCloud to pcl::PCLPointCloud2
        pcl_conversions::toPCL(msg, msg_cloud);

        //Convert pcl::PCLPointCloud2 to pcl::PointCloud<T>
        pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
        pcl::fromPCLPointCloud2(msg_cloud, pcl_cloud);

        //Declare transformed point cloud in pcl::PointCloud format
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

        //Declare transformation (shift + rotate)
        Eigen::Affine3f my_transform = Eigen::Affine3f::Identity();

        //Calculate transform matrix using Eigen for simplicity
        float angle = -23.0*(M_PI/180.0); // The angle of rotation in radians of RealSense camera around X axis
        my_transform.translation() << -0.03, 0.0, 0.0; //Also do some shift by 0.03 meters along X axis
        my_transform.rotate (Eigen::AngleAxisf (angle, Eigen::Vector3f::UnitX()));

        //Do transform point cloud
        pcl::transformPointCloud (pcl_cloud, *transformed_cloud, my_transform);

        //Publishing transformed point cloud
        point_cloud_transformed_pub.publish(transformed_cloud);
		
}


int main(int argc, char **argv){	

	// Initializing ros parameters and subscribing to the topic.
        ros::init(argc,argv, "transform_point_cloud_simple");

        //Initialise ROS node
        ros::NodeHandle n;

        std::cout<<"Subscribing to topic /camera/depth_registered/points"<<std::endl;

        //Subscribe and register callback function
        ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points",1, processCloud);

        //Advertise publisher
        point_cloud_transformed_pub = n.advertise<sensor_msgs::PointCloud2> ("/point_cloud_transformed", 1);


	ros::spin(); 
	return 0;
}
