#include <ros/ros.h>

#include <tf/tf.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

class Tf_lidar2map
{
public:
  Tf_lidar2map();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Subscriber object_sub_;
  ros::Publisher tf_points_pub_;

  tf::TransformListener* tf_listener_;

  pcl::KdTreeFLANN<pcl::PointXYZI> tree_;

  double distance_threshold_;
  double min_clipping_height_;
  double max_clipping_height_;

  std::string map_frame_;
  std::string input_topic_;
  std::string output_topic_;

  void objectCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_cloud_msg_ptr);
};

Tf_lidar2map::Tf_lidar2map()
  : nh_()
  , nh_private_("~")
  , tf_listener_(new tf::TransformListener)
{   
    nh_private_.getParam("publish_frame", map_frame_);
    nh_private_.getParam("input_rgbpoints", input_topic_);
    nh_private_.getParam("output_rgbpoints", output_topic_);

    std::cout << "input_point_topics : " << input_topic_ << std::endl;
    std::cout << "input_rgbpoints : " << input_topic_ << std::endl;
    std::cout << "output_rgbpoints : " << output_topic_ << std::endl;

    object_sub_ = nh_.subscribe(input_topic_, 10, &Tf_lidar2map::objectCallback, this);
    tf_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
}

void Tf_lidar2map::objectCallback(const sensor_msgs::PointCloud2::ConstPtr& sensor_cloud_msg_ptr){
    const ros::Time sensor_time = sensor_cloud_msg_ptr->header.stamp;
    const std::string sensor_frame = sensor_cloud_msg_ptr->header.frame_id;

    pcl::PointCloud<pcl::PointXYZI>::Ptr sensorTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*sensor_cloud_msg_ptr, *sensorTF_cloud_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr mapTF_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    try{
        tf_listener_->waitForTransform(map_frame_, sensor_frame, sensor_time, ros::Duration(3.0));
        pcl_ros::transformPointCloud(map_frame_, sensor_time, *sensorTF_cloud_ptr, sensor_frame, *mapTF_cloud_ptr, *tf_listener_);
    }catch(tf::TransformException& ex){
        ROS_ERROR("Transform error : %s", ex.what());
        return;
    }

    sensor_msgs::PointCloud2 mapTF_tf_cloud_msg;

    pcl::toROSMsg(*mapTF_cloud_ptr, mapTF_tf_cloud_msg);

    mapTF_tf_cloud_msg.header.stamp = sensor_time;
    mapTF_tf_cloud_msg.header.frame_id = map_frame_;
    mapTF_tf_cloud_msg.fields = sensor_cloud_msg_ptr->fields;

    tf_points_pub_.publish(mapTF_tf_cloud_msg);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "tf_lidar2map");
    Tf_lidar2map node;
    ros::spin();

    return 0;
}