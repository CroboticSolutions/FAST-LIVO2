/**
 * @file pointcloud_to_livox.cpp
 * @brief ROS node that republishes PointCloud messages as Livox CustomMsg format
 * 
 * Subscribes to: /lidar3d/points (sensor_msgs/PointCloud)
 * Publishes to: /livox/lidar (livox_ros_driver/CustomMsg)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <livox_ros_driver/CustomMsg.h>
#include <livox_ros_driver/CustomPoint.h>

class PointCloudToLivox
{
public:
    PointCloudToLivox() : nh_("~")
    {
        // Get parameters
        nh_.param<std::string>("input_topic", input_topic_, "/lidar3d/points");
        nh_.param<std::string>("output_topic", output_topic_, "/livox/lidar");
        nh_.param<int>("lidar_id", lidar_id_, 0);

        // Setup subscriber and publisher
        sub_ = nh_.subscribe(input_topic_, 10, &PointCloudToLivox::pointcloudCallback, this);
        pub_ = nh_.advertise<livox_ros_driver::CustomMsg>(output_topic_, 10);

        ROS_INFO("PointCloud to Livox republisher started");
        ROS_INFO("  Subscribing to: %s", input_topic_.c_str());
        ROS_INFO("  Publishing to: %s", output_topic_.c_str());
    }

private:
    void pointcloudCallback(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
    {
        livox_ros_driver::CustomMsg livox_msg;

        // Fill header
        livox_msg.header = cloud_msg->header;
        
        // Set timebase from header timestamp (in nanoseconds)
        livox_msg.timebase = static_cast<uint64_t>(cloud_msg->header.stamp.sec) * 1000000000ULL 
                           + static_cast<uint64_t>(cloud_msg->header.stamp.nsec);
        
        // Set lidar id
        livox_msg.lidar_id = static_cast<uint8_t>(lidar_id_);
        
        // Reserved bytes set to 0
        livox_msg.rsvd[0] = 0;
        livox_msg.rsvd[1] = 0;
        livox_msg.rsvd[2] = 0;

        // Get number of points
        uint32_t point_count = cloud_msg->points.size();
        livox_msg.point_num = point_count;
        livox_msg.points.reserve(point_count);

        // Check if intensity channel exists
        int intensity_channel_idx = -1;
        int ring_channel_idx = -1;
        for (size_t i = 0; i < cloud_msg->channels.size(); ++i)
        {
            if (cloud_msg->channels[i].name == "intensity")
                intensity_channel_idx = static_cast<int>(i);
            if (cloud_msg->channels[i].name == "ring")
                ring_channel_idx = static_cast<int>(i);
        }

        // Convert each point
        for (size_t i = 0; i < point_count; ++i)
        {
            livox_ros_driver::CustomPoint point;
            
            // Set offset time (0 for all points since we don't have timing info per point)
            point.offset_time = 0;
            
            // Set coordinates
            point.x = cloud_msg->points[i].x;
            point.y = cloud_msg->points[i].y;
            point.z = cloud_msg->points[i].z;
            
            // Set reflectivity from intensity channel if available
            if (intensity_channel_idx >= 0 && i < cloud_msg->channels[intensity_channel_idx].values.size())
            {
                // Clamp intensity to 0-255 range
                float intensity = cloud_msg->channels[intensity_channel_idx].values[i];
                point.reflectivity = static_cast<uint8_t>(std::min(255.0f, std::max(0.0f, intensity)));
            }
            else
            {
                point.reflectivity = 0;
            }
            
            // Set tag (default 0)
            point.tag = 0;
            
            // Set line (ring/laser number) if available
            if (ring_channel_idx >= 0 && i < cloud_msg->channels[ring_channel_idx].values.size())
            {
                point.line = static_cast<uint8_t>(cloud_msg->channels[ring_channel_idx].values[i]);
            }
            else
            {
                point.line = 0;
            }
            
            livox_msg.points.push_back(point);
        }

        // Publish the converted message
        pub_.publish(livox_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    std::string input_topic_;
    std::string output_topic_;
    int lidar_id_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_to_livox");
    
    PointCloudToLivox converter;
    
    ros::spin();
    
    return 0;
}
