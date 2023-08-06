#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iomanip>

class Converter {
public:
    Converter() {
        sub_ = nh_.subscribe("/livox/lidar", 10000, &Converter::callback, this);
        count_ = 0;
        timestamp_file_.open("/root/workspace/data/timestamp/timestamp.txt", std::ofstream::out);
    }
    ~Converter() {
        timestamp_file_.close();
    }


    void callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
        sensor_msgs::PointCloud2 pc2;
        pc2.header = msg->header;

        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        
        // Assuming your CustomMsg contains an array of points with x, y, z
        pcl_cloud.reserve(msg->points.size());
        for (const auto& pt : msg->points) {
            pcl::PointXYZ point;
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            pcl_cloud.points.push_back(point);
        }
        
        pcl_cloud.width = pcl_cloud.points.size(); // Set the width to the number of points
        pcl_cloud.height = 1; // Unorganized point cloud
        
        pcl::toROSMsg(pcl_cloud, pc2);

        // Save to PCD
        std::string pcd_filename = generateFileName();
        pcl::io::savePCDFileASCII(pcd_filename, pcl_cloud);
        
        // Save timestamp
        saveTimestamp(pcd_filename, msg->header.stamp);

        count_++;
        ROS_INFO("Save %d.pcd file. Time : %f", count_, msg->header.stamp.toSec());
    }


    std::string generateFileName() {
        std::ostringstream ss;
        std::string pcdDir = "/root/workspace/data/pcd/";
        ss << pcdDir << std::setw(5) << std::setfill('0') << count_ << ".pcd";
        return ss.str();
    }

    void saveTimestamp(const std::string& pcd_filename, const ros::Time& stamp) {
        timestamp_file_ << std::fixed << std::setprecision(6) << stamp.toSec() << std::endl;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    int count_;
    std::ofstream timestamp_file_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_to_pcd_converter");
    Converter converter;
    
    ros::spin();

    return 0;
}