#include <ros/ros.h>
#include <livox_ros_driver2/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/Imu.h>
#include <iomanip>

class Converter {
public:
    Converter() {
        ros::param::param<std::string>("/livox_to_pcd_converter/save_dir", save_dir_, "/data/lg22/custom_data");
        ros::param::param<std::string>("/livox_to_pcd_converter/lidar_topic", lidar_topic_, "/livox/lidar");
        ros::param::param<std::string>("/livox_to_pcd_converter/imu_topic", imu_topic_, "/livox/imu");    
        lidar_sub_ = nh_.subscribe(lidar_topic_, 10000, &Converter::LiDARCallBack, this);
        imu_sub_ = nh_.subscribe(imu_topic_, 10000, &Converter::ImuCallBack, this);
        count_ = 0;
        timestamp_file_.open(save_dir_ + "/timestamp/timestamp.txt", std::ofstream::out);
        imu_file_.open(save_dir_ + "/imu/imu.txt", std::ofstream::out);
        first_imu_data_received_ = false;
    }
    ~Converter() {
        timestamp_file_.close();
    }


    void LiDARCallBack(const livox_ros_driver2::CustomMsg::ConstPtr& msg) {
        sensor_msgs::PointCloud2 pc2;
        pc2.header = msg->header;

        pcl::PointCloud<pcl::PointXYZINormal> pcl_cloud;
        
        // Assuming your CustomMsg contains an array of points with x, y, z
        pcl_cloud.reserve(msg->points.size());
        for (const auto& pt : msg->points) {
            pcl::PointXYZINormal point;
            point.x = pt.x;
            point.y = pt.y;
            point.z = pt.z;
            point.intensity = pt.reflectivity;
            point.curvature = pt.offset_time / float(1000000);
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

    void ImuCallBack(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // Check if this is the first IMU data
    if (!first_imu_data_received_) {
        // Write headers
        imu_file_ << "Timestamp ang_vel.x ang_vel.y ang_vel.z lin_acc.x lin_accn.y lin_acc.z" << std::endl;
        first_imu_data_received_ = true;
    }

        // Extract IMU data and save to the text file
        imu_file_ << imu_msg->header.stamp << " ";
        imu_file_ << imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << " ";
        imu_file_ << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << std::endl;
    }

    std::string generateFileName() {
        std::ostringstream ss;
        std::string pcdDir = save_dir_ + "/pcd/";
        ss << pcdDir << std::setw(5) << std::setfill('0') << count_ << ".pcd";
        return ss.str();
    }

    void saveTimestamp(const std::string& pcd_filename, const ros::Time& stamp) {
        timestamp_file_ << std::fixed << std::setprecision(6) << stamp.toSec() << std::endl;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    int count_;
    std::ofstream timestamp_file_;
    std::ofstream imu_file_;
    bool first_imu_data_received_;
    std::string save_dir_;
    std::string lidar_topic_;
    std::string imu_topic_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "livox_to_pcd_converter");
    Converter converter;
    
    ros::spin();

    return 0;
}