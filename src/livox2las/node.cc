#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <livox_ros_driver2/CustomMsg.h>
// #include <laslib/laswriter.hpp>
#include <boost/filesystem.hpp>

std::string output_folder = "/path/to/output_folder";

void callback(const livox_ros_driver2::CustomMsg::ConstPtr& msg)
{
    // // Convert the ROS timestamp to a string
    // std::ostringstream ss;
    // ss << msg->header.stamp;
    // std::string timestamp_str = ss.str();

    // // Create the output filepath
    // std::string output_filepath = (boost::filesystem::path(output_folder) / (timestamp_str + ".las")).string();

    // // Create a LASwriter
    // LASwriteOpener laswriteopener;
    // laswriteopener.set_file_name(output_filepath.c_str());
    // LASwriter* laswriter = laswriteopener.open();

    // // Convert the livox_ros_driver2/CustomMsg message to sensor_msgs/PointCloud and save to .las
    // for (int i = 0; i < msg->points.size(); i++)
    // {
    //     // Create a LASpoint
    //     LASpoint laspoint;
    //     laspoint.set_x(msg->points[i].x);
    //     laspoint.set_y(msg->points[i].y);
    //     laspoint.set_z(msg->points[i].z);

    //     // Write the LASpoint to the file
    //     laswriter->write_point(&laspoint);
    // }

    // // Close the LASwriter
    // laswriter->close();
    // delete laswriter;
}

int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "las_writer");
    ros::NodeHandle nh;

    // Subscribe to livox_ros_driver2/CustomMsg messages
    ros::Subscriber sub = nh.subscribe("input_topic", 1, callback);

    // Enter a loop, pumping callbacks
    ros::spin();

    return 0;
}