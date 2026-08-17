//
// Created by xiang on 2021/10/9.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "laser_mapping.h"
#include "utils.h"

/// run faster-LIO in offline mode (rosbag2)

DEFINE_string(config_file, "./config/avia.yaml", "path to config file");
DEFINE_string(bag_file, "/data/bag", "path to the rosbag2 directory or file");
DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    LOG(WARNING) << "catch sig " << sig;
}

int main(int argc, char **argv) {
    // rclcpp claims "--ros-args ..." first, then gflags parses what is left.
    // Handing the raw argv to gflags instead makes it consume or reject the ROS
    // arguments.
    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char *> gflags_argv;
    gflags_argv.reserve(args.size());
    for (auto &arg : args) {
        gflags_argv.push_back(const_cast<char *>(arg.c_str()));
    }
    int gflags_argc = static_cast<int>(gflags_argv.size());
    char **gflags_argv_ptr = gflags_argv.data();

    gflags::ParseCommandLineFlags(&gflags_argc, &gflags_argv_ptr, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(gflags_argv_ptr[0]);

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    if (!laser_mapping->InitWithoutROS(FLAGS_config_file)) {
        LOG(ERROR) << "laser mapping init failed.";
        return -1;
    }

    /// handle ctrl-c
    signal(SIGINT, SigHandle);

    // just read the bag and send the data
    LOG(INFO) << "Opening rosbag, be patient";
    rosbag2_cpp::Reader reader;
    reader.open(FLAGS_bag_file);

    // topic name -> type
    std::unordered_map<std::string, std::string> topic_types;
    for (const auto &topic : reader.get_all_topics_and_types()) {
        topic_types[topic.name] = topic.type;
    }

    rclcpp::Serialization<livox_ros_driver::msg::CustomMsg> livox_serde;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc2_serde;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serde;

    LOG(INFO) << "Go!";
    while (reader.has_next()) {
        auto m = reader.read_next();
        const std::string &type = topic_types[m->topic_name];
        rclcpp::SerializedMessage serialized(*m->serialized_data);

        if (type == "livox_ros_driver/msg/CustomMsg") {
            auto livox_msg = std::make_shared<livox_ros_driver::msg::CustomMsg>();
            livox_serde.deserialize_message(&serialized, livox_msg.get());
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &livox_msg]() {
                    laser_mapping->LivoxPCLCallBack(livox_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
        } else if (type == "sensor_msgs/msg/PointCloud2") {
            auto point_cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
            pc2_serde.deserialize_message(&serialized, point_cloud_msg.get());
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &point_cloud_msg]() {
                    laser_mapping->StandardPCLCallBack(point_cloud_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
        } else if (type == "sensor_msgs/msg/Imu") {
            auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            imu_serde.deserialize_message(&serialized, imu_msg.get());
            laser_mapping->IMUCallBack(imu_msg);
        }

        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    /// print the fps
    double fps = 1.0 / (faster_lio::Timer::GetMeanTime("Laser Mapping Single Run") / 1000.);
    LOG(INFO) << "Faster LIO average FPS: " << fps;

    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    faster_lio::Timer::PrintAll();
    faster_lio::Timer::DumpIntoFile(FLAGS_time_log_file);

    rclcpp::shutdown();
    return 0;
}
