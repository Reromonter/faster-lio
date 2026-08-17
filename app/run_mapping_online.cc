//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"

/// run the lidar mapping in online mode

DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    LOG(WARNING) << "catch sig " << sig;
}

int main(int argc, char **argv) {
    // rclcpp claims "--ros-args ..." first, then gflags parses what is left.
    // Handing the raw argv to gflags instead makes it consume or reject the ROS
    // arguments, and the node silently comes up with default parameters.
    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    std::vector<char *> gflags_argv;
    gflags_argv.reserve(args.size());
    for (auto &arg : args) {
        gflags_argv.push_back(const_cast<char *>(arg.c_str()));
    }
    int gflags_argc = static_cast<int>(gflags_argv.size());
    char **gflags_argv_ptr = gflags_argv.data();

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(gflags_argv_ptr[0]);
    google::ParseCommandLineFlags(&gflags_argc, &gflags_argv_ptr, true);

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    laser_mapping->InitROS();

    signal(SIGINT, SigHandle);
    rclcpp::Rate rate(5000);

    // online, almost same with offline, just receive the messages from ros
    while (rclcpp::ok()) {
        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
        rclcpp::spin_some(laser_mapping);
        laser_mapping->Run();
        rate.sleep();
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    faster_lio::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    rclcpp::shutdown();
    return 0;
}
