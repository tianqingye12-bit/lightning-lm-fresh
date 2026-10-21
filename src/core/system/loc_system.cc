//
// Created by xiang on 25-9-12.
//

#include "core/system/loc_system.h"
#include "core/localization/localization.h"
#include "io/yaml_io.h"
#include "wrapper/ros_utils.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <sstream>
#include <rclcpp/executors/multi_threaded_executor.hpp>

namespace lightning {

LocSystem::LocSystem(LocSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

LocSystem::~LocSystem() {
    loc_started_.store(false);
    if (watchdog_) watchdog_->cancel();
    if (loc_) loc_->Finish();
}

bool LocSystem::Init(const std::string &yaml_path) {
    loc::Localization::Options opt;
    opt.online_mode_ = true;
    loc_ = std::make_shared<loc::Localization>(opt);

    YAML_IO yaml(yaml_path);
    const auto config = YAML::LoadFile(yaml_path);
    const auto health_config = config["system"]["localization_health"];
    loc::LocalizationHealth::Limits limits;
    if (health_config) {
        limits.imu_timeout_s = health_config["imu_timeout_s"].as<double>(limits.imu_timeout_s);
        limits.lidar_timeout_s = health_config["lidar_timeout_s"].as<double>(limits.lidar_timeout_s);
        limits.result_timeout_s = health_config["result_timeout_s"].as<double>(limits.result_timeout_s);
        limits.max_result_lag_s = health_config["max_result_lag_s"].as<double>(limits.max_result_lag_s);
        limits.max_match_lag_s = health_config["max_match_lag_s"].as<double>(limits.max_match_lag_s);
        int recovery = health_config["recovery_matches"].as<int>(3);
        if (recovery < 1 || recovery > 100) return false;
        limits.recovery_matches = static_cast<unsigned>(recovery);
    }
    health_.Configure(limits);

    std::string map_path = yaml.GetValue<std::string>("system", "map_path");

    LOG(INFO) << "online mode, creating ros2 node ... ";

    /// subscribers
    node_ = std::make_shared<rclcpp::Node>("lightning_localization");
    const auto health_qos = rclcpp::QoS(1).reliable().transient_local();
    health_pub_ = node_->create_publisher<std_msgs::msg::String>("/lightning/localization_health", health_qos);
    valid_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/lightning/localization_valid", health_qos);
    // Separate callback group + two executor threads: sensor processing cannot starve the wall watchdog.
    watchdog_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    watchdog_ = node_->create_wall_timer(std::chrono::milliseconds(200),
                                        [this] { PublishHealth(); }, watchdog_group_);
    loc_->SetMatchCallback([this](double stamp, bool good) { health_.Match(stamp, good, SteadyNow()); });
    loc_->SetResultCallback([this](const loc::LocalizationResult& result) {
        health_.Result(result.timestamp_, result.valid_, SteadyNow());
        return health_.Get(SteadyNow()).valid;
    });

    imu_topic_ = yaml.GetValue<std::string>("common", "imu_topic");
    cloud_topic_ = yaml.GetValue<std::string>("common", "lidar_topic");
    livox_topic_ = yaml.GetValue<std::string>("common", "livox_lidar_topic");

    const auto imu_qos = rclcpp::SensorDataQoS().keep_last(200);
    const auto cloud_qos = rclcpp::SensorDataQoS().keep_last(5);

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
            IMUPtr imu = std::make_shared<IMU>();
            imu->timestamp = ToSec(msg->header.stamp);
            imu->linear_acceleration =
                Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            imu->angular_velocity = Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

            ProcessIMU(imu);
        });

    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
        livox_topic_, cloud_qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
            Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Proc Lidar", true);
        });

    if (options_.pub_tf_) {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        loc_->SetTFCallback(
            [this](const geometry_msgs::msg::TransformStamped &pose) {
                if (health_.Get(SteadyNow()).valid) tf_broadcaster_->sendTransform(pose);
            });
    }

    bool ret = loc_->Init(yaml_path, map_path);
    map_loaded_.store(ret);
    PublishHealth();
    if (ret) {
        LOG(INFO) << "online loc node has been created.";
    }

    return ret;
}

void LocSystem::SetInitPose(const SE3 &pose) {
    if (!map_loaded_.load() || !pose.matrix().allFinite()) return;
    LOG(INFO) << "set init pose: " << pose.translation().transpose() << ", "
              << pose.unit_quaternion().coeffs().transpose();

    loc_->SetExternalPose(pose.unit_quaternion(), pose.translation());
    loc_started_ = true;
}

void LocSystem::ProcessIMU(const IMUPtr &imu) {
    if (loc_started_ && imu && health_.Input(true, imu->timestamp, SteadyNow())) {
        loc_->ProcessIMUMsg(imu);
    }
}

void LocSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud) {
    if (loc_started_ && cloud && health_.Input(false, ToSec(cloud->header.stamp), SteadyNow())) {
        loc_->ProcessLidarMsg(cloud);
    }
}

void LocSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr &cloud) {
    if (loc_started_ && cloud && health_.Input(false, ToSec(cloud->header.stamp), SteadyNow())) {
        loc_->ProcessLivoxLidarMsg(cloud);
    }
}

void LocSystem::Spin() {
    if (node_ != nullptr) {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        executor.add_node(node_);
        executor.spin();
    }
}

double LocSystem::SteadyNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

void LocSystem::PublishHealth() {
    if (!health_pub_ || !valid_pub_) return;
    const auto s = health_.Get(SteadyNow());
    std_msgs::msg::Bool valid;
    valid.data = loc_started_.load() && s.valid;
    valid_pub_->publish(valid);
    std::ostringstream json;
    json << "{\"state\":\"" << s.state << "\",\"reason\":\"" << s.reason
         << "\",\"valid\":" << (valid.data ? "true" : "false")
         << ",\"imu_age_s\":" << s.imu_age << ",\"lidar_age_s\":" << s.lidar_age
         << ",\"result_age_s\":" << s.result_age << ",\"result_lag_s\":" << s.result_lag
         << ",\"match_lag_s\":" << s.match_lag << ",\"good_matches\":" << s.good_matches << "}";
    std_msgs::msg::String message;
    message.data = json.str();
    health_pub_->publish(message);
}

}  // namespace lightning