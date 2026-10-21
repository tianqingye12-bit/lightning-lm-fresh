//
// Created by xiang on 25-5-6.
//

#include "core/system/slam.h"
#include "core/g2p5/g2p5.h"
#include "core/lio/laser_mapping.h"
#include "core/loop_closing/loop_closing.h"
#include "core/maps/tiled_map.h"
#include "core/maps/navigation_grid.h"
#include "core/maps/map_transaction.h"
#include "ui/pangolin_window.h"
#include "wrapper/ros_utils.h"

#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <stdexcept>
#include <opencv2/opencv.hpp>

namespace lightning {

SlamSystem::SlamSystem(lightning::SlamSystem::Options options) : options_(options) {
    /// handle ctrl-c
    signal(SIGINT, lightning::debug::SigHandle);
}

bool SlamSystem::Init(const std::string& yaml_path) {
    lio_ = std::make_shared<LaserMapping>();
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio module";
        return false;
    }

    auto yaml = YAML::LoadFile(yaml_path);
    export_keyframe_clouds_ = yaml["system"]["export_keyframe_clouds"].as<bool>(false);
    const auto map_export = yaml["system"]["map_export"];
    if (map_export && !map_export.IsMap()) return false;
    if (map_export) {
        save_global_pcd_ = map_export["save_global_pcd"].as<bool>(true);
        save_localization_tiles_ = map_export["save_localization_tiles"].as<bool>(true);
        compress_keyframes_ = map_export["compress_keyframes"].as<bool>(true);
    }
    if (!save_global_pcd_ && !save_localization_tiles_ && !export_keyframe_clouds_) {
        LOG(ERROR) << "map export must retain global PCD, tiles or keyframes";
        return false;
    }
    localization_export_resolution_ = map_export ? map_export["localization_resolution"].as<float>(0.1f) : 0.1f;
    if (!std::isfinite(localization_export_resolution_) || localization_export_resolution_ <= 0) return false;
    options_.with_loop_closing_ = yaml["system"]["with_loop_closing"].as<bool>();
    options_.with_visualization_ = yaml["system"]["with_ui"].as<bool>();
    options_.with_2dvisualization_ = yaml["system"]["with_2dui"].as<bool>();
    options_.with_gridmap_ = yaml["system"]["with_g2p5"].as<bool>();
    options_.step_on_kf_ = yaml["system"]["step_on_kf"].as<bool>();

    if (options_.with_loop_closing_) {
        LOG(INFO) << "slam with loop closing";
        LoopClosing::Options options;
        options.online_mode_ = options_.online_mode_;
        lc_ = std::make_shared<LoopClosing>(options);
        lc_->Init(yaml_path);
    }

    if (options_.with_visualization_) {
        LOG(INFO) << "slam with 3D UI";
        ui_ = std::make_shared<ui::PangolinWindow>();
        ui::LoopDisplayOptions display;
        const auto settings = yaml["system"]["loop_display"];
        if (settings && !settings.IsMap()) {
            LOG(ERROR) << "system.loop_display must be a map";
            return false;
        }
        if (settings) {
            display.enabled = settings["enabled"].as<bool>(display.enabled);
            display.min_interval = settings["min_interval"].as<double>(display.min_interval);
            const auto max_points = settings["max_points"].as<long long>(display.max_points);
            if (max_points <= 0) return false;
            display.max_points = static_cast<size_t>(max_points);
        }
        if (!display.Valid()) return false;
        ui::RuntimeDisplayOptions render_options;
        auto render = yaml["system"]["runtime_display"];
        if (!render) render = YAML::Node(YAML::NodeType::Map);
        if (!render.IsMap()) return false;
        render_options.render_hz = render["render_hz"].as<double>(20);
        render_options.scan_hz = render["scan_hz"].as<double>(5);
        render_options.stale_wall_s = render["stale_wall_s"].as<double>(1);
        const auto read_budget = [&](const char* key, size_t& budget) {
            const auto value = render[key].as<long long>(static_cast<long long>(budget));
            if (value <= 0 || value > 100000000) return false;
            budget = static_cast<size_t>(value); return true;
        };
        if (!read_budget("scan_max_points", render_options.scan_max_points) ||
            !read_budget("history_max_points", render_options.history_max_points) ||
            !read_budget("history_scans", render_options.history_scans) ||
            !read_budget("trajectory_max_points", render_options.trajectory_max_points) ||
            !render_options.Valid()) return false;
        ui_->ConfigureRuntimeDisplay(render_options, &lio_->Stats());
        ui_->ConfigureLoopDisplay(display);
        if (!ui_->Init()) return false;

        lio_->SetUI(ui_);
    }

    if (options_.with_gridmap_) {
        g2p5::G2P5::Options opt;
        opt.online_mode_ = options_.online_mode_;

        g2p5_ = std::make_shared<g2p5::G2P5>(opt);
        g2p5_->Init(yaml_path);

        if (options_.with_2dvisualization_) {
            g2p5_->SetMapUpdateCallback([this](g2p5::G2P5MapPtr map) {
                cv::Mat image = map->ToCV();
                cv::imshow("map", image);

                if (options_.step_on_kf_) {
                    cv::waitKey(0);

                } else {
                    cv::waitKey(10);
                }
            });
        }
    }

    if (lc_) {
        // One callback serves both displays, including configurations with no 2D grid map.
        lc_->SetLoopClosedCB([this](const std::vector<Keyframe::Ptr>& keyframes) {
            if (g2p5_) g2p5_->RedrawGlobalMap();
            if (ui_) ui_->UpdateOptimizedMap(keyframes);
        });
    }

    if (options_.online_mode_) {
        auto runtime = yaml["system"]["runtime"];
        if (!runtime) runtime = YAML::Node(YAML::NodeType::Map);
        if (runtime && !runtime.IsMap()) return false;
        runtime_stats_interval_ = runtime["stats_interval"].as<int>(20);
        const int imu_capacity = runtime["imu_capacity"].as<int>(4096);
        const int lidar_capacity = runtime["lidar_capacity"].as<int>(2);
        if (runtime_stats_interval_ < 0 || imu_capacity < 10 || imu_capacity > 100000 ||
            lidar_capacity < 1 || lidar_capacity > 100) return false;
        lio_->ConfigureInputLimits(static_cast<size_t>(imu_capacity),
                                   std::max<size_t>(4, static_cast<size_t>(lidar_capacity) + 2));
        if (runtime["async"].as<bool>(true)) {
            online_worker_.reset(new SerialWorker({static_cast<size_t>(imu_capacity),
                                                   static_cast<size_t>(lidar_capacity), 8}));
            online_worker_->Start();
        }
        LOG(INFO) << "online mode, creating ros2 node ... ";

        /// subscribers
        node_ = std::make_shared<rclcpp::Node>("lightning_slam");

        imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
        cloud_topic_ = yaml["common"]["lidar_topic"].as<std::string>();
        livox_topic_ = yaml["common"]["livox_lidar_topic"].as<std::string>();

        rclcpp::QoS qos(10);
        // A 200 Hz IMU history of 10 messages covers only 50 ms while this same executor maps a scan.
        const int imu_queue_depth = yaml["common"]["imu_queue_depth"].as<int>(2000);
        if (imu_queue_depth < 10 || imu_queue_depth > 100000) {
            LOG(ERROR) << "common.imu_queue_depth must be between 10 and 100000";
            return false;
        }
        rclcpp::QoS imu_qos{rclcpp::KeepLast(static_cast<size_t>(imu_queue_depth))};
        LOG(INFO) << "[imu_clock] IMU subscription history=" << imu_queue_depth;

        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
                IMUPtr imu = std::make_shared<IMU>();
                imu->timestamp = ToSec(msg->header.stamp);
                imu->linear_acceleration =
                    Vec3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
                imu->angular_velocity =
                    Vec3d(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

                ProcessIMU(imu);
            });

        cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos, [this](sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Receive Lidar", false);
            });

        livox_sub_ = node_->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            livox_topic_, qos, [this](livox_ros_driver2::msg::CustomMsg ::SharedPtr cloud) {
                Timer::Evaluate([&]() { ProcessLidar(cloud); }, "Receive Lidar", false);
            });

        save_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        savemap_service_ = node_->create_service<SaveMapService>(
            "lightning/save_map", [this](const SaveMapService::Request::SharedPtr& req,
                                         SaveMapService::Response::SharedPtr res) { SaveMap(req, res); },
            rmw_qos_profile_services_default, save_callback_group_);

        LOG(INFO) << "online slam node has been created.";
    }

    return true;
}

SlamSystem::~SlamSystem() {
    if (online_worker_) online_worker_->Stop();  // Drain core work before backend/UI teardown.
    lc_.reset();  // Join the backend callback thread before destroying either display.
    if (ui_) {
        ui_->Quit();
    }
}

void SlamSystem::StartSLAM(std::string map_name) {
    if (online_worker_ && !online_worker_->IsCurrent()) {
        online_worker_->Call([this, map_name] { StartSLAM(map_name); });
        return;
    }
    if (input_fault_.load()) {
        LOG(ERROR) << "[runtime] start rejected after input fault; restart the SLAM node";
        return;
    }
    map_name_ = map_name;
    running_ = true;
}

void SlamSystem::SaveMap(const SaveMapService::Request::SharedPtr request,
                         SaveMapService::Response::SharedPtr response) {
    if (online_worker_ && !online_worker_->IsCurrent()) {
        try { online_worker_->Call([this, request, response] { SaveMap(request, response); }); }
        catch (const std::exception& e) { LOG(ERROR) << e.what(); response->response = 3; }
        return;
    }
    if (!lio_ || lio_->GetAllKeyframes().empty()) {
        LOG(WARNING) << "save map rejected: no keyframes yet";
        response->response = 3;
        return;
    }
    if (!maps::ValidMapId(request->map_id)) {
        LOG(ERROR) << "save map rejected: map_id must be a non-hidden directory basename";
        response->response = 3;
        return;
    }
    try {
        SaveMap("./data/" + request->map_id);
        map_name_ = request->map_id;
        response->response = 0;
    } catch (const std::exception& e) {
        LOG(ERROR) << "save map failed; previous map preserved: " << e.what();
        response->response = 3;  // Existing wire contract: save failure.
    }
}

void SlamSystem::SaveMap(const std::string& path) {
    if (online_worker_ && !online_worker_->IsCurrent()) {
        online_worker_->Call([this, path] { SaveMap(path); });
        return;
    }
    // Validate before accessing front() or replacing an existing map directory.
    if (!lio_) {
        throw std::runtime_error("LIO is not initialized");
    }
    RuntimeStats::Scope save_time(lio_->Stats(), RuntimeStats::save_map);
    const auto keyframes = lio_->GetAllKeyframes();
    if (keyframes.empty()) {
        throw std::runtime_error("no keyframes yet");
    }
    std::vector<SE3> exported_poses;
    CloudPtr global_map;
    if (save_global_pcd_ || save_localization_tiles_) {
        global_map = lio_->GetGlobalMap(!options_.with_loop_closing_, true, -1.0f, &exported_poses);
        if (!global_map || global_map->empty()) throw std::runtime_error("empty global map");
    } else {
        exported_poses = Keyframe::SnapshotPoses(keyframes, !options_.with_loop_closing_);
    }
    if (exported_poses.size() != keyframes.size()) throw std::runtime_error("keyframe snapshot changed");
    std::string save_path = path;
    if (save_path.empty()) {
        save_path = "./data/" + map_name_ + "/";
    }

    const std::string destination = save_path;
    maps::MapTransaction transaction(destination);
    save_path = transaction.Staging().string();
    LOG(INFO) << "staging map for " << destination << " in " << save_path;

    // auto global_map_no_loop = lio_->GetGlobalMap(true);
    // auto global_map_raw = lio_->GetGlobalMap(!options_.with_loop_closing_, false, 0.1);

    if (save_localization_tiles_) {
        TiledMap::Options tm_options;
        tm_options.map_path_ = save_path;
        tm_options.voxel_size_in_chunk_ = localization_export_resolution_;
        tm_options.preserve_observed_points_ = lio_->PreserveObservedPoints();
        TiledMap tm(tm_options);
        tm.ConvertFromFullPCD(global_map, exported_poses.front(), save_path);
    }
    if (save_global_pcd_ && pcl::io::savePCDFileBinaryCompressed(save_path + "/global.pcd", *global_map) < 0)
        throw std::runtime_error("failed to save global map");

    /// 保存关键帧时间戳和位姿（与 global.pcd 使用同一套位姿），供离线去除车辆等后处理使用
    /// 每行: id timestamp tx ty tz qx qy qz qw
    std::ofstream kf_file(save_path + "/keyframes.txt");
    kf_file.exceptions(std::ios::badbit | std::ios::failbit);
    kf_file << std::setprecision(18);
    // Publish the manifest only after all frame files have been saved successfully.
    std::ofstream frame_index;
    if (export_keyframe_clouds_) {
        std::filesystem::create_directories(save_path + "/keyframe_clouds");
        frame_index.open(save_path + "/keyframe_clouds/index.csv.tmp");
        frame_index.exceptions(std::ios::badbit | std::ios::failbit);
        frame_index << "id,timestamp,file\n" << std::setprecision(18);
        if (!frame_index) throw std::runtime_error("cannot create keyframe cloud manifest");
    }

    for (size_t i = 0; i < keyframes.size(); ++i) {
        const auto& kf = keyframes[i];
        if (export_keyframe_clouds_) {
            const auto cloud = kf->GetCloud();
            const auto filename = std::to_string(kf->GetID()) + ".pcd";
            const auto file_path = save_path + "/keyframe_clouds/" + filename;
            if (!cloud || (compress_keyframes_ ? pcl::io::savePCDFileBinaryCompressed(file_path, *cloud)
                                             : pcl::io::savePCDFileBinary(file_path, *cloud)) < 0)
                throw std::runtime_error("failed to export keyframe " + filename);
            frame_index << kf->GetID() << "," << kf->GetState().timestamp_ << "," << filename << "\n";
        }
        const SE3& pose = exported_poses[i];
        const auto& t = pose.translation();
        const auto q = pose.unit_quaternion();
        kf_file << kf->GetID() << " " << kf->GetState().timestamp_ << " " << t.x() << " " << t.y() << " " << t.z()
                << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
    }
    kf_file.flush();
    kf_file.close();
    if (export_keyframe_clouds_) {
        frame_index.flush();
        if (!frame_index) throw std::runtime_error("failed to write keyframe cloud manifest");
        frame_index.close();
        std::filesystem::rename(save_path + "/keyframe_clouds/index.csv.tmp",
                                save_path + "/keyframe_clouds/index.csv");
        LOG(INFO) << "exported " << keyframes.size() << " IMU-frame keyframe clouds for offline cleaning";
    }
    // Version the coordinate contract; cleaning interpolates IMU poses first, then applies T_imu_lidar.
    std::ofstream frame_file(save_path + "/map_frames.yaml");
    frame_file.exceptions(std::ios::badbit | std::ios::failbit);
    frame_file << std::setprecision(18)
               << "format_version: 2\nkeyframe_pose_frame: imu\nkeyframe_cloud_frame: imu\nT_imu_lidar: [";
    const auto T_imu_lidar = lio_->GetTImuLidar().matrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (row || col) frame_file << ", ";
            frame_file << T_imu_lidar(row, col);
        }
    }
    frame_file << "]\n";
    frame_file.flush();
    frame_file.close();
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_no_loop.pcd", *global_map_no_loop);
    // pcl::io::savePCDFileBinaryCompressed(save_path + "/global_raw.pcd", *global_map_raw);

    if (options_.with_gridmap_) {
        /// 存为ROS兼容的模式
        const auto newest = g2p5_->GetNewestMap();
        if (!newest) throw std::runtime_error("no navigation grid available for export");
        auto map = newest->ToROS();
        navigation::GridSpec spec;
        spec.width = map.info.width;
        spec.height = map.info.height;
        spec.resolution = map.info.resolution;
        const auto& q = map.info.origin.orientation;
        const double yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
        spec.origin = {map.info.origin.position.x, map.info.origin.position.y, yaw};
        navigation::SaveGrid(save_path, spec, map.data);
    }

    std::ofstream complete(save_path + "/save_complete.yaml");
    complete.exceptions(std::ios::badbit | std::ios::failbit);
    complete << "format_version: 1\nkeyframes: " << keyframes.size() << "\n";
    complete.flush();
    complete.close();
    if (!transaction.Commit()) LOG(WARNING) << "map published but parent fsync failed; durability is unconfirmed";
    LOG(INFO) << "map saved to " << destination;
    if (!transaction.Previous().empty()) LOG(INFO) << "previous map retained at " << transaction.Previous();
}

void SlamSystem::Enqueue(SerialWorker::Kind kind, std::function<void()> task) {
    if (!running_.load()) return;
    if (!online_worker_->Post(kind, [this, kind, task = std::move(task)](double wait_ms) {
            lio_->Stats().Observe(kind == SerialWorker::imu ? RuntimeStats::imu_wait : RuntimeStats::lidar_wait, wait_ms);
            try {
                task();
                if (lio_->GetInputStats().fault) {
                    running_.store(false); input_fault_.store(true);
                    if (ui_) ui_->UpdatePredictionStatus(false, "core_input_fault", 0);
                }
            }
            catch (const std::exception& e) {
                running_.store(false); input_fault_.store(true);
                LOG(ERROR) << "[runtime] core task failed: " << e.what();
                if (ui_) ui_->UpdatePredictionStatus(false, "core_fault", 0);
            } catch (...) {
                running_.store(false); input_fault_.store(true);
                LOG(ERROR) << "[runtime] unknown core task failure";
                if (ui_) ui_->UpdatePredictionStatus(false, "core_fault", 0);
            }
            if (kind == SerialWorker::lidar) ReportRuntime();
        })) {
        const auto state = online_worker_->State();
        if (kind == SerialWorker::imu) {
            running_.store(false); input_fault_.store(true);
            LOG(ERROR) << "[runtime] IMU queue overflow: stop core ingestion; save map and restart slower";
            if (ui_) ui_->UpdatePredictionStatus(false, "input_overflow", 0);
        } else if (state.rejected[kind] == 1 || state.rejected[kind] % 20 == 0) {
            LOG(WARNING) << "[runtime] drop newest queued LiDAR frame count=" << state.rejected[kind];
        }
    }
}

void SlamSystem::ReportRuntime() {
    ++runtime_frames_;
    const auto state = online_worker_ ? online_worker_->State() : SerialWorker::Snapshot{};
    const auto core_input = lio_->GetInputStats();
    std::ostringstream status;
    status << (input_fault_.load() ? "INPUT FAULT" : "worker active")
           << " queue imu/lidar=" << state.queued[0] << '/' << state.queued[1]
           << " core queue=" << core_input.imu << '/' << core_input.lidar
           << " rejected imu/lidar=" << state.rejected[0] << '/' << state.rejected[1]
           << " core_lidar_dropped=" << core_input.dropped_lidar;
    if (ui_) ui_->UpdateRuntimeStatus(status.str());
    if (runtime_stats_interval_ > 0 && runtime_frames_ % runtime_stats_interval_ == 0) {
        LOG(INFO) << "[runtime] " << status.str() << lio_->Stats().Summary();
    }
}

void SlamSystem::ProcessIMU(const lightning::IMUPtr& imu) {
    if (online_worker_ && !online_worker_->IsCurrent()) {
        Enqueue(SerialWorker::imu, [this, imu] { ProcessIMU(imu); });
        return;
    }
    if (running_ == false) {
        return;
    }
    RuntimeStats::Scope imu_time(lio_->Stats(), RuntimeStats::imu_update);
    lio_->ProcessIMU(imu);
}

void SlamSystem::ProcessLidar(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    if (online_worker_ && !online_worker_->IsCurrent()) {
        Enqueue(SerialWorker::lidar, [this, cloud] { ProcessLidar(cloud); });
        return;
    }
    if (running_ == false) {
        return;
    }
    RuntimeStats::Scope frame_time(lio_->Stats(), RuntimeStats::frame_total);

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::ProcessLidar(const livox_ros_driver2::msg::CustomMsg::SharedPtr& cloud) {
    if (online_worker_ && !online_worker_->IsCurrent()) {
        Enqueue(SerialWorker::lidar, [this, cloud] { ProcessLidar(cloud); });
        return;
    }
    if (running_ == false) {
        return;
    }
    RuntimeStats::Scope frame_time(lio_->Stats(), RuntimeStats::frame_total);

    lio_->ProcessPointCloud2(cloud);
    lio_->Run();

    auto kf = lio_->GetKeyframe();
    if (kf != cur_kf_) {
        cur_kf_ = kf;
    } else {
        return;
    }

    if (cur_kf_ == nullptr) {
        return;
    }

    if (options_.with_loop_closing_) {
        lc_->AddKF(cur_kf_);
    }

    if (options_.with_gridmap_) {
        g2p5_->PushKeyframe(cur_kf_);
    }

    if (ui_) {
        ui_->UpdateKF(cur_kf_);
    }
}

void SlamSystem::Spin() {
    if (options_.online_mode_ && node_ != nullptr) {
        // Sensors share the node's mutually-exclusive group; the save service has its own.
        // Both only submit to ONE core worker, so ESKF/map access remains serialized.
        if (online_worker_) {
            rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
            executor.add_node(node_);
            executor.spin();
        } else {
            rclcpp::spin(node_);  // synchronous fallback retains one callback thread
        }
    }
}

}  // namespace lightning
