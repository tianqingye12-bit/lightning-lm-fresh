#include "ui/pangolin_window_impl.h"
#include <algorithm>
#include <stdexcept>

namespace lightning::ui {

PangolinWindow::PangolinWindow() { impl_ = std::make_shared<PangolinWindowImpl>(); }
PangolinWindow::~PangolinWindow() { Quit(); }

bool PangolinWindow::Init() {
    impl_->cloud_global_need_update_.store(false);
    impl_->kf_result_need_update_.store(false);
    impl_->lidarloc_need_update_.store(false);
    impl_->current_scan_need_update_.store(false);

    bool inited = impl_->Init();
    // 创建渲染线程
    if (inited) {
        impl_->StartLoopMapWorker();
        impl_->render_thread_ = std::thread([this]() { impl_->Render(); });
    }
    return inited;
}

void PangolinWindow::Reset(const std::vector<Keyframe::Ptr>& keyframes) { impl_->Reset(keyframes); }

void PangolinWindow::Quit() {
    impl_->StopLoopMapWorker();
    if (impl_->render_thread_.joinable()) {
        impl_->exit_flag_.store(true);
        impl_->render_thread_.join();
    }
    impl_->DeInit();
}

void PangolinWindow::UpdatePointCloudGlobal(const std::map<int, CloudPtr>& cloud) {
    std::lock_guard<std::mutex> lock(impl_->mtx_map_cloud_);
    impl_->cloud_global_map_ = cloud;
    impl_->cloud_global_need_update_.store(true);
}

void PangolinWindow::UpdatePointCloudDynamic(const std::map<int, CloudPtr>& cloud) {
    std::unique_lock<std::mutex> lock(impl_->mtx_map_cloud_);
    impl_->cloud_dynamic_map_.clear();  // need deep copy

    for (auto& cp : cloud) {
        CloudPtr c(new PointCloudType());
        *c = *cp.second;
        impl_->cloud_dynamic_map_.emplace(cp.first, c);
    }

    for (auto iter = impl_->cloud_dynamic_map_.begin(); iter != impl_->cloud_dynamic_map_.end();) {
        if (cloud.find(iter->first) == cloud.end()) {
            iter = impl_->cloud_dynamic_map_.erase(iter);
        } else {
            iter++;
        }
    }

    impl_->cloud_dynamic_need_update_.store(true);
}

void PangolinWindow::UpdateNavState(const NavState& state) {
    std::unique_lock<std::mutex> lock_lio_res(impl_->mtx_nav_state_);

    impl_->pose_ = state.GetPose();
    impl_->prediction_valid_ = state.pose_is_ok_;
    impl_->prediction_reason_ = state.pose_is_ok_ ? "valid" : "invalid_state";
    if (!state.pose_is_ok_) impl_->prediction_break_pending_ = true;
    impl_->last_nav_arrival_ = std::chrono::steady_clock::now();
    impl_->vel_ = state.GetVel();
    impl_->bias_acc_ = state.Getba();
    impl_->bias_gyr_ = state.Getbg();
    impl_->confidence_ = state.confidence_;

    impl_->kf_result_need_update_.store(true);
}

void PangolinWindow::UpdateRecentPose(const SE3& pose) {
    std::lock_guard<std::mutex> lock(impl_->mtx_nav_state_);
    impl_->pose_ = pose;
    impl_->prediction_valid_ = true;
    impl_->prediction_reason_ = "pose_update";
    impl_->last_nav_arrival_ = std::chrono::steady_clock::now();
    impl_->kf_result_need_update_.store(true);
}

void PangolinWindow::UpdatePredictPose(const SE3& pose) {
    UL lock(impl_->mtx_nav_state_);
    impl_->predicted_pose_ = pose;
}

void PangolinWindow::ConfigureRuntimeDisplay(const RuntimeDisplayOptions& options, RuntimeStats* stats) {
    if (!options.Valid()) throw std::invalid_argument("invalid runtime display options");
    impl_->runtime_display_options_ = options;
    impl_->max_size_of_current_scan_ = static_cast<int>(options.history_scans);
    impl_->runtime_stats_ = stats;
}

void PangolinWindow::UpdatePredictionStatus(bool valid, const std::string& reason, double lead_s) {
    std::lock_guard<std::mutex> lock(impl_->mtx_nav_state_);
    impl_->prediction_valid_ = valid;
    impl_->prediction_reason_ = reason;
    impl_->prediction_lead_s_ = lead_s;
    // Latch even if pause and recovery both arrive between two render frames.
    if (!valid) impl_->prediction_break_pending_ = true;
}

void PangolinWindow::UpdateRuntimeStatus(const std::string& text) {
    std::lock_guard<std::mutex> lock(impl_->mtx_nav_state_);
    impl_->runtime_status_ = text;
}

void PangolinWindow::UpdateScan(CloudPtr cloud, const SE3& pose, const SE3& cloud_to_body) {
    std::lock_guard<std::mutex> lock(impl_->mtx_current_scan_);
    if (!cloud || cloud->empty()) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - impl_->last_scan_submit_ < std::chrono::duration<double>(1.0 / impl_->runtime_display_options_.scan_hz))
        return;
    impl_->last_scan_submit_ = now;
    CloudPtr display(new PointCloudType());
    const size_t n = std::min(cloud->size(), impl_->runtime_display_options_.scan_max_points);
    display->reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const size_t index = n > 1 ? i * (cloud->size() - 1) / (n - 1) : 0;
        auto point = cloud->points[index];
        const Vec3d xyz = cloud_to_body * point.getVector3fMap().cast<double>();
        point.getVector3fMap() = xyz.cast<float>();
        display->push_back(point);
    }
    impl_->current_scan_ = std::move(display);  // immutable snapshot; input never retained or modified
    impl_->current_scan_pose_ = pose;
    impl_->current_scan_need_update_.store(true);
}

void PangolinWindow::UpdateKF(std::shared_ptr<Keyframe> kf) {
    UL lock(impl_->mtx_current_scan_);
    impl_->all_keyframes_.emplace_back(kf);
}

void PangolinWindow::ConfigureLoopDisplay(const LoopDisplayOptions& options) {
    impl_->loop_display_options_ = options;
}

void PangolinWindow::UpdateOptimizedMap(const std::vector<Keyframe::Ptr>& keyframes) {
    if (!impl_->loop_display_options_.enabled || keyframes.empty()) return;
    PangolinWindowImpl::LoopSnapshot snapshot;
    snapshot.frames.reserve(keyframes.size());
    const auto poses = Keyframe::SnapshotPoses(keyframes);
    for (size_t i = 0; i < keyframes.size(); ++i) {
        snapshot.frames.push_back({keyframes[i]->GetCloud(), poses[i]});
    }
    const auto& last = keyframes.back();
    snapshot.map_from_odom = snapshot.frames.back().pose * last->GetLIOPose().inverse();
    impl_->loop_map_tasks_.Put(std::move(snapshot));
}

void PangolinWindow::SetCurrentScanSize(int current_scan_size) { impl_->max_size_of_current_scan_ = current_scan_size; }

void PangolinWindow::SetTImuLidar(const SE3& T_imu_lidar) { impl_->T_imu_lidar_ = T_imu_lidar; }

bool PangolinWindow::ShouldQuit() { return pangolin::ShouldQuit(); }

}  // namespace lightning::ui
