//
// Created by xiang on 25-4-21.
//

#include "core/loop_closing/loop_closing.h"
#include "common/keyframe.h"
#include "common/loop_candidate.h"
#include "utils/pointcloud_utils.h"

#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/point_tests.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <stdexcept>
#include "core/lio/map_detail.h"

#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/cauchy.h"
#include "core/types/edge_se3.h"
#include "core/types/edge_se3_height_prior.h"
#include "core/types/vertex_se3.h"
#include "io/yaml_io.h"

namespace lightning {

LoopClosing::~LoopClosing() {
    if (options_.online_mode_) {
        kf_thread_.Quit();
    }
}

void LoopClosing::Init(const std::string yaml_path) {
    /// setup miao
    miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                                 miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    // Rebuild the active set on loop events: level changes must exclude old outliers too.
    // No graph solve or rebuild is performed for ordinary non-loop keyframes.
    config.incremental_mode_ = false;
    optimizer_ = miao::SetupOptimizer<6, 3>(config);

    info_motion_.setIdentity();
    info_motion_.block<3, 3>(0, 0) =
        Mat3d::Identity() * 1.0 / (options_.motion_trans_noise_ * options_.motion_trans_noise_);
    info_motion_.block<3, 3>(3, 3) =
        Mat3d::Identity() * 1.0 / (options_.motion_rot_noise_ * options_.motion_rot_noise_);

    info_loops_.setIdentity();
    info_loops_.block<3, 3>(0, 0) = Mat3d::Identity() * 1.0 / (options_.loop_trans_noise_ * options_.loop_trans_noise_);
    info_loops_.block<3, 3>(3, 3) = Mat3d::Identity() * 1.0 / (options_.loop_rot_noise_ * options_.loop_rot_noise_);

    if (!yaml_path.empty()) {
        YAML_IO yaml(yaml_path);

        options_.loop_kf_gap_ = yaml.GetValue<int>("loop_closing", "loop_kf_gap");
        options_.min_id_interval_ = yaml.GetValue<int>("loop_closing", "min_id_interval");
        options_.closest_id_th_ = yaml.GetValue<int>("loop_closing", "closest_id_th");
        options_.max_range_ = yaml.GetValue<double>("loop_closing", "max_range");
        options_.ndt_score_th_ = yaml.GetValue<double>("loop_closing", "ndt_score_th");
        options_.with_height_ = yaml.GetValue<bool>("loop_closing", "with_height");
        const auto validation = YAML::LoadFile(yaml_path)["loop_closing"]["validation"];
        if (validation && !validation.IsMap()) throw std::invalid_argument("loop_closing.validation must be a map");
        if (validation) {
            auto& q = options_.quality_;
            q.overlap_distance = validation["overlap_distance"].as<double>(q.overlap_distance);
            q.min_overlap = validation["min_overlap"].as<double>(q.min_overlap);
            q.max_inlier_rmse = validation["max_inlier_rmse"].as<double>(q.max_inlier_rmse);
            q.max_translation = validation["max_translation"].as<double>(q.max_translation);
            q.max_rotation_deg = validation["max_rotation_deg"].as<double>(q.max_rotation_deg);
            q.max_samples = validation["max_samples"].as<int>(q.max_samples);
            q.min_inliers = validation["min_inliers"].as<int>(q.min_inliers);
            q.max_candidates = validation["max_candidates"].as<int>(q.max_candidates);
            q.max_loop_chi2 = validation["max_loop_chi2"].as<double>(q.max_loop_chi2);
            q.refine_iterations = validation["refine_iterations"].as<int>(q.refine_iterations);
        }
    }

    if (!options_.quality_.Valid() || !std::isfinite(options_.ndt_score_th_) || options_.ndt_score_th_ < 0) {
        throw std::invalid_argument("invalid loop validation settings");
    }
    LOG(INFO) << "[loop_validation] overlap=" << options_.quality_.min_overlap
              << " rmse_m=" << options_.quality_.max_inlier_rmse
              << " max_candidates=" << options_.quality_.max_candidates;

    if (options_.online_mode_) {
        LOG(INFO) << "loop closing module is running in online mode";
        kf_thread_.SetProcFunc([this](Keyframe::Ptr kf) { HandleKF(kf); });
        kf_thread_.SetName("handle loop closure");
        kf_thread_.Start();
    }
}

void LoopClosing::AddKF(Keyframe::Ptr kf) {
    if (options_.online_mode_) {
        kf_thread_.AddMessage(kf);
    } else {
        HandleKF(kf);
    }
}

void LoopClosing::HandleKF(Keyframe::Ptr kf) {
    if (!kf || kf == last_kf_) return;
    if (kf->GetID() != all_keyframes_.size()) {
        LOG(WARNING) << "loop backend requires ordered contiguous keyframes; skip id=" << kf->GetID();
        return;
    }

    cur_kf_ = kf;
    all_keyframes_.emplace_back(kf);

    // 检测回环候选
    DetectLoopCandidates();

    if (options_.verbose_) {
        LOG(INFO) << "lc: get kf " << cur_kf_->GetID() << " candi: " << candidates_.size();
    }

    // 计算回环位姿
    ComputeLoopCandidates();

    // 位姿图优化
    PoseOptimization();

    last_kf_ = kf;
}

void LoopClosing::DetectLoopCandidates() {
    candidates_.clear();

    auto& kfs_mapping = all_keyframes_;
    Keyframe::Ptr check_first = nullptr;

    if (last_loop_kf_ == nullptr) {
        last_loop_kf_ = cur_kf_;
        return;
    }

    if (last_loop_kf_ && (cur_kf_->GetID() - last_loop_kf_->GetID()) <= options_.loop_kf_gap_) {
        LOG(INFO) << "skip because last loop kf: " << last_loop_kf_->GetID();
        return;
    }

    for (auto kf : kfs_mapping) {
        if (check_first != nullptr && abs(int(kf->GetID() - check_first->GetID())) <= options_.min_id_interval_) {
            // 同条轨迹内，跳过一定的ID区间
            continue;
        }

        if (abs(int(kf->GetID() - cur_kf_->GetID())) < options_.closest_id_th_) {
            /// 在同一条轨迹中，如果间隔太近，就不考虑回环
            break;
        }

        Vec3d dt = kf->GetOptPose().translation() - cur_kf_->GetOptPose().translation();
        double t2d = dt.head<2>().norm();  // x-y distance
        double range_th = options_.max_range_;

        if (t2d < range_th) {
            LoopCandidate c(kf->GetID(), cur_kf_->GetID());
            c.Tij_ = kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();

            candidates_.emplace_back(c);
            check_first = kf;
        }
    }

    // Bound expensive multi-resolution NDT attempts, checking the closest candidates first.
    std::stable_sort(candidates_.begin(), candidates_.end(), [this](const auto& a, const auto& b) {
        const Vec3d position = cur_kf_->GetOptPose().translation();
        return (all_keyframes_[a.idx1_]->GetOptPose().translation() - position).squaredNorm() <
               (all_keyframes_[b.idx1_]->GetOptPose().translation() - position).squaredNorm();
    });
    if (candidates_.size() > static_cast<size_t>(options_.quality_.max_candidates)) {
        candidates_.resize(options_.quality_.max_candidates);
    }
    if (!candidates_.empty()) {
        last_loop_kf_ = cur_kf_;
    }

    if (options_.verbose_ && !candidates_.empty()) {
        LOG(INFO) << "lc candi: " << candidates_.size();
    }
}

void LoopClosing::ComputeLoopCandidates() {
    if (candidates_.empty()) {
        return;
    }

    // 执行计算
    std::for_each(candidates_.begin(), candidates_.end(), [this](LoopCandidate& c) { ComputeForCandidate(c); });
    // 保存成功的候选
    std::vector<LoopCandidate> succ_candidates;
    for (const auto& lc : candidates_) {
        // LOG(INFO) << "candi " << lc.idx1_ << ", " << lc.idx2_ << " s: " << lc.ndt_score_;
        if (lc.verified_) {
            succ_candidates.emplace_back(lc);
        }
    }

    if (options_.verbose_) {
        LOG(INFO) << "success: " << succ_candidates.size() << "/" << candidates_.size();
    }

    candidates_.swap(succ_candidates);
}

void LoopClosing::ComputeForCandidate(lightning::LoopCandidate& c) {
    c.verified_ = false;
    c.quality_ = loop_quality::MatchMetrics{};
    const int submap_idx_range = 40;
    auto kf1 = all_keyframes_.at(c.idx1_), kf2 = all_keyframes_.at(c.idx2_);

    auto build_submap = [this](int given_id, bool build_in_world) -> CloudPtr {
        CloudPtr submap(new PointCloudType);
        for (int idx = -submap_idx_range; idx < submap_idx_range; idx += 4) {
            int id = idx + given_id;
            if (id < 0 || id >= all_keyframes_.size()) {
                continue;
            }
            // Never validate against the source scan or its temporally adjacent neighborhood.
            if (std::abs(id - static_cast<int>(cur_kf_->GetID())) < options_.closest_id_th_) continue;

            auto kf = all_keyframes_[id];
            CloudPtr cloud = kf->GetCloud();

            // RemoveGround(cloud, 0.1);

            if (cloud->empty()) {
                continue;
            }

            // 转到世界系下
            SE3 Twb = kf->GetOptPose();

            if (!build_in_world) {
                Twb = all_keyframes_.at(given_id)->GetOptPose().inverse() * Twb;
            }

            CloudPtr cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*cloud, *cloud_trans, Twb.matrix());

            *submap += *cloud_trans;
        }
        return submap;
    };

    auto finite_cloud = [](const CloudPtr& input) {
        CloudPtr output(new PointCloudType());
        output->reserve(input->size());
        for (const auto& p : input->points) if (pcl::isFinite(p)) output->push_back(p);
        return output;
    };
    auto submap_kf1 = finite_cloud(build_submap(kf1->GetID(), true));
    CloudPtr submap_kf2 = finite_cloud(kf2->GetCloud());

    if (submap_kf1->empty() || submap_kf2->empty()) {
        c.ndt_score_ = 0;
        return;
    }

    const SE3 predicted = kf2->GetOptPose();
    if (!predicted.matrix().allFinite()) return;
    Mat4f Tw2 = predicted.matrix().cast<float>();

    /// 不同分辨率下的匹配
    CloudPtr output(new PointCloudType);
    std::vector<double> res{10.0, 5.0, 2.0, 1.0};

    CloudPtr rough_map1, rough_map2;

    for (auto& r : res) {
        pcl::NormalDistributionsTransform<PointType, PointType> ndt;
        ndt.setTransformationEpsilon(0.05);
        ndt.setStepSize(0.7);
        ndt.setMaximumIterations(40);

        ndt.setResolution(r);
        rough_map1 = VoxelGrid(submap_kf1, r * 0.1);
        rough_map2 = VoxelGrid(submap_kf2, r * 0.1);
        if (rough_map1->size() < 6 || rough_map2->size() < 6) return;
        ndt.setInputTarget(rough_map1);
        ndt.setInputSource(rough_map2);

        ndt.align(*output, Tw2);
        Tw2 = ndt.getFinalTransformation();

        c.ndt_score_ = ndt.getTransformationProbability();
        c.quality_.converged = ndt.hasConverged();
        if (!Tw2.allFinite()) return;
    }

    const Mat3f rotation = Tw2.block<3, 3>(0, 0);
    c.quality_.finite_transform =
        (rotation.transpose() * rotation - Mat3f::Identity()).norm() < 1e-3 &&
        std::abs(rotation.determinant() - 1.0f) < 1e-3;
    if (!c.quality_.finite_transform) return;
    Mat4d T = Tw2.cast<double>();
    Quatd q(T.block<3, 3>(0, 0));
    q.normalize();
    Vec3d t = T.block<3, 1>(0, 3);

    const SE3 matched(q, t);
    const SE3 correction = predicted.inverse() * matched;
    auto& metrics = c.quality_;
    metrics.probability = c.ndt_score_;
    metrics.correction_translation = correction.translation().norm();
    metrics.correction_rotation_deg = correction.so3().log().norm() * 180.0 / M_PI;

    pcl::KdTreeFLANN<PointType> tree;
    tree.setInputCloud(rough_map1);
    metrics.samples = std::min(rough_map2->size(), static_cast<size_t>(options_.quality_.max_samples));
    double squared_error = 0.0;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    const double max_squared_distance = options_.quality_.overlap_distance * options_.quality_.overlap_distance;
    for (size_t i = 0; i < metrics.samples; ++i) {
        const auto& source = rough_map2->points[mapping::BudgetIndex(i, rough_map2->size(), metrics.samples)];
        const Vec3f transformed = rotation * source.getVector3fMap() + Tw2.block<3, 1>(0, 3);
        PointType query;
        query.x = transformed.x(); query.y = transformed.y(); query.z = transformed.z();
        if (pcl::isFinite(query) && tree.nearestKSearch(query, 1, indices, distances) == 1 &&
            std::isfinite(distances[0]) && distances[0] <= max_squared_distance) {
            ++metrics.inliers;
            squared_error += distances[0];
        }
    }
    if (metrics.inliers > 0) metrics.inlier_rmse = std::sqrt(squared_error / metrics.inliers);
    const char* reason = options_.quality_.Rejection(metrics, options_.ndt_score_th_);
    c.verified_ = reason == nullptr;
    LOG(INFO) << "[loop_match] " << c.idx1_ << "->" << c.idx2_
              << " accepted=" << c.verified_ << " reason=" << (reason ? reason : "accepted")
              << " probability=" << metrics.probability
              << " overlap=" << (metrics.samples ? double(metrics.inliers) / metrics.samples : 0.0)
              << " rmse_m=" << metrics.inlier_rmse
              << " correction_m=" << metrics.correction_translation
              << " correction_deg=" << metrics.correction_rotation_deg;
    if (!c.verified_) return;
    c.Tij_ = kf1->GetOptPose().inverse() * matched;

    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_out.pcd", *output);
    // pcl::io::savePCDFileBinaryCompressed(
    //     "./data/lc_" + std::to_string(c.idx1_) + "_" + std::to_string(c.idx2_) + "_tgt.pcd", *rough_map1);
}

void LoopClosing::PoseOptimization() {
    auto v = std::make_shared<miao::VertexSE3>();
    v->SetId(cur_kf_->GetID());
    v->SetEstimate(cur_kf_->GetOptPose());
    v->SetFixed(kf_vert_.empty());  // Anchor the SE(3) gauge at the first keyframe.

    optimizer_->AddVertex(v);
    kf_vert_.emplace_back(v);

    /// 上一个关键帧的运动约束
    for (int i = 1; i < 3; i++) {
        int id = static_cast<int>(cur_kf_->GetID()) - i;
        if (id >= 0) {
            auto last_kf = all_keyframes_[id];
            auto e = std::make_shared<miao::EdgeSE3>();
            e->SetVertex(0, optimizer_->GetVertex(last_kf->GetID()));
            e->SetVertex(1, v);

            SE3 motion = last_kf->GetLIOPose().inverse() * cur_kf_->GetLIOPose();
            e->SetMeasurement(motion);
            e->SetInformation(info_motion_);
            optimizer_->AddEdge(e);
        }
    }

    if (options_.with_height_) {
        /// 高度约束
        auto e = std::make_shared<miao::EdgeHeightPrior>();
        e->SetVertex(0, v);
        e->SetMeasurement(0);
        e->SetInformation(Mat1d::Identity() * 1.0 / (options_.height_noise_ * options_.height_noise_));
        optimizer_->AddEdge(e);
    }

    /// 回环的约束
    std::vector<std::shared_ptr<miao::EdgeSE3>> new_loops;
    for (auto& c : candidates_) {
        auto e = std::make_shared<miao::EdgeSE3>();
        e->SetVertex(0, optimizer_->GetVertex(c.idx1_));
        e->SetVertex(1, optimizer_->GetVertex(c.idx2_));
        e->SetMeasurement(c.Tij_);
        e->SetInformation(info_loops_);

        auto rk = std::make_shared<miao::RobustKernelCauchy>();
        rk->SetDelta(options_.rk_loop_th_);
        e->SetRobustKernel(rk);

        optimizer_->AddEdge(e);
        edge_loops_.emplace_back(e);
        new_loops.emplace_back(e);
    }

    if (optimizer_->GetEdges().empty()) {
        return;
    }

    if (candidates_.empty()) {
        return;
    }

    std::vector<SE3> before;
    before.reserve(kf_vert_.size());
    for (const auto& vertex : kf_vert_) before.emplace_back(vertex->Estimate());
    const auto restore = [&]() {
        for (size_t i = 0; i < kf_vert_.size(); ++i) kf_vert_[i]->SetEstimate(before[i]);
    };
    const auto solve = [&](int iterations) {
        optimizer_->InitializeOptimization(0);
        optimizer_->SetVerbose(false);
        if (optimizer_->Optimize(iterations) <= 0) return false;
        for (const auto& vertex : kf_vert_) if (!vertex->Estimate().matrix().allFinite()) return false;
        return std::isfinite(optimizer_->ActiveRobustChi2());
    };
    const auto reject_new_on_failure = [&]() {
        restore();
        for (const auto& edge : new_loops) edge->SetLevel(1);
        optimizer_->InitializeOptimization(0);
        LOG(WARNING) << "[loop_graph] optimization failed; restore estimates and disable proposed loops";
    };
    if (!solve(20)) {
        reject_new_on_failure();
        return;
    }

    int cnt_outliers = 0;
    for (auto& e : edge_loops_) {
        if (e->Level() != 0) continue;
        e->ComputeError();
        if (options_.quality_.RejectEdge(e->Chi2())) {
            e->SetLevel(1);
            ++cnt_outliers;
        }
        // Keep robust kernels on accepted loops so later optimization can recheck them.
    }
    if (cnt_outliers > 0) {
        restore();  // Do not seed the second solve with an outlier-deformed trajectory.
        if (!solve(options_.quality_.refine_iterations)) {
            reject_new_on_failure();
            return;
        }
    }
    // A second solve can expose another inconsistent edge. Never publish that trajectory.
    bool unstable = false;
    for (const auto& edge : edge_loops_) {
        if (edge->Level() != 0) continue;
        edge->ComputeError();
        if (options_.quality_.RejectEdge(edge->Chi2())) {
            edge->SetLevel(1);
            unstable = true;
        }
    }
    if (unstable) {
        reject_new_on_failure();
        LOG(WARNING) << "[loop_graph] inconsistent loops after refinement; keep last committed poses";
        return;
    }
    const auto active_loops = std::count_if(edge_loops_.begin(), edge_loops_.end(),
                                          [](const auto& edge) { return edge->Level() == 0; });
    LOG(INFO) << "[loop_graph] rejected=" << cnt_outliers << " active_loops=" << active_loops
              << " second_pass=" << (cnt_outliers > 0);

    /// get results
    {
        UL graph_lock(Keyframe::PoseGraphMutex());
        for (auto& vert : kf_vert_) {
            all_keyframes_[vert->GetId()]->SetOptPose(vert->Estimate());
        }
    }

    if (loop_cb_) {
        loop_cb_(all_keyframes_);
    }

    LOG(INFO) << "optimize finished, active loops: " << active_loops;

    // LOG(INFO) << "lc: cur kf " << cur_kf_->GetID() << ", opt: " << cur_kf_->GetOptPose().translation().transpose()
    //           << ", lio: " << cur_kf_->GetLIOPose().translation().transpose();
}

}  // namespace lightning