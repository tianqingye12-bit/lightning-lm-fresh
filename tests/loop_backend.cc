// Integration test: compile against the real built lightning.libs and miao backend.
#include "core/loop_closing/loop_closing.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include <atomic>
#include <thread>

class LoopHarness : public lightning::LoopClosing {
 public:
    static Options TestOptions() {
        Options options;
        options.with_height_ = false;
        options.online_mode_ = false;
        options.verbose_ = false;
        return options;
    }
    LoopHarness() : LoopClosing(TestOptions()) {
        Init("");
        SetLoopClosedCB([this](const std::vector<lightning::Keyframe::Ptr>& keyframes) {
            ++callbacks;
            for (const auto& kf : keyframes) assert(kf->GetOptPose().matrix().allFinite());
        });
    }
    void Append(double x, bool propose_loop = false, double measurement = 0) {
        lightning::NavState state;
        state.pos_ = lightning::Vec3d(x, 0, 0);
        state.timestamp_ = x;
        lightning::CloudPtr cloud(new lightning::PointCloudType());
        cur_kf_ = std::make_shared<lightning::Keyframe>(all_keyframes_.size(), cloud, state);
        all_keyframes_.push_back(cur_kf_);
        candidates_.clear();
        if (propose_loop) {
            lightning::LoopCandidate c(0, cur_kf_->GetID());
            c.Tij_ = lightning::SE3(lightning::SO3(), lightning::Vec3d(measurement, 0, 0));
            c.verified_ = true;
            candidates_.push_back(c);
        }
        PoseOptimization();
    }
    void CheckActiveEdges() {
        std::set<const lightning::miao::Edge*> seen;
        for (const auto* edge : optimizer_->ActiveEdges()) {
            assert(edge->Level() == 0);
            assert(seen.insert(edge).second); // no duplicates across repeated graph initialization
        }
    }
    double Position(std::size_t i) { return all_keyframes_[i]->GetOptPose().translation().x(); }
    bool Anchored() { return kf_vert_.front()->Fixed(); }
    int LoopLevel(std::size_t i) { return edge_loops_.at(i)->Level(); }
    void ForceFailure(bool value) {
        stopped = value;
        optimizer_->SetForceStopFlag(&stopped);
    }
    int callbacks = 0;
    bool stopped = false;
};

int main() {
    LoopHarness graph;
    graph.Append(0); graph.Append(1);
    assert(graph.Anchored() && graph.callbacks == 0);
    graph.Append(2, true, 2.15);
    assert(graph.callbacks == 1 && graph.LoopLevel(0) == 0);
    graph.CheckActiveEdges();
    assert(std::abs(graph.Position(0)) < 1e-10);
    graph.Append(3, true, 100); // grossly contradictory edge must be removed and solved again
    assert(graph.LoopLevel(1) == 1 && graph.callbacks == 2);
    assert(std::abs(graph.Position(3) - 3.0) < 0.3);
    graph.CheckActiveEdges();
    graph.Append(4, true, 4.15);
    assert(graph.LoopLevel(1) == 1 && graph.callbacks == 3); // rejected edge must not re-enter a later solve
    graph.CheckActiveEdges();
    assert(std::abs(graph.Position(0)) < 1e-10);
    const int callbacks = graph.callbacks;
    const double before = graph.Position(2);
    graph.ForceFailure(true);
    graph.Append(5, true, 5.15);
    assert(graph.callbacks == callbacks); // no partially optimized snapshot is published
    assert(graph.LoopLevel(3) == 1);
    assert(std::abs(graph.Position(2) - before) < 1e-10);
    graph.CheckActiveEdges();
    graph.ForceFailure(false);

    // Whole-graph readers must see one committed revision, never a mixed set of poses.
    std::vector<lightning::Keyframe::Ptr> frames;
    for (int i = 0; i < 8; ++i) {
        lightning::CloudPtr cloud(new lightning::PointCloudType());
        frames.push_back(std::make_shared<lightning::Keyframe>(i, cloud, lightning::NavState()));
    }
    std::atomic<bool> start{false};
    std::thread writer([&] {
        while (!start.load()) std::this_thread::yield();
        for (int revision = 1; revision <= 500; ++revision) {
            std::lock_guard<std::mutex> lock(lightning::Keyframe::PoseGraphMutex());
            for (const auto& kf : frames) {
                kf->SetOptPose(lightning::SE3(lightning::SO3(), lightning::Vec3d(revision, 0, 0)));
            }
        }
    });
    start.store(true);
    for (int read = 0; read < 500; ++read) {
        auto poses = lightning::Keyframe::SnapshotPoses(frames);
        for (const auto& pose : poses) assert(pose.translation().x() == poses.front().translation().x());
    }
    writer.join();
    std::cout << "real loop graph anchor, rejection/refinement, active set and rollback tests passed\n";
}
