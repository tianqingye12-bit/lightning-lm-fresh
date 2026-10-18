#include "ui/ui_trajectory.h"

#include <GL/gl.h>

namespace lightning::ui {

void UiTrajectory::AddPt(const SE3& pose) {
    // 如果轨迹点超出阈值 直接删除前一半点
    segments_.Append(pos_.size());
    pos_.emplace_back(pose.translation().cast<float>());
    if (pos_.size() > max_size_) {
        const size_t removed = pos_.size() / 2;
        pos_.erase(pos_.begin(), pos_.begin() + removed);
        segments_.Trim(removed);
    }
}

void UiTrajectory::Render() {
    // 点线形式
    glLineWidth(5.0);
    glColor3f(color_[0], color_[1], color_[2]);
    const auto& starts = segments_.Starts();
    for (size_t segment = 0; segment < starts.size(); ++segment) {
        const size_t end = segment + 1 < starts.size() ? starts[segment + 1] : pos_.size();
        if (end <= starts[segment] + 1) continue;
        glBegin(GL_LINE_STRIP);
        for (size_t i = starts[segment]; i < end; ++i) {
            const auto& p = pos_[i]; glVertex3f(p[0], p[1], p[2]);
        }
        glEnd();
    }
}

}  // namespace lightning::ui
