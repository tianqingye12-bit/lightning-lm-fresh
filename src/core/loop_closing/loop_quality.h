#ifndef LIGHTNING_LOOP_QUALITY_H
#define LIGHTNING_LOOP_QUALITY_H

#include <cmath>
#include <cstddef>
#include <limits>

namespace lightning::loop_quality {

struct MatchMetrics {
    bool converged = false;
    bool finite_transform = false;
    double probability = 0;
    std::size_t samples = 0;
    std::size_t inliers = 0;
    double inlier_rmse = std::numeric_limits<double>::infinity();
    double correction_translation = std::numeric_limits<double>::infinity();
    double correction_rotation_deg = std::numeric_limits<double>::infinity();
};

struct Options {
    double overlap_distance = 0.5;  // metres
    double min_overlap = 0.3;
    double max_inlier_rmse = 0.3;   // metres
    double max_translation = 5.0;   // correction relative to predicted pose
    double max_rotation_deg = 30.0;
    int max_samples = 2000;
    int min_inliers = 50;
    int max_candidates = 3;
    double max_loop_chi2 = 25.0;    // separate from the Cauchy kernel scale
    int refine_iterations = 10;

    bool Valid() const {
        return std::isfinite(overlap_distance) && overlap_distance > 0 &&
               std::isfinite(min_overlap) && min_overlap > 0 && min_overlap <= 1 &&
               std::isfinite(max_inlier_rmse) && max_inlier_rmse > 0 &&
               std::isfinite(max_translation) && max_translation > 0 &&
               std::isfinite(max_rotation_deg) && max_rotation_deg > 0 && max_rotation_deg <= 180 &&
               max_samples > 0 && min_inliers > 0 && min_inliers <= max_samples && max_candidates > 0 &&
               std::isfinite(max_loop_chi2) && max_loop_chi2 > 0 && refine_iterations > 0;
    }

    const char* Rejection(const MatchMetrics& m, double min_probability) const {
        if (!m.converged) return "not_converged";
        if (!m.finite_transform) return "invalid_transform";
        if (!std::isfinite(m.probability) || m.probability <= min_probability) return "ndt_probability";
        if (m.samples == 0 || m.inliers > m.samples || m.inliers < static_cast<std::size_t>(min_inliers))
            return "insufficient_inliers";
        if (static_cast<double>(m.inliers) / m.samples < min_overlap) return "low_overlap";
        if (!std::isfinite(m.inlier_rmse) || m.inlier_rmse < 0 || m.inlier_rmse > max_inlier_rmse)
            return "inlier_rmse";
        if (!std::isfinite(m.correction_translation) || m.correction_translation < 0 ||
            m.correction_translation > max_translation) return "translation_correction";
        if (!std::isfinite(m.correction_rotation_deg) || m.correction_rotation_deg < 0 ||
            m.correction_rotation_deg > max_rotation_deg) return "rotation_correction";
        return nullptr;
    }

    bool RejectEdge(double chi2) const { return !std::isfinite(chi2) || chi2 < 0 || chi2 > max_loop_chi2; }
};

}  // namespace lightning::loop_quality
#endif
