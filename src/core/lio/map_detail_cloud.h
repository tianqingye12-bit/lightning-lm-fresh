#ifndef LIGHTNING_MAP_DETAIL_CLOUD_H
#define LIGHTNING_MAP_DETAIL_CLOUD_H

#include <pcl/filters/voxel_grid.h>
#include "core/lio/map_detail.h"

namespace lightning {
namespace mapping {

// Read-only input: retaining a keyframe must never decimate the registration scan.
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr RetainCloud(
    const typename pcl::PointCloud<PointT>::Ptr& input, const DetailOptions& options) {
    using Cloud = pcl::PointCloud<PointT>;
    typename Cloud::Ptr retained = input;
    if (options.keyframe_resolution > 0.0) {
        retained.reset(new Cloud());
        pcl::VoxelGrid<PointT> voxel;
        const float leaf = static_cast<float>(options.keyframe_resolution);
        voxel.setLeafSize(leaf, leaf, leaf);
        voxel.setInputCloud(input);
        voxel.filter(*retained);
    }
    const std::size_t budget = options.keyframe_max_points;
    if (budget > 0 && retained->size() > budget) {
        typename Cloud::Ptr limited(new Cloud());
        limited->reserve(budget);
        for (std::size_t i = 0; i < budget; ++i) {
            limited->push_back(retained->points[BudgetIndex(i, retained->size(), budget)]);
        }
        retained = limited;
    }
    return retained;
}

}  // namespace mapping
}  // namespace lightning
#endif
