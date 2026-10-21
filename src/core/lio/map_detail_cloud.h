#ifndef LIGHTNING_MAP_DETAIL_CLOUD_H
#define LIGHTNING_MAP_DETAIL_CLOUD_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include "core/lio/map_detail.h"
#include "core/lio/observed_voxel.h"

namespace lightning {
namespace mapping {

template <typename PointT>
using CloudObservedSampler = ObservedVoxelSampler<PointT, decltype(std::declval<pcl::PointCloud<PointT>>().points)>;

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ObservedVoxelCloud(
    const typename pcl::PointCloud<PointT>::Ptr& input, double leaf) {
    using Cloud = pcl::PointCloud<PointT>;
    typename Cloud::Ptr output(new Cloud());
    CloudObservedSampler<PointT> sampler(leaf);
    for (const auto& point : input->points) sampler.Add(point);
    output->points = sampler.ReleasePoints();
    output->header = input->header;
    output->sensor_origin_ = input->sensor_origin_;
    output->sensor_orientation_ = input->sensor_orientation_;
    output->height = 1;
    output->width = output->size();
    output->is_dense = false;
    return output;
}

// Allocate even for identity: callers may hold shared registration input.
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr CloudInBodyFrame(
    const typename pcl::PointCloud<PointT>::Ptr& input, const Eigen::Matrix4d& T_body_lidar) {
    typename pcl::PointCloud<PointT>::Ptr body(new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*input, *body, T_body_lidar);
    return body;
}

// Read-only input: retaining a keyframe must never decimate the registration scan.
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr RetainCloud(
    const typename pcl::PointCloud<PointT>::Ptr& input, const DetailOptions& options) {
    using Cloud = pcl::PointCloud<PointT>;
    typename Cloud::Ptr retained = input;
    if (options.keyframe_resolution > 0.0) {
        if (options.preserve_observed_points) {
            retained = ObservedVoxelCloud<PointT>(input, options.keyframe_resolution);
        } else {
            retained.reset(new Cloud());
            pcl::VoxelGrid<PointT> voxel;
            const float leaf = static_cast<float>(options.keyframe_resolution);
            voxel.setLeafSize(leaf, leaf, leaf);
            voxel.setInputCloud(input);
            voxel.filter(*retained);
        }
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
