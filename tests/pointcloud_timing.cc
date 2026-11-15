#include "core/lio/pointcloud_preprocess.h"
#include "core/lio/scan_timing.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace lightning;
int main() {
    auto msg=std::make_shared<sensor_msgs::msg::PointCloud2>();
    msg->height=1;msg->width=3;msg->point_step=24;msg->row_step=72;
    msg->is_bigendian=false;msg->is_dense=true;msg->data.resize(72,0);
    const std::vector<std::string> names{"x","y","z","intensity","time","ring"};
    for(size_t i=0;i<names.size();++i) {
        sensor_msgs::msg::PointField f;f.name=names[i];f.offset=i*4;f.count=1;
        f.datatype=i==5 ? sensor_msgs::msg::PointField::UINT16 : sensor_msgs::msg::PointField::FLOAT32;
        msg->fields.push_back(f);
    }
    const std::vector<float> times{-.1f,0.f,-.05f};
    for(size_t i=0;i<times.size();++i) {
        const float values[]{2.f,0.f,0.f,1.f,times[i]};
        std::memcpy(msg->data.data()+i*24,values,sizeof(values));
    }
    PointCloudPreprocess processor;
    processor.SetLidarType(LidarType::VELO32);
    processor.TimeScale()=1000.f;processor.RequireHeaderOffsets(true);
    CloudPtr output(new PointCloudType);
    processor.Process(msg,output);
    assert(output->size()==3); // Negative/zero last time must not activate yaw reconstruction.
    assert(output->front().time < -99. && output->back().time < -49.);
    const auto normalized=timing::NormalizeHeaderOffsets(output->points,100.,timing::Options{});
    assert(normalized.valid && output->front().time==0);
    msg->width=0;msg->row_step=0;msg->data.clear();
    processor.Process(msg,output);assert(output->empty());
    for (auto& f:msg->fields) if(f.name=="time") f.name="not_time";
    bool refused=false;
    try { processor.Process(msg,output); } catch (const std::invalid_argument&) { refused=true; }
    assert(refused);
    std::cout<<"PointCloud2 -> preprocessor: signed offsets retained, empty scan guarded, missing time rejected\n";
}
