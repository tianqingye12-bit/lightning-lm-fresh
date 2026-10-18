// Requires the complete ROS2/PCL build; not part of the std-only tests.
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include "common/keyframe.h"
#include "core/g2p5/g2p5.h"
#include "core/maps/tiled_map.h"

namespace {
void Require(bool condition, const char* reason) { if (!condition) throw std::runtime_error(reason); }
}
int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;
    try {
        using namespace lightning;
        Require(argc==2, "provide a NEW temporary directory");
        const std::filesystem::path root(argv[1]);
        Require(!std::filesystem::exists(root), "test directory must not exist");
        std::filesystem::create_directories(root);
        const auto tiles = root/"tiles";
        std::filesystem::create_directory(tiles);
        CloudPtr points(new PointCloudType);
        for (float x : {-51.f, 0.f, 151.f}) {
            PointType point; point.x=x; point.y=0; point.z=1; point.intensity=7; point.time=0;
            points->push_back(point);
        }
        TiledMap original;
        Require(original.ConvertFromFullPCD(points, SE3(), tiles.string()), "tile conversion failed");
        TiledMap::Options map_options; map_options.map_path_=tiles.string(); map_options.load_dyn_cloud_=false;
        TiledMap reloaded(map_options);
        Require(reloaded.LoadMapIndex(), "tile index reload failed");
        reloaded.LoadOnPose(SE3());
        Require(reloaded.GetAllMap()->size()==3, "single-point tile lost its first point");

        const auto config = root/"grid.yaml";
        std::ofstream yaml(config);
        yaml << "g2p5:\n  esti_floor: false\n  min_th_floor: 0.3\n  max_th_floor: 1.2\n"
                "  floor_height: -1.0\n  lidar_height: 0.0\n  grid_map_resolution: 0.2\n";
        yaml.close();
        const SE3 extrinsic(SO3::exp(Vec3d(0,0,.2)), Vec3d(.2,-.1,.3));
        CloudPtr frame(new PointCloudType);
        for (int angle=0; angle<360; ++angle) {
            const double a=angle*3.14159265358979323846/180;
            for (int j=0; j<3; ++j) {
                const double range=3+j*2;
                const Vec3d xyz=extrinsic*Vec3d(range*std::cos(a),range*std::sin(a),j==0 ? -1. : 0.);
                PointType point; point.getVector3fMap()=xyz.cast<float>(); point.intensity=7; point.time=0;
                frame->push_back(point);
            }
        }
        g2p5::G2P5::Options options; options.online_mode_=false; options.verbose_=false;
        g2p5::G2P5 retained(options);
        options.retain_offline_keyframes_=false;
        g2p5::G2P5 streaming(options);
        retained.Init(config.string()); streaming.Init(config.string());
        for (unsigned long i=0; i<4; ++i) {
            NavState state; state.timestamp_=i+1;
            auto keyframe=std::make_shared<Keyframe>(i,frame,state,extrinsic);
            retained.PushKeyframe(keyframe); streaming.PushKeyframe(keyframe);
        }
        auto a=retained.GetNewestMap()->ToROS(), b=streaming.GetNewestMap()->ToROS();
        Require(a.info.width==b.info.width && a.info.height==b.info.height && a.data==b.data,
                "streamed grid differs from retained-frame rendering");
        Require(std::abs(b.info.resolution-.2)<1e-6, "grid resolution changed");
        g2p5::G2P5::Options small_options=options; small_options.max_grid_cells_=256;
        g2p5::G2P5 small(small_options); small.Init(config.string());
        bool rejected=false;
        try { small.PushKeyframe(std::make_shared<Keyframe>(0,frame,NavState(),extrinsic)); }
        catch (const std::length_error&) { rejected=true; }
        Require(rejected, "oversized G2P5 allocation not rejected");
        LOG(INFO) << "PCL tile reload, streamed G2P5 equivalence, extrinsics and budget checks passed";
        return 0;
    } catch (const std::exception& error) {
        LOG(ERROR) << error.what();
        return 1;
    }
}
