#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace lightning::navigation {
// ROS OccupancyGrid rows start at the lower left; PGM rows start at the upper left.
struct GridSpec {
    std::size_t width = 0, height = 0;
    double resolution = 0;
    std::array<double, 3> origin{0, 0, 0};  // x, y, yaw
    bool Valid(std::size_t cells) const {
        return width > 0 && height > 0 && width <= std::numeric_limits<std::size_t>::max()/height &&
               width*height == cells && std::isfinite(resolution) && resolution > 0 &&
               std::isfinite(origin[0]) && std::isfinite(origin[1]) && std::isfinite(origin[2]);
    }
};
inline void SaveGrid(const std::string& directory, const GridSpec& grid,
                     const std::vector<int8_t>& occupancy) {
    if (!grid.Valid(occupancy.size())) throw std::invalid_argument("invalid navigation grid geometry");
    const std::filesystem::path root(directory);
    const auto pgm = root/"map.pgm", yaml = root/"map.yaml";
    std::ofstream image(pgm.string()+".tmp", std::ios::binary);
    if (!image) throw std::runtime_error("cannot create navigation image");
    image << "P5\n" << grid.width << ' ' << grid.height << "\n255\n";
    std::vector<unsigned char> row(grid.width);
    for (std::size_t y=grid.height; y-- > 0;) {
        for (std::size_t x=0; x<grid.width; ++x) {
            const auto value = occupancy[y*grid.width+x];
            row[x] = value == 0 ? 255 : value == 100 ? 0 : 128;
        }
        image.write(reinterpret_cast<const char*>(row.data()), static_cast<std::streamsize>(row.size()));
    }
    image.flush();
    if (!image) throw std::runtime_error("failed to write navigation image");
    image.close();
    std::ofstream meta(yaml.string()+".tmp");
    meta << std::setprecision(17) << "image: map.pgm\nmode: trinary\nresolution: " << grid.resolution
         << "\norigin: [" << grid.origin[0] << ", " << grid.origin[1] << ", " << grid.origin[2]
         << "]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    meta.flush();
    if (!meta) throw std::runtime_error("failed to write navigation metadata");
    meta.close();
    std::filesystem::rename(pgm.string()+".tmp", pgm);
    std::filesystem::rename(yaml.string()+".tmp", yaml);
}
}  // namespace lightning::navigation
