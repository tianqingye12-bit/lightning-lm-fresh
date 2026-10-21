#pragma once

// Linux/ROS deployments: publish a complete directory without deleting the old map.
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/syscall.h>
#include <unistd.h>

namespace lightning::maps {
inline bool ValidMapId(const std::string& id) {
    if (id.empty() || id.size() > 128 || id == "." || id == ".." || id.front() == '.') return false;
    for (unsigned char c : id) if (c < 32 || c == 127 || c == '/' || c == '\\') return false;
    return true;
}

class MapTransaction {
 public:
    explicit MapTransaction(const std::filesystem::path& destination,
                            std::uintmax_t reserve_bytes = 64ULL * 1024 * 1024) {
        namespace fs = std::filesystem;
        target_ = fs::absolute(destination).lexically_normal();
        // A trailing slash must not turn the target basename into an empty lock name.
        if (target_.filename().empty()) target_ = target_.parent_path();
        if (target_ == target_.root_path() || !ValidMapId(target_.filename().string()))
            throw std::invalid_argument("invalid map destination");
        fs::create_directories(target_.parent_path());
        auto lock_path = target_.parent_path() / ("." + target_.filename().string() + ".save.lock");
        lock_fd_ = ::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (lock_fd_ < 0) throw std::runtime_error("cannot open map save lock");
        try {
            if (::flock(lock_fd_, LOCK_EX | LOCK_NB) != 0) throw std::runtime_error("map save already in progress");
            if (fs::is_symlink(fs::symlink_status(target_)) ||
                (fs::exists(target_) && !fs::is_directory(target_)))
                throw std::runtime_error("map destination must be a real directory");
            if (fs::space(target_.parent_path()).available < reserve_bytes)
                throw std::runtime_error("insufficient free space for map save reserve (64 MiB)");
            auto pattern = (target_.parent_path() / ("." + target_.filename().string() + ".version-XXXXXX")).string();
            std::vector<char> name(pattern.begin(), pattern.end());
            name.push_back(0);
            if (!::mkdtemp(name.data())) throw std::runtime_error("cannot create map staging directory");
            staging_ = name.data();
        } catch (...) { ::close(lock_fd_); lock_fd_ = -1; throw; }
    }
    ~MapTransaction() {
        if (!published_ && !staging_.empty()) {
            std::error_code ec;
            std::filesystem::remove_all(staging_, ec);
        }
        if (lock_fd_ >= 0) ::close(lock_fd_);
    }
    MapTransaction(const MapTransaction&) = delete;
    MapTransaction& operator=(const MapTransaction&) = delete;
    const std::filesystem::path& Staging() const { return staging_; }
    const std::filesystem::path& Previous() const { return previous_; }

    // false means publication succeeded but parent-directory fsync failed. Never retry blindly.
    bool Commit() {
        if (published_) throw std::logic_error("map transaction already published");
        namespace fs = std::filesystem;
        for (const auto& entry : fs::recursive_directory_iterator(staging_)) {
            if (entry.is_symlink()) throw std::runtime_error("symlink in staged map");
            if (entry.is_regular_file()) Sync(entry.path(), false);
            else if (!entry.is_directory()) throw std::runtime_error("non-regular staged map file");
        }
        // Children before parents; directory entries must also be persisted.
        std::vector<fs::path> dirs;
        for (const auto& entry : fs::recursive_directory_iterator(staging_))
            if (entry.is_directory()) dirs.push_back(entry.path());
        for (auto it = dirs.rbegin(); it != dirs.rend(); ++it) Sync(*it, true);
        Sync(staging_, true);
        if (fs::exists(target_)) {
            if (fs::is_symlink(fs::symlink_status(target_)) || !fs::is_directory(target_))
                throw std::runtime_error("map destination changed during save");
            // RENAME_EXCHANGE: old map remains accessible at staging_, new map at target_.
            // Refuse overwrite on unsupported filesystems instead of a delete/rename gap.
            previous_ = staging_;  // Allocate before exchanging; no throwing work before published_ is set.
            if (::syscall(SYS_renameat2, AT_FDCWD, staging_.c_str(), AT_FDCWD, target_.c_str(), 2U) != 0)
                throw std::runtime_error(std::string("atomic map exchange failed; old map preserved: ") + std::strerror(errno));
        } else {
            // RENAME_NOREPLACE also protects against a competing non-cooperating writer.
            if (::syscall(SYS_renameat2, AT_FDCWD, staging_.c_str(), AT_FDCWD, target_.c_str(), 1U) != 0)
                throw std::runtime_error(std::string("map publication failed: ") + std::strerror(errno));
        }
        published_ = true;
        try { Sync(target_.parent_path(), true); } catch (...) { return false; }
        return true;
    }
 private:
    static void Sync(const std::filesystem::path& path, bool directory) {
        int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW | (directory ? O_DIRECTORY : 0));
        if (fd < 0) throw std::runtime_error("cannot open staged map for fsync: " + path.string());
        int result = ::fsync(fd);
        int error = errno;
        ::close(fd);
        if (result != 0) throw std::runtime_error("map fsync failed: " + std::string(std::strerror(error)));
    }
    int lock_fd_ = -1;
    bool published_ = false;
    std::filesystem::path target_, staging_, previous_;
};
}  // namespace lightning::maps
