//
// Created by xiang on 2022/2/9.
//

#ifndef ASYNC_MESSAGE_PROCESS_H
#define ASYNC_MESSAGE_PROCESS_H

#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace lightning::sys {
using UL = std::unique_lock<std::mutex>;

/**
 * 异步消息处理类
 * 内部有线程和队列机制，保证回调是串行的
 * @tparam T
 *
 * NOTE skip设为1的时候实际不会跳帧。。设为2的时候实际走一帧跳一帧
 */
template <typename T>
class AsyncMessageProcess {
   public:
    using ProcFunc = std::function<void(const T&)>;  // 消息回调函数
    AsyncMessageProcess() = default;
    ~AsyncMessageProcess() { Quit(); }
    AsyncMessageProcess(ProcFunc proc_func, std::string name = "");

    // Configure and call Start/Quit from one owner thread; AddMessage may run concurrently.
    /// 设置处理函数
    void SetProcFunc(ProcFunc proc_func) { custom_func_ = proc_func; }

    /// 设置队列最大长度
    void SetMaxSize(size_t size) { max_size_ = std::max(size_t(1), size); }

    /// 开始处理消息
    void Start();

    /// 添加一条消息
    void AddMessage(const T& msg);

    /// 退出
    void Quit();

    /// 清空跳帧计数器，下一个数据会立即执行
    void CleanSkipCnt();
    size_t Dropped() { UL lock(mutex_); return dropped_; }
    bool Running() { UL lock(mutex_); return !exit_flag_; }

    void SetName(std::string name) { name_ = std::move(name); }
    void SetSkipParam(bool enable_skip, int skip_num) {
        if (enable_skip && skip_num < 1) throw std::invalid_argument("skip_num must be positive");
        enable_skip_ = enable_skip; skip_num_ = skip_num;
    }

    AsyncMessageProcess(const AsyncMessageProcess&) = delete;
    void operator=(const AsyncMessageProcess&) = delete;

   private:
    void ProcLoop();

    std::thread proc_;
    std::mutex mutex_;
    std::condition_variable cv_msg_;
    std::deque<T> msg_buffer_;
    bool exit_flag_ = true;
    size_t dropped_ = 0;
    size_t max_size_ = 100;  // 40
    std::string name_;

    /// 跳帧
    bool enable_skip_ = false;
    int skip_num_ = 0;
    int skip_cnt_ = 0;

    ProcFunc custom_func_;
};

template <typename T>
void AsyncMessageProcess<T>::CleanSkipCnt() {
    UL lock(mutex_);
    skip_cnt_ = 0;
}

template <typename T>
AsyncMessageProcess<T>::AsyncMessageProcess(AsyncMessageProcess::ProcFunc proc_func, std::string name) {
    custom_func_ = std::move(proc_func);
    name_ = name;
}

template <typename T>
void AsyncMessageProcess<T>::Start() {
    UL lock(mutex_);
    if (proc_.joinable()) return;
    exit_flag_ = false;
    skip_cnt_ = 0;
    dropped_ = 0;
    proc_ = std::thread([this]() { ProcLoop(); });
}

template <typename T>
void AsyncMessageProcess<T>::ProcLoop() {
    for (;;) {
        UL lock(mutex_);
        cv_msg_.wait(lock, [this]() { return exit_flag_ || !msg_buffer_.empty(); });
        if (exit_flag_) return;
        T msg = std::move(msg_buffer_.front());
        msg_buffer_.pop_front();
        lock.unlock();
        custom_func_(msg);
    }
}

template <typename T>
void AsyncMessageProcess<T>::AddMessage(const T& msg) {
    UL lock(mutex_);
    if (exit_flag_) return;
    if (enable_skip_) {
        if (skip_cnt_ != 0) {
            skip_cnt_++;
            skip_cnt_ = skip_cnt_ % skip_num_;
            return;
        }

        skip_cnt_++;
        skip_cnt_ = skip_cnt_ % skip_num_;
    }

    msg_buffer_.push_back(msg);
    while (msg_buffer_.size() > max_size_) {
        ++dropped_;
        msg_buffer_.pop_front();
    }

    cv_msg_.notify_one();
}

template <typename T>
void AsyncMessageProcess<T>::Quit() {
    {
        UL lock(mutex_);
        exit_flag_ = true;
        msg_buffer_.clear();
    }
    cv_msg_.notify_one();

    if (proc_.joinable()) {
        proc_.join();
    }
}

}  // namespace lightning::sys

#endif
