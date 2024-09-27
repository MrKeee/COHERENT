#pragma once

#include <chrono>
#include <thread>
#include <functional>
#include <iostream>
#include <memory>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
// #include <spdlog/spdlog.h>

namespace UNITREE_ARM {

/**
 * @brief A timer to ensure the frequency of thread execution.
 */
class Timer
{
public:
  /**
   * @brief Construct a new Timer object
   * 
   * @param period_sec time, Unit: second
   */
  Timer(double period_sec) {
    period_ = period_sec;
    start(); // default
  }

  double period() {return period_; }

  /**
   * @brief Update the beginning time.
   */
  void start() { start_time_ = std::chrono::steady_clock::now();  }

  /**
   * @brief Caculate the time from the beginning to the present.
   * 
   * @return second
   */
  double elasped_time() 
  {
    auto end_time_ = std::chrono::steady_clock::now();
    auto elasped_time = end_time_ - start_time_;
    size_t t = std::chrono::duration_cast<std::chrono::microseconds>(elasped_time).count();
    return (double)t/1000000.0;
  }

  /**
   * @brief Caculate the remaining time to a period
   * 
   * @return second
   */
  double wait_time() { return period_ - elasped_time(); }

  /**
   * @brief Sleep for wait_time() until a period finished.
   * 
   * If it has timeout, do nothing.
   */
  void sleep()
  {
    double waitTime = wait_time();
    if(waitTime > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(size_t(waitTime*1000000)));
    }
    start();// If the last one ends and then start a new timer.
  }

private:
  double period_;
  std::chrono::steady_clock::time_point start_time_;
};
typedef std::shared_ptr<Timer> TimerPtr;


/**
 * @brief Maintains a thread to run once every period.
 */
class LoopFunc
{
public:
  /**
   * @brief Construct a new Loop object
   * 
   * @param name Indicate what the loop aims to
   * @param period time, Unit: second
   * @param callback the running function pointer
   */
  LoopFunc(std::string name, double period, std::function<void ()> callback) 
    :name_(name), cb_(callback) {
    timer_ = std::make_shared<Timer>(period);
  }
  ~LoopFunc() { shutdown(); }
  
  void start()
  {
    if(isrunning_)
    {
      // spdlog::warn("Loop {} is already running.", name_);
      return;
    }

    isrunning_ = true;
    thread_ = std::thread(&LoopFunc::running_impl, this);
  }

  void shutdown()
  {
    if(!isrunning_) return;
    isrunning_ = false;
    thread_.join();
    // spdlog::debug("[Report] The time out rate of thread {0} is {1}%",
    //                name_, 100.0*(double)timeout_times_/(double)run_times_);
  }

  void spinOnce()
  {
    timer_->start();
    ++run_times_;
    cb_();

    if(timer_->wait_time() > 0) {
      timer_->sleep();
    } else {
      ++timeout_times_;
      // spdlog::debug("[Report] Loop {0} timeout. It has cost {1}s.", name_, timer_->elasped_time());
    }
  }
private:
  void running_impl() {
    while (isrunning_) {
      spinOnce();
    }
    // spdlog::debug("Loop {} end.", name_);
  }

  std::string name_{};
  bool isrunning_ = false;
  std::shared_ptr<Timer> timer_;

  std::function<void ()> cb_;
  std::thread thread_;

  size_t run_times_ = 0;
  size_t timeout_times_ = 0;
};

}