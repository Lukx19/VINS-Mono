#include "timer.h"
#include <iostream>

Timer &Timer::getInstance() {
  static Timer timer_instance;
  return timer_instance;
}

void Timer::start(const std::string &timer_name) {
  getInstance().internalStart(timer_name);
}

void Timer::stop(const std::string &timer_name) {
  getInstance().internalStop(timer_name);
}


void Timer::printTimers() {
  Timer &instance = getInstance();
  for (auto timer : instance.timers_) {
    printTimer(timer.first);
  }
}

void Timer::printTimer(const std::string &timer_name) {
  Timer &instance = getInstance();
  auto timer_iter = instance.timers_.find(timer_name);
  if (timer_iter != instance.timers_.end()) {
    float mean = timer_iter->second.mean;
    float total = timer_iter->second.total;
    size_t count = timer_iter->second.count;
    float var = timer_iter->second.M2 / (timer_iter->second.count -1);
    std::cout << "Desc: " << timer_name << " iter: " << count
              << " mean (ms): " << mean <<" total(s): "<<total / 1000.0f << " var{ms): " << var << std::endl;
  } else {
    std::cout << "Trying to print unknown timer" << std::endl;
  }
}

void Timer::clearAll() {
  getInstance().internalClearAll();
}

void Timer::updateTimer(const std::string &timer_name) {
  TimeData data = timers_.at(timer_name);
  data.running = false;
  float ms = std::chrono::duration<float, std::milli>(
          std::chrono::steady_clock::now() - data.start)
          .count();
  data.count += 1;
  float delta = ms - data.mean;
  data.mean += delta / data.count;
  data.total += ms;
  float delta2 = ms - data.mean;
  data.M2 += delta * delta2;
  // std::cout << timer_name <<"   "<< ms << std::endl;
  timers_.at(timer_name) = data;
}

void Timer::internalStart(const std::string &timer_name) {
  // check if start was called on this timer
  if (timers_.find(timer_name) == timers_.end()) {
    timers_.emplace(std::make_pair(timer_name, TimeData()));
  } else {
    if (timers_.at(timer_name).running) {
      std::cout << "CALLED START ON TIMER: " << timer_name
                << " BEFORE IT WAS STOPPED" << std::endl;
      updateTimer(timer_name);
      timers_.at(timer_name).start = std::chrono::steady_clock::now();
      timers_.at(timer_name).running = true;
    } else {
      timers_.at(timer_name).start = std::chrono::steady_clock::now();
      timers_.at(timer_name).running = true;
    }
  }
}

void Timer::internalStop(const std::string &timer_name) {
  // check if start was called on this timer
  if (timers_.find(timer_name) != timers_.end() and
      timers_.at(timer_name).running) {
    updateTimer(timer_name);
  } else {
    std::cout << "CALLED STOP ON TIMER: " << timer_name
                 << " BEFORE IT WAS STARTED" << std::endl;
  }
}



void Timer::internalClearAll() {
  timers_.clear();
}