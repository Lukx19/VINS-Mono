#pragma once

/**
 * Timer: Static timer which can be used to time parts of code over long time
 *
 * @author Lukas Jelinek
 */

#include <map>
#include <string>
#include <chrono>

class Timer {
public:
  /**
   * Start the timer.
   * @param timer_name The name of the timer.
   */
  static void start(const std::string &timer_name);

  /**
   * Stop the timer.
   * @param timer_name The name of the timer.
   * @return Duration of the timer.
   */
  static void stop(const std::string &timer_name);


  static std::string toString();
  static std::string toString(const std::string& timer_name);
  /**
   * @brief
   *
   */
  static void printTimers();

  static void printTimer(const std::string &timer_name);
  /**
   * Reset all timers.
   */
  static void clearAll();

private:
  /**
   * Default constructor.
   */
  Timer() = default;

  struct TimeData {
    std::chrono::steady_clock::time_point start;
    size_t count;
    float mean;
    float max;
    float min;
    float total;
    float M2;
    bool running;
    TimeData()
        : start(std::chrono::steady_clock::now()), count(0), mean(0),max(0),min(1000000.f), total(0),M2(0), running(true) {}
  };

  void updateTimer(const std::string &timer_name);
  /**
   * Start the timer internally.
   * @param timer_name The name of the timer.
   */
  void internalStart(const std::string &timer_name);

  /**
   * Stop the timer internally.
   * @param timer_name The name of the timer.
   */
  void internalStop(const std::string &timer_name);


  /**
   * Reset all timers internally.
   */
  void internalClearAll();

  /**
   * Get an instance of a timer.
   * @return An instance of a timer.
   */
  static Timer &getInstance();

  /**
   * Map between the timer names and their data.
   */
  std::map<std::string, TimeData> timers_;
};
