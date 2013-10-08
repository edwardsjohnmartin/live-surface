#ifndef __TIMER_H__
#define __TIMER_H__

#include <time.h>
#include <sstream>
#include <string>
#include <iostream>

namespace oct {

class Timer {
 public:
  Timer(const std::string& msg, bool initial_msg = false)
      : _msg(msg), _t(clock()), _active(true),
        _initial_msg(initial_msg), _output(true) {
    initial();
  }

  ~Timer() {
    stop();
  }

  void suspend() {
    _active = false;
    _t = clock()-_t;
  }

  void resume() {
    _active = true;
    _t = clock()-_t;
  }

  void restart(const std::string& msg) {
    stop();
    _msg = msg;
    _t = clock();
    _active = true;
    initial();
  }

  void reset(const std::string& msg) {
    restart(msg);
  }

  void stop() {
    if (_active) {
      const double t = (clock()-_t)/static_cast<double>(CLOCKS_PER_SEC);
      std::stringstream ss;
      if (_output)
        std::cout << _msg << " " << t << " sec" << std::endl;
      _active = false;
    }
  }

  void set_output(bool output) {
    _output = output;
  }

 private:
  void initial() const {
    if (_initial_msg) {
      if (_output)
        std::cout << _msg << "..." << std::endl;
    }
  }

 private:
  std::string _msg;
  time_t _t;
  bool _active;
  bool _initial_msg;
  bool _output;
};

}  // end namespace

#endif
