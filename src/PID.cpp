#include "PID.h"
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID(double kp, double ki, double kd) :
  kp_(kp), ki_(ki), kd_(kd),
  prev_cte_(0), max_cte_(0),
  iter_cnt_(0)
{
}

PID::~PID() {}

double
PID::getSteering(double cte)
{
  if (total_cte_.size() > 20) total_cte_.pop_front();
  total_cte_.push_back(cte);
  double i_cte = 0;
  for (auto c : total_cte_) i_cte += c;
  if (fabs(cte) > max_cte_) max_cte_ = fabs(cte);
  double steer = (-kp_ * cte) + (-kd_ * (cte - prev_cte_)) + (-ki_ * i_cte);
  prev_cte_ = cte;
  iter_cnt_++;
  total_error_ += ((cte*cte) - total_error_) / iter_cnt_;
  if (steer < -1) steer = -1;
  else if (steer > 1) steer = 1;
  return steer;
}


