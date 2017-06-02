#ifndef PID_H
#define PID_H

#include <list>

class PID {
public:

  /*
  * Constructor
  */
  PID(double kp, double ki, double kd);

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
   * Update the PID error variables given cross track error.
   */
  double getSteering(double cte);

  inline double totalError() const { return total_error_; }
  inline unsigned int iterCount() const { return iter_cnt_; }
  inline double prevCTE() const { return prev_cte_; }
  inline double maxCTE() const { return max_cte_; }
  
private:
  /*
   * Coefficients
   */ 
  double kp_;
  double ki_;
  double kd_;

  std::list<double> total_cte_;
  double prev_cte_;
  double max_cte_;
  double total_error_;
  unsigned int iter_cnt_;
};

#endif /* PID_H */
