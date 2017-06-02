#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <climits>
#include <vector>

using json = nlohmann::json;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  const double TOLERANCE = 0.1;
  int TWIDDLE_ITER = -1;
  if (argc >= 2) TWIDDLE_ITER = atoi(argv[1]);
  
  // We're not realling covering the cost for p(0, 0, 0) but it should
  // not be relevant in this scenario
  // If no argument is passed, we don't perform the twiddle parameter
  // tuning and instead use a set of predefined PID parameters
  vector<double> dp{0.4, 0.1, 2};
  vector<double> p{dp[0], 0, 0};
  if (TWIDDLE_ITER <= 0) p = vector<double>{0.25, 0.0, 8};

  // Initializing with a small but not unbeatable number
  // Trying to get rid of scenarios where the car being stopped on the curb
  // is actually better than moving (another option would be to take the
  // average speed into account
  double best_error = std::numeric_limits<double>::max();
  PID pid(p[0], p[1], p[2]);
  bool increase = true;
  int index = 0;
  int tuning_iter = 0;

  h.onMessage([&pid, &p, &dp, &index,
               &increase, &best_error, &tuning_iter,
               &TWIDDLE_ITER, TOLERANCE](uWS::WebSocket<uWS::SERVER> ws,
                                         char *data,
                                         size_t length,
                                         uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // Twiddle update
          if (pid.iterCount() >= TWIDDLE_ITER && TWIDDLE_ITER > 0) {
            double cur_error = pid.totalError();
            std::cout << "----------- Twiddle update (" << tuning_iter++ << ")" << std::endl
                      << " P = [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl
                      << " DP = [" << dp[0] << ", " << dp[1] << ", " << dp[2] << "]" << std::endl
                      << " Current error: " << cur_error << std::endl
                      << " Best error: " << best_error << std::endl
                      << " Index: " << index << std::endl
                      << " Increase: " << increase << std::endl;
            // Simple state machine controlling which index to update and by how much
            if (cur_error < best_error) {              
              best_error = cur_error;
              dp[index] *= 1.1;
              index = (index + 1) % 3;
              increase = true;
              p[index] += dp[index];
            } else {
              if (increase) {
                p[index] -= 2 * dp[index];
                increase = false;
              } else {
                p[index] += dp[index];
                dp[index] *= 0.9;
                index = (index + 1) % 3;
                increase = true;
                p[index] += dp[index];
              }
            }
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            pid = PID(p[0], p[1], p[2]);
            double dp_sum = dp[0] + dp[1] + dp[2];
            if (dp_sum < TOLERANCE) {
              std::cout << "Twiddle finished" << std::endl
                        << " P = [" << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl
                        << " DP = [" << dp[0] << ", " << dp[1] << ", " << dp[2] << "]" << std::endl
                        << " Best error: " << best_error << std::endl;
              TWIDDLE_ITER = -1;
            }
            std::cout << "----------- Trying next  P = ["
                      << p[0] << ", " << p[1] << ", " << p[2] << "]" << std::endl;
          } else {
            // Steering update
            // j[1] is the data JSON object
            double cte = std::stod(j[1]["cte"].get<std::string>());
            double speed = std::stod(j[1]["speed"].get<std::string>());
            double angle = std::stod(j[1]["steering_angle"].get<std::string>());          
            double steer_value = pid.getSteering(cte);
             
            // DEBUG
            // std::cout << pid.iterCount()
            //           << " cte = " << cte << ", "
            //           << " total cte = " << pid.totalCTE() << ", "
            //           << " prev cte = " << pid.prevCTE() << ", "
            //           << " steering = " << steer_value
            //           << std::endl;

            double max_cte = pid.maxCTE();
            if (max_cte < 1e-9) max_cte = 1000;
            double throttle = 0.5 - (0.49 * (fabs(cte) / max_cte));
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
