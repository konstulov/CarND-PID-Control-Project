#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include <vector>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// initial parameter values
// these values were reached after 108 cycles of training starting at {0.1, 3.0, 0.004}
const vector<double> init_p = {0.12, 3.4, 0.0045}; // {0.111518, 4.08299, 0.00403688}; // {0.123803, 3.3709, 0.00450544}
// change in parameter values (for the Twiddle Algorithm)
// converged to these values after 279 cycles of training starting at {0.01, 0.3, 0.0004};
const vector<double> init_dp = {2.80662e-07, 1.0291e-05, 1.12265e-08}; // {0.03, 0.8, 0.001};
// number of steps for one training cycle
const int n_steps = 2000;
// number of steps for showing intermediate output (for progress tracking)
const int n_show = 1000;
// number of steps to look back for computing diff_cte (consecutive steps
// might have too small of a difference to properly register change)
const int n_diff = 4;
// scale parameter for the inverse relationship b/w steering angle and throttle
const double inv_scale = 10.0;
// min/max throttle
const double min_throttle = 0.3;
const double max_throttle = 1.0;
// max steering angle
const double max_steer = 0.6;
// Twiddle convergence criterion
const double twiddle_eps = 0.00001;
 
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// add values of a vector
double sum(const vector<double>& v) {
  double s = 0.0;
  for (int i=0; i<v.size(); i++) {
    s += v[i];
  }
  return s;
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  // parameter vector
  vector<double> p = init_p;
  // change in parameter vector (for the Twiddle Algorithm)
  vector<double> dp = init_dp;
  // CTE history vector - stores n_diff consecutive CTE values using circular indexing
  vector<double> ctes(n_diff, 0.0);

  // cumulative CTE (cross-track error)
  double int_cte = 0.0;
  // step counter
  int counter = 0;
  // total error (sum of squares of CTE)
  double err = 0.0;
  // best normalized error over one training cycle
  double best_err = std::numeric_limits<double>::infinity();
  // boolean flag to toggle Twiddle algo b/w increasing and decreasing parameter values
  bool increase = true;

  h.onMessage([&pid, &p, &dp, &ctes, &int_cte, &counter, &err, &best_err, &increase](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          // index for storing the new CTE in the CTE history vector
          int idx = counter % n_diff;
          // step within the current training cycle
          int cycle_step = counter % n_steps;
          
          if (counter == 0) {
            // initialize CTE history vector with the first CTE value
            ctes = vector<double>(n_diff, cte);
          } else {
            ctes[idx] = cte;
            // Perform Twiddle Algorithm updates (until convergence)
            if (sum(dp) > twiddle_eps) {
              // current training cycle
              int cycle = counter / n_steps;
              // index of the currently optimized parameter
              int i = cycle % dp.size();

              if (cycle_step == 0) {
                // completed n_steps of training -> perform Twiddle updates
                
                // calculate average error over n_steps
                double n_err = err / n_steps;
                if (i == 0) {
                  // print out current parameter values at the end of each full training cycle
                  std::cout << cycle << ": best_err = " << best_err << ", n_err = " << n_err
                  << ", p = {" << p[0] << ", " << p[1] << ", " << p[2]
                  << "}, dp = {" << dp[0] << ", " << dp[1] << ", " << dp[2] << "}" << std::endl;
                }
                if (increase) {
                  // end of "twiddle up" cycle
                  if (n_err < best_err) {
                    // achieved better error by increasing the current parameter
                    best_err = n_err;
                    dp[i] *= 1.1;
                  } else {
                    // set incease to false since incrementing the parameter didn't lead to better error
                    increase = false;
                    // set back counter since we aren't done tuning the current parameter,
                    // i.e. re-start the current training cycle with increase set to false
                    counter -= n_steps;
                  }
                } else {
                  // end of "twiddle down" cycle
                  if (n_err < best_err) {
                    // achieved better error by decreasing the current parameter
                    best_err = n_err;
                    dp[i] *= 1.1;
                  } else {
                    // neither increasing nor decreasing the current parameter lead to better performance,
                    // reset the parameter (to its previous value) and decrease the twiddle magnitude.
                    p[i] += dp[i];
                    dp[i] *= 0.9;
                  }
                  // set incease to true for the next training cycle
                  increase = true;
                }
                
                // reset total squared error and cumulative error
                err = 0.0;
                int_cte = 0.0;
                if (increase) {
                  // start "twiddle up" cycle
                  p[i] += dp[i];
                } else {
                  // start "twiddle down" cycle
                  p[i] -= 2*dp[i];
                }
                std::cout << counter << ": n_err = " << n_err << ", p[" << i << "] = " << p[i]
                << ", dp[" << i << "] = " << dp[i] << std::endl;
              }
            }
          }
          

          counter++;
          err += cte * cte;
          double prev_cte = ctes[(idx+1) % n_diff];
          double diff_cte = cte - prev_cte;
          int_cte += cte;
          double steer_value = -p[0] * cte - p[1] * diff_cte - p[2] * int_cte;
          // clip steer value to be b/w -max_steer and max_steer
          steer_value = std::min(max_steer, std::max(-max_steer, steer_value));
          if (counter % n_show == 0) {
            double cycle_err = err / (cycle_step+1);
            std::cout << "counter = " << counter << ", best_err = " << best_err << ", cycle_err = " << cycle_err << ", diff_cte = " << diff_cte << ", int_cte = " << int_cte << std::endl;
            std::cout << "CTE: " << cte << " diff_cte: " << diff_cte << " int_cte: " << int_cte << " Steering Value: " << steer_value << std::endl;
          }
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          // set throttle to be between min_throttle and max_throttle
          //double throttle = min_throttle + (max_throttle - min_throttle) * (max_steer - fabs(steer_value)) / max_steer;
          double throttle = min_throttle + (max_throttle - min_throttle) / (inv_scale * fabs(steer_value) + 1);
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; // 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
