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

/*
const double tau_p = 0.2;
const double tau_d = 3.0;
const double tau_i = 0.004;
*/
const int n_steps = 1000;
 
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
  vector<double> p = {0.2, 3.0, 0.004};
  vector<double> dp = {0.05, 0.5, 0.001};
  double prev_cte = 0.0;
  double int_cte = 0.0;
  int counter = 0;
  double err = 0.0;
  double best_err = std::numeric_limits<double>::infinity();
  bool increase = true;

  h.onMessage([&pid, &p, &dp, &prev_cte, &int_cte, &counter, &err, &best_err, &increase](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          
          if (sum(dp) > 0.00001) {
            // index of the currently optimized parameter
            int i = (counter / n_steps) % dp.size();
            if (counter % n_steps == 0) {
              err = 0.0;
              prev_cte = cte;
              int_cte = 0.0;
              // new training cycle
              if (increase) {
                // start "twiddle up" cycle
                p[i] += dp[i];
              } else {
                // start "twiddle down" cycle
                p[i] -= 2*dp[i];
              }
              std::cout << counter << ": p[" << i << "] = " << p[i] << std::endl;
            } else if (counter % n_steps == n_steps - 1) {
              // completed n_steps of training
              
              // calculate average error over n_steps
              double n_err = err / n_steps;
              if (increase) {
                // end of "twiddle up" cycle
                if (n_err < best_err) {
                  // achieved better error by increasing the current parameter
                  best_err = n_err;
                  dp[i] *= 1.1;
                } else {
                  // set incease to false since incrementing the parameter didn't lead to better error
                  increase = false;
                  // set back counter since we aren't done tuning the current parameter
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
              std::cout << counter << ": n_err = " << n_err << ", p[" << i << "] = " << p[i] << ", dp[" << i << "] = " << dp[i] << std::endl;
            }
          }
          counter++;
          err += cte * cte;
          double diff_cte = cte - prev_cte;
          prev_cte = cte;
          int_cte += cte;
          double steer_value = -p[0] * cte - p[1] * diff_cte - p[2] * int_cte;
          if (counter % 500 == 0) {
            std::cout << "counter = " << counter << ", best_err = " << best_err << ", err = " << err << ", diff_cte = " << diff_cte << ", int_cte = " << int_cte << std::endl;
          }
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
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
