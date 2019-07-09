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

int main() {
  uWS::Hub h;

  PID pid;

  bool initial = true;
  bool twiddle = true;
  int verbose = 3;
  // vector<double> p = {0.2, 0.004, 3.0}; // initial
  vector<double> p = {0.18, 0.004, 3.0}; // after twiddle
  vector<double> dp = {0.1, 0.001, 1.0};
  double tol = 0.05;
  double err = 0.0;
  double mse = 0.0;
  double best_err = 0.0;
  double dp_sum = 0.0;
  for (int i=0; (unsigned)i<dp.size(); ++i)
  {
    dp_sum += dp[i];
  }
  int count = 0;
  int n = 100;
  int p_index = 0;
  int prev_p_index = p.size();

  /**
   * TODO: Initialize the pid variable.
   */
   pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid, &initial, &twiddle, &verbose, &p, &dp, &tol, &err, &mse,
               &best_err, &dp_sum, &count, &n, &p_index, &prev_p_index]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          // double speed = std::stod(j[1]["speed"].get<string>());
          // double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
           if (verbose == 1)
           {
             std::cout << "Initial: " << initial << std::endl;
             std::cout << "Twiddle: " << twiddle << std::endl;
           }

           if (twiddle)
           {
             if (initial)
             {
               if (verbose == 1)
               {
                 std::cout << "Initial and Twiddle? " << (initial && twiddle) << std::endl;
               }

               pid.UpdateError(cte);
               steer_value = pid.TotalError();

               count += 1;
               if (verbose == 1)
               {
                 std::cout << "Count: " << count << std::endl;
               }

               if (count > n)
               {
                 err += cte*cte;
               }

               if (count == 2*n)
               {
                 best_err = err/n;
                 if (verbose == 2)
                 {
                   std::cout << "err: " << err << std::endl;
                   std::cout << "n: " << n << std::endl;
                   std::cout << "Initial error: " << best_err << std::endl;
                 }
                 initial = false;

                 err = 0.0;
                 count = 0;

               }
             }
             else
             {

               if (verbose == 1)
               {
                 std::cout << "Begin Twiddle"  << std::endl;
                 std::cout << "twiddle: " << twiddle << std::endl;
                 std::cout << "initial: " << initial << std::endl;
               }

               if (dp_sum > tol)
               {
                 if (count == 0)
                 {
                   if (p_index == prev_p_index)
                   {
                     p[p_index] -= 2*dp[p_index];
                   }
                   else
                   {
                     p[p_index] += dp[p_index];
                   }


                   if (verbose == 3)
                   {
                     std::cout << "P: ";
                     for (int i=0; (unsigned)i<p.size(); ++i)
                     {
                       std::cout << p[i] << " ";
                     }
                     std::cout << std::endl;
                   }


                   pid.Init(p[0], p[1], p[2]);
                 }

                 if (verbose == 2)
                 {
                   std::cout << "Count: " << count << std::endl;
                   std::cout << "P index: " << p_index << std::endl;
                 }

                 pid.UpdateError(cte);
                 steer_value = pid.TotalError();

                 count += 1;

                 if (count > n)
                 {
                   err += cte*cte;
                 }

                 if (count == 2*n)
                 {
                   mse = err/n;


                   if (verbose == 3)
                   {
                     std::cout << "MSE: " << mse << std::endl;
                   }


                   if (mse < best_err)
                   {
                     best_err = mse;
                     if (verbose == 3)
                     {
                       std::cout << "New Best Error: " << best_err << std::endl;
                     }

                     dp[p_index] *= 1.1;

                     prev_p_index = p.size();

                     p_index += 1;
                     if ((unsigned)p_index == p.size())
                     {
                       p_index = 0;
                     }
                   }
                   else
                   {
                     if (p_index == prev_p_index)
                     {
                       p[p_index] += dp[p_index];

                       if (verbose == 3)
                       {
                         std::cout << "P: ";
                         for (int i=0; (unsigned)i<p.size(); ++i)
                         {
                           std::cout << p[i] << " ";
                         }
                         std::cout << std::endl;
                       }

                       dp[p_index] *= 0.9;

                       prev_p_index = p.size();

                       p_index += 1;
                       if ((unsigned)p_index == p.size())
                       {
                         p_index = 0;
                       }
                     }
                     else
                     {
                       prev_p_index = p_index;
                     }
                   }

                   count = 0;
                   err = 0.0;
                 }

                 if (verbose == 2)
                 {
                   std::cout << "Count: " << count << std::endl;
                   std::cout << "P index: " << p_index << std::endl;
                   std::cout << "Previous p index: " << prev_p_index << std::endl;
                   std::cout << "Test: " << (count==0 && p_index==0 && p_index!=prev_p_index) << std::endl;
                 }

                 if (count == 0 && p_index == 0 && p_index != prev_p_index)
                 {
                   dp_sum = 0.0;
                   for (int i=0; (unsigned)i<dp.size(); ++i)
                   {
                     dp_sum += dp[i];
                   }

                   if (verbose == 2)
                   {
                     std::cout << "Sum of dp: " << dp_sum << std::endl;
                   }
                 }

               }
               else
               {
                 pid.UpdateError(cte);
                 steer_value = pid.TotalError();
               }
             }

           }
           else
           {
             if (verbose == 2)
             {
               std::cout << "No Twiddle"  << std::endl;
               std::cout << "twiddle: " << twiddle << std::endl;
               std::cout << "initial: " << initial << std::endl;
             }

             pid.UpdateError(cte);
             steer_value = pid.TotalError();
           }

          // DEBUG
          /*
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;
          */

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
