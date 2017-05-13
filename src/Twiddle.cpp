/*
 * Twiddle.cpp
 *
 *  Created on: May 12, 2017
 *      Author: ramiz
 */

#include <iostream>
#include <exception>
#include "Twiddle.h"
#include "PID.h"
#include <numeric>


Twiddle::Twiddle()
: N_(6000), ksteps_before_error_sum_(100) {
  last_avg_error_ = 0;
}

std::string hasData1(std::string s) {
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

int Twiddle::RunPID(double params[]) {

  //initialize simulator
  int input;
  std::cout << "Please start/reset simulator and enter 1: ";
  std::cin >> input;

  //initialize PID-Controller
  PID pid;
  pid.Init(params[0], params[1], params[2]);
  //  pid.Init(0.01, 0.00002, 0.05);

  const char *close_message = "I'm closing now";
  size_t close_message_length = strlen(close_message);

  int count = 0;
  double error_sum = 0;

  uWS::Hub h;
  uWS::Group<uWS::SERVER> *group = h.createGroup<uWS::SERVER>();

  auto msg_handler = [&pid, &count, &error_sum, this, group](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {


    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData1(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
           * TODO: Calcuate steering value here, remember the steering value is
           * [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed. Maybe use
           * another PID controller to control the speed!
           */

          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if (steer_value > 1) {
            steer_value = 1;
          } else if (steer_value < -1) {
            steer_value = -1;
          }

          count = count + 1;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.5;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if (count >= this->ksteps_before_error_sum_) {
            error_sum += cte * cte;
          }

          if (count >= this->N_) {
            try {

              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::CLOSE);

              last_avg_error_ = error_sum / this->N_;
              ws.close();
              group->close();

            } catch (...) {
              std::cout << "Exception caught during close" << std::endl;
            }
          }

        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  };

  try {
    group->onMessage(msg_handler);
  } catch (...) {
    std::cout << "Caught onmsg exception " << std::endl;
  }

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  group->onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  group->onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  group->onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {

    //commenting the line below because I am manually closing it in msg handler.
    //    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port, nullptr, 0, group))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

  return 0;
}

bool Twiddle::TryIncreasing(double p[], double dp[], int i, double &best_error) {
  //try increasing p[i] by value dp[i] to see if it decreases error
  p[i] += dp[i];
  //try PID controller on updated values of p
  RunPID(p);

  //get the current run of PID error calculated
  double new_error = last_avg_error_;

  std::cout << "best error, error after increase: " << best_error << ", " << last_avg_error_ << std::endl;

  //check if new error is decreased than best_error
  if (last_avg_error_ < best_error) {
    //increasing by value dp[i] improved error than before's best_err so
    //update best error to new improved error
    best_error = last_avg_error_;

    //increase dp[i] value for next iteration
    dp[i] *= 1.5;

    return true;

  } else {
    //increasing p[i] by dp[i] did not help
    //reset p[i] back to value that was at the start of this iteration
    //which is: revert this above step --> p[i] += dp[i]
    p[i] -= dp[i];

    return false;
  }
}

bool Twiddle::TryDecreasing(double p[], double dp[], int i, double &best_error) {
  //try decreasing p[i] by value dp[i] to see if it decreases error
  p[i] -= dp[i];
  //try PID controller on updated values of p
  RunPID(p);

  //get the current run of PID error calculated
  double new_error = last_avg_error_;
  std::cout << "best error, error after decrease: " << best_error << ", " << last_avg_error_ << std::endl;

  //check if new error is decreased than best_error
  if (last_avg_error_ < best_error) {
    //decreasing by value dp[i] improved error than before's best_err so
    //update best error to new improved error
    best_error = last_avg_error_;

    //increase dp[i] value for next iteration
    dp[i] *= 1.5;

    return true;

  } else {
    //increasing p[i] by dp[i] did not help
    //reset p[i] back to value that was at the start of this iteration
    //which is: revert this above step --> p[i] -= dp[i]
    p[i] += dp[i];

    return false;
  }
}

void Twiddle::FindParams(double tolerance) {
  //  double p[] = {0, 0, 0};
  //  int result = RunPID(p);
  //  if (result == -1) {
  //    std::cout << "RunPID failed" << std::endl;
  //  }
  //  std::cout << "Error received: " << last_avg_error_ << std::endl;

  //  error = RunPID();
  //  std::cout << "Error received: " << error << std::endl;


  double p[] = {0.45, 0.000025, 22};
  double dp[] = {0.1, 0.0001, 4};

  RunPID(p);
  double best_error = last_avg_error_;

  int iteration = 1;
  while ((dp[0] + dp[1] + dp[2]) > tolerance) {

    std::cout << "-----------Iteration-" << iteration << "-----------" << std::endl;
    std::cout << "P: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
    std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2] << std::endl;
    iteration++;

    for ( int i = 0; i < 3; ++i) {
      std::cout << "-----------param-" << i << "-----------" << std::endl;
      std::cout << "Best error: " << best_error << std::endl;

      //try increasing p[i] by value dp[i] to see if it decreases error
      std::cout << "Trying increasing" << std::endl;
      bool didIncreaseHelped = TryIncreasing(p, dp, i, best_error);

      //if increasing p[i] improved error then we should continue increasing it
      if(didIncreaseHelped) {
        std::cout << "Increasing helped: params: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
        continue;
      }

      std::cout << "Trying decreasing" << std::endl;
      //try decreasing p[i] by value dp[i] to see if it decreases error
      bool didDecreaseHelped = TryDecreasing(p, dp, i, best_error);

      //if decreasing p[i] improved error then we should continue increasing it
      if(didDecreaseHelped) {
        std::cout << "Decreasing helped: params: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
        continue;
      }

      std::cout << "Neither increasing nor decreasing helped, making increase/decrease interval smaller." << std::endl;
      //as both increasing and decreasing both did not help so
      //we should decrease dp[i] value/interval to try on
      //smaller increments/decrements to p[i]
      dp[i] *= 0.5;
    }
  }

  std::cout << "----->Final params: " << p[0] << ", " << p[1] << ", " << p[2] << std::endl;
}


