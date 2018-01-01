#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos)
  {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos)
  {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;

  // TODO: Initialize the pid variable.
  pid.Init(0.15, 0.001, 2.5);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    PID pid_throttle;

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          steer_value -= pid.TotalError();

          static double throttle = 1;
          static int brakeCounter = 0;
          static bool brake = false;

          if ((fabs(cte - pid.pre_cte) > 0.12) && brakeCounter == 0)
          {
            brakeCounter = 5;
            brake = true;
            std::cout << "brake: " << brake << std::endl;
          }

          // Throttle control. Brake when speed is over 28MPH
          const double target_speed = 25;
          std::cout << "Speed: " << speed << std::endl;

          if (!brake)
          {
            if (speed < target_speed)
            {
              throttle += 0.1;
              if (throttle > 1)
                throttle = 1;
            }
            else if (speed > target_speed || (fabs(cte - pid.pre_cte) > 0.12) || angle > 7 || angle < -7 || cte > 4 || cte < -5)
            {
              pid_throttle.Init(0.45, 0.001, 0.5);
              double max_throttle = -1;
              throttle = -1;
              throttle = pid_throttle.OutputThrottle(max_throttle);
            }
          }
          else
          {
            throttle = -0.0;
            brakeCounter--;
            if (brakeCounter == 0)
            {
              pid_throttle.Init(0.15, 0.001, 2.5);
              double normal_throttle = 1;
              throttle = pid_throttle.OutputThrottle(normal_throttle);
              brake = false;
            }
            std::cout << brakeCounter << std::endl;
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
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
