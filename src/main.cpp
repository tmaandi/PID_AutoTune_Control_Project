#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

/*
 * Debug flag
 */
#define DEBUG 0

/*
 * for convenience
 */
using json = nlohmann::json;

/*
 * For converting back and forth between radians and degrees.
 */
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

/*
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 */

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

/*
 * Declare PID object
 */
PID pid;

int main()
{
  uWS::Hub h;

  /*
   * Initialize the pid variable.
   */
  if (pid.is_initialized == false)
  {
    /*
     * Initializing the PID with optimized gains found via twiddle auto-tuning
     * Set this flag to 'false' to run twiddle (PID Auto-tuning) for further
     * fine tuning
     */
    pid.Init(0.157667, 0.000381473, 1.45305, true);
  }

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    /*
     * "42" at the start of the message means there's a websocket message event.
     * The 4 signifies a websocket message
     * The 2 signifies a websocket event
     */
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "")
      {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry")
        {
          /*
           * j[1] is the data JSON object
           */
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;

          /********************************
           * Calculating 'Steering' value *
           ********************************/

          /*
           * Run 'twiddle()' algorithm for PID auto-tuning untill
           * PID gains are otimized
           */
          if (pid.pid_optimized == false)
          {
            pid.twiddle();

            steer_value = pid.CalculatePIDOut(cte);
          }
          else
          {
            steer_value = pid.CalculatePIDOut(cte);
          }

          if (DEBUG == 1)
          {
            std::cout<<"Kp: "<<pid.Kp<<" Ki: "<<pid.Ki<<" Kd: "<<pid.Kd<<std::endl;
            std::cout << "reset_simulator: "<<pid.reset_simulator<< std::endl;
            std::cout << "tunetest_count: "<<pid.tunetest_count<< std::endl;
            std::cout << "pid_optimized: "<<pid.pid_optimized<< std::endl;
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
            std::cout << "reset_simulator: "<<pid.reset_simulator<< std::endl;
          }

          /*
           * Reset the simulator once the car reaches the end of
           * tuning strech defined for the track
           */
          if (pid.reset_simulator == true)
          {
            pid.reset_simulator = false;
            pid.tunetest_count = 0;
            pid.p_error = 0;
            pid.i_error = 0;
            pid.d_error = 0;

            /*
             * Resetting the simulator, car back to starting position
             */
            std::string msg = "42[\"reset\",{}]";

            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
          else
          {
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          }
        }
      }
      else
      {

        /*
         * Manual driving
         */
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });


  /*
   * We don't need this since we're not using HTTP but if it's removed the program
   * doesn't compile :-(
   */
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
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
