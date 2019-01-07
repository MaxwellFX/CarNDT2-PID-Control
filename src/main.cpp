#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <exception>

#define WRITE_TO_FILE 0

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

    std::ofstream file;
#if WRITE_TO_FILE
    file.open("../Full_Params.csv");
    file << "CTE, steer_value" << std::endl;
#endif
    PID pid_steer, pid_throttle;

    // TODO: Initialize the pid variable.
	// pid_steer.Init(0.1744, 0.0, 0.0);
    // pid_steer.Init(0.1744, 0.00042837, 0);
    // pid_steer.Init(0.1744, 0.0, 2.525);
    pid_steer.Init(0.1744, 0.00033837, 2.525);
	pid_throttle.Init(0.376731, 0.0, 0.0206185);

    bool bTwiddle = false;
    pid_steer.SetTuningParams(2020, 650.0, 0.05);

    h.onMessage([&pid_steer, &pid_throttle, &bTwiddle, &file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
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
                    // double angle = std::stod(j[1]["steering_angle"].get<std::string>());
                    double steer_value, throttle_value;
                    /*
          			 * TODO: Calcuate steering value here, remember the steering value is
          			 * [-1, 1].
          			 * NOTE: Feel free to play around with the throttle and speed. Maybe use
          			 * another PID controller to control the speed!
          			*/
					// update error and calculate steer_value at each stepW
                    if (bTwiddle) {
                        try{
                            pid_steer.Twiddle(cte);
                        } catch (std::exception& e) {
                            std::cout<< e.what() <<std::endl;
                        }
                    } else {
                        pid_steer.UpdateError(cte);
                    }
                    
          			steer_value = pid_steer.GetSteeringValue();
					if (steer_value > 1) {
                        steer_value = 1;
                    } else if (steer_value < -1) {
                        steer_value = - 1;
                    }
					
					// update error and calculate throttle_value at each step
                    pid_throttle.UpdateError(fabs(cte));
					 
					// the 0.6 throttle bias is an arbitrary number I found to be working well
                    throttle_value = 0.6 + pid_throttle.GetSteeringValue();

					// Keep the speed under 80 km/h for more realistic scenario
                    if (speed > 50) {
                        throttle_value = 0.0;
                    }
					
                    // DEBUG
                    // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
                    #if WRITE_TO_FILE
                    file << cte << "," << steer_value << std::endl;
                    #endif
                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;
                    // msgJson["throttle"] = 0.3;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // std::cout << msg << std::endl;
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
#if WRITE_TO_FILE
    file.close();
#endif
}
