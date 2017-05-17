#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;


  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // It seems that ptsx and ptsy contains the nearest way points (up to 6)
          // ahead of current vehicle position (px, py).
          // So it makes sense to transform from global corrdinates 
          // to vehicle coordinates, for two reasons:
          //  1. the polynomial fit of waypoints is expected to be more accurate
          // in the vehicle coordinates, as in most cases the road is just linear
          //  2. it helps the optimizer find optimal solutions more easily, because
          // the optimal solutions won't be too far away from initials (all zeros).

          // It turns out switching to the vehicle coordnates makes a huge
          // improvement in performance, although I am not sure whether it is
          // still possible to come up with a good solution with original coordinates.

          // transform map coordinates to car coordinates
          // (1) centerize to vehicle position
          // (2) rotate -psi to align with vehichle y axis
          vector<double> vehicle_xs(ptsx.size());
          vector<double> vehicle_ys(ptsy.size());
          for (auto i = 0; i < ptsx.size(); ++i) {
            // centered
            double centered_x = ptsx[i] - px;
            double centered_y = ptsy[i] - py;
            // rotated
            vehicle_xs[i] = centered_x * cos(-psi) - centered_y * sin(-psi);
            vehicle_ys[i] = centered_x * sin(-psi) + centered_y * cos(-psi);
          }
          

          // estimate the track as a 3rd polynomial
          Eigen::Map<Eigen::VectorXd> xs(&vehicle_xs[0], vehicle_xs.size());
          Eigen::Map<Eigen::VectorXd> ys(&vehicle_ys[0], vehicle_ys.size());
          auto coeffs = polyfit(xs, ys, 3);

          // From now on, vehicle position also transformed to
          /*
          px = 0;
          py = 0;
          psi = 0;
          */

          // estimate error - the derivative is actually
          // linear when px = 0
          double cte = polyeval(coeffs, 0) /*- py*/;
          double epsi = atan(coeffs[1]) /*- psi*/;
          
          /*
          * TODO: Calculate steeering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          // now the vehicle state will always be in (x=0, y=0) with psi=0
          Eigen::VectorXd state(6);
          state << /*px=*/0, /*py=*/0, /*psi=*/0, v, cte, epsi;

          auto actuator = mpc.Solve(state, coeffs);
          double steer_value = -actuator[0];
          double throttle_value = actuator[1];

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
