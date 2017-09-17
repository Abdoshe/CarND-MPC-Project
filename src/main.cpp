#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
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

/**
 Returns a tuple representing the translated x and y of a point (first and second argument) into the car coordinates
 */
tuple<double, double> transform_to_car_coordinates(double point_in_global_x, double point_in_global_y, double car_x, double car_y, double car_psi) {
  double shift_x = point_in_global_x - car_x;
  double shift_y = point_in_global_y - car_y;
  
  double final_x = (shift_x * cos(-car_psi) - shift_y * sin(-car_psi));
  double final_y = (shift_x * sin(-car_psi) + shift_y * cos(-car_psi));
  
  return {final_x, final_y};
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  
  const double Lf = 2.67;
  
  h.onMessage([&mpc, Lf](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          //6 next waypoints
          vector<double> waypoints_global_x = j[1]["ptsx"];
          vector<double> waypoints_global_y = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"]; //Radians
          double v = j[1]["speed"];
          double steer = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];
          
          //Transform to car coordinates
          vector<double> waypoints_car_x;
          vector<double> waypoints_car_y;
          
          for (int i = 0; i < waypoints_global_x.size(); i++) {
            tuple<double, double> new_coordinates = transform_to_car_coordinates(waypoints_global_x[i], waypoints_global_y[i], px, py, psi);
            waypoints_car_x.push_back(get<0>(new_coordinates));
            waypoints_car_y.push_back(get<1>(new_coordinates));
          }
 
          double* ptrx = &waypoints_car_x[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
          
          double* ptry = &waypoints_car_y[0];
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);
          
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
          
          double latency = 0.1;
          
          //Predict steer rate and psi after latency
          double pred_v = v + throttle * latency;
          double steer_rate = steer * (deg2rad(25));
          double pred_psi = -pred_v * latency * steer_rate / 2;
          
          double displacement_x = pred_v * latency * cos(pred_psi);
          double displacement_y = pred_v * latency * sin(pred_psi);
          
          cout << displacement_x << ", " << displacement_y << endl;
          
          double pred_global_psi = psi + pred_psi;
          double pred_global_x = px + displacement_x * cos(pred_global_psi) - displacement_y * sin(pred_global_psi);
          double pred_global_y = py + displacement_x * sin(pred_global_psi) + displacement_y * cos(pred_global_psi);
          
          tuple<double, double> new_car_coordinates = transform_to_car_coordinates(pred_global_x, pred_global_y, px, py, psi);
          double pred_x = get<0>(new_car_coordinates);
          double pred_y = get<1>(new_car_coordinates);
          
          const double pred_cte = polyeval(coeffs, pred_x);
          const double pred_epsi = pred_psi - atan(coeffs[1] + 2 * pred_x * coeffs[2] + 3 * coeffs[3] * pow(pred_x, 2));
          
          Eigen::VectorXd state(6);
          state << pred_x, pred_y, pred_psi, pred_v, pred_cte, pred_epsi;
          
          //This is the resulted prediction based on a state without latency
          auto pred_result = mpc.Solve(state, coeffs);

          json msgJson;

          //Display the MPC predicted trajectory 
          vector<double> predicted_x_vals;
          vector<double> predicted_y_vals;

          for (int i = 2; i < pred_result.size(); i++) {
            if (i % 2 == 0) {
              predicted_x_vals.push_back(pred_result[i]);
            }
            else {
              predicted_y_vals.push_back(pred_result[i]);
            }
          }
          
          //Display the waypoints/reference line
          vector<double> waypoints_x_vals;
          vector<double> waypoints_y_vals;
          
          //Iterate from 1 to 6, both inclusive
          const double steps_inc = 3;
          const double number_of_steps = 20;
          for (int i = 1; i < number_of_steps; i++) {
            const double step = steps_inc * i;
            waypoints_x_vals.push_back(step + displacement_x);
            waypoints_y_vals.push_back(polyeval(coeffs, step) + displacement_y);
          }

          double steering_angle = pred_result[0] / (deg2rad(25 * Lf));
          msgJson["steering_angle"] = steering_angle;
          msgJson["throttle"] = pred_result[1];

          msgJson["mpc_x"] = predicted_x_vals;
          msgJson["mpc_y"] = predicted_y_vals;
          
          msgJson["next_x"] = waypoints_x_vals;
          msgJson["next_y"] = waypoints_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

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
