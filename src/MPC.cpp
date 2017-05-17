#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration

///////////////// How to choose N and dt values /////////////////////
// 1. If the waypoint model is accurate enough, using a big 'N'
// will help "foresee" the road further and thus plan driving in
// a smoother way. 
// However, since most of time the polynomial model
// of waypoints is just a local approximation, a big N might 
// increases the computation cost and decreases the accuracy - 
// because the model is not accurate for 'N' too far away from 0.

// 2. Using a smaller 'dt' would help find actuators parameters
// that are optimal to "local approximations" that are not too
// far away from the current position.
// However, if 'dt' is too small and 'N' is also small, the driving
// model won't be able to see "far" enough so it might have difficulties
// at turning. 
// Similiarly, using 'dt' should not be too small compared to latency.

// In practice, 'N' or/and 'dt' should be increased when the reference
// speed is increased - this helps look further ahead on the road, and
// make the driving smoother.
size_t N = 20;
double dt = 0.2;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// helper variables derived from quizz code
// https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp
double ref_cte = 0; // target cross track error
double ref_epsi = 0; // target oritent error
double ref_v = 80; // target speed

// index for variables and condtions
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // set objective function as fg[0] - accumulative on all predictions

    // different objectives
    AD<double> cte_loss = 0,
               epsi_loss = 0,
               v_loss = 0,
               delta_loss = 0,
               a_loss = 0,
               delta_smooth = 0,
               a_smooth = 0;
    for (auto i = 0; i < N; ++i) {
      cte_loss += CppAD::pow(vars[cte_start+i] - ref_cte, 2);
      epsi_loss += CppAD::pow(vars[epsi_start+i] - ref_epsi, 2);
      v_loss += CppAD::pow(vars[v_start+i] - ref_v, 2);
      if (i < N - 1) {
        delta_loss += CppAD::pow(vars[delta_start+i], 2);
        a_loss += CppAD::pow(vars[a_start+i], 2);
      }
      if (i < N - 2) {
        delta_smooth += CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
        a_smooth += CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
      }
    }
    //////////////// objective as weighted sum ///////////////////////
    // When refence speed is set, tuning of loss weights is based 
    // on trial-and-error. But there are still some intuitions behind
    // different components.

    fg[0] =   1000 * cte_loss          /*staying on the track by miminizing cte and epsi*/
            + 1000 * epsi_loss         /*these should be the most important things in driving*/
            + 1 * v_loss              /*release v constraint when driving at high speed*/
            + 10 * delta_loss         /*heavier delta_loss makes driving smoother, same effect as restrcting delta ranges*/
            + 50 * a_loss             /*restrict acceleration helps make driving smoother*/
            + 100 * delta_smooth      /*smaller steering (delta) change means less wiggly*/
            + 50 * a_smooth;          /*try to keep the driving speed constant*/


    // set constraints for each of 6 states, for every prediction point
    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];

      AD<double> f0 = coeffs[0] + coeffs[1]*x0 + coeffs[2]*pow(x0,2) + coeffs[3]*pow(x0,3);
      AD<double> psides0 = CppAD::atan(coeffs[1] + coeffs[2]*2*x0 + coeffs[3]*3*x0*x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9

  // 6 states and 2 actuators
  size_t n_vars = N * 6 + (N-1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = N * 6;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (auto i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (auto i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (auto i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  // keep the initial state fixed
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  // std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  //
  // Check some of the solution values
  //
  return {
    /*steering=*/solution.x[delta_start],
    /*throttle=*/solution.x[a_start]
  };
}
