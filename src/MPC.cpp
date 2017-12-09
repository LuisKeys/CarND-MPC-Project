#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

# define PI           3.14159265358979323846  /* pi */

// TODO: Set the timestep length and duration
size_t N = 15;
double delta_ang_amp = 25.0;
double dt = 0.08;
double ref_v = 50;

//Vars offset values for different state and control values
int x_start = 0; // Position x offset
int y_start = x_start + N; // Position y offset
int psi_start = y_start + N; // Orientation angle offset
int v_start = psi_start + N; // Speed module offset
int cte_start = v_start + N; // Cross track error offset
int epsi_start = cte_start + N; // Orientation error offset
int delta_start = epsi_start + N; // Steering offset
int alpha_start = delta_start + N - 1; // Accelerator and break position offset

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

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // Cost coefficients
    double cte_coff = 1000.0;
    double epsi_coff = 1000.0;
    double v_coff = 1.0;
    double delta_coff = 30.0;
    double alpha_coff = 30.0;
    double diff_delta_coff = 1000.0;
    double diff_alpha_coff = 400.0;

    // Initialize fg[0] to 0, where the cost value will be stored
    fg[0] = 0;
    // Contribution to Cost based on ref state
    for (uint t = 0; t < N; t++) {
      fg[0] += cte_coff * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_coff * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_coff * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }    

    // Minimize the use of actuators.
    for (uint t = 0; t < N - 1; t++) {
      fg[0] += delta_coff * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += alpha_coff * CppAD::pow(vars[alpha_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (uint t = 0; t < N - 2; t++) {
      fg[0] += diff_delta_coff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += diff_alpha_coff * CppAD::pow(vars[alpha_start + t + 1] - vars[alpha_start + t], 2);
    }

    // Copy initial state to fg[]
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Calculate all the rest of fg[]
    for (uint t = 1; t < N; t++) {      
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      int t_prev = t - 1;
      AD<double> x0 = vars[x_start + t_prev];
      AD<double> y0 = vars[y_start + t_prev];
      AD<double> psi0 = vars[psi_start + t_prev];
      AD<double> v0 = vars[v_start + t_prev];
      AD<double> cte0 = vars[cte_start + t_prev];
      AD<double> epsi0 = vars[epsi_start + t_prev];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t_prev];
      AD<double> a0 = vars[alpha_start + t_prev];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1]);

      // Model equations:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
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
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
  //Number of state elements is 6 (x, y, psi, v, cte and epsi)
  //Number of controls is 2 (delta and a)
  size_t n_vars = 6 * N + 2 * (N - 1);
  // TODO: Set the number of constraints
  size_t n_constraints = 6 * N;

  Dvector vars(n_vars);
  //Initial state copied to individual variables
  double x = state[0]; //position x
  double y = state[1]; //position y
  double psi = state[2]; //orientation angle
  double v = state[3]; //speed module
  double cte = state[4]; //Steering
  double epsi = state[5]; //accelerator and break position

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  for (uint i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }

  // Copy initial state into vars
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // TODO: Set lower and upper limits for variables.
  for (uint i = 0; i < n_vars; i++) {
    vars_lowerbound[i] = -100000.0;
    vars_upperbound[i] = 100000.0;
  }

  //Set bounds for delta
  double min_delta = -delta_ang_amp * PI / 180.0;
  double max_delta = delta_ang_amp * PI / 180.0;

  for (int i = delta_start; i < alpha_start; i++) {
    vars_lowerbound[i] = min_delta; //min value expressed in radians
    vars_upperbound[i] = max_delta; //max value expressed in radians
  }

  //Set bounds for a (-1 is full brake, 1 is full throttle)
  for (uint i = alpha_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }

  // Copy initial state for lower and upper bounds of constraints
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
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> solved_values;

  solved_values.push_back(solution.x[delta_start]);
  solved_values.push_back(solution.x[alpha_start]);

  int i_next = 0;
  for (uint i = 0; i < N-1; i++) {
    i_next = i + 1;
    solved_values.push_back(solution.x[x_start + i_next]);
    solved_values.push_back(solution.x[y_start + i_next]);
  }

  return solved_values;
}
