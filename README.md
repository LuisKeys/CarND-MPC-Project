# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Model description:
1) The model
MPC is based on predicting the car position in the next second or so,
comparing the predictied path with a waypoints based path 
(calculated based on location and path planing) and using a cost function 
to penalize different variables involved in the model. A library (cppad) is used to 
minimize this cost function (local minimums for non linear function problem),
a similar concept that is being used in Machine Learning to train algorithms.
The following terms were added to the cost function in the selected model:
a) CTE
b) PSI error
c) Velocity
d) steering actuator
e) throttle actuator
f) The product of steering actuator and speed
g) steering actuator differential
h) throttle actuator differential

The cost function is a simple addition of squares of the parameters (as CTE for example)
multiplied by a coefficient as follows:


    // Initialize fg[0] to 0, where the cost value will be stored
    fg[0] = 0;
    // Contribution to Cost based on ref state
    for (uint t = 0; t < N; t++) {
      fg[0] += cte_coeff * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += epsi_coeff * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += v_coeff * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }    

    // Minimize the use of actuators.
    for (uint t = 0; t < N - 1; t++) {
      fg[0] += delta_coeff * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += alpha_coeff * CppAD::pow(vars[alpha_start + t], 2);
      fg[0] += delta_coeff * alpha_coeff *CppAD::pow(vars[delta_start + t] * vars[v_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (uint t = 0; t < N - 2; t++) {
      fg[0] += diff_delta_coeff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += diff_alpha_coeff * CppAD::pow(vars[alpha_start + t + 1] - vars[alpha_start + t], 2);
    }

The beauty of the cost function is its simplicity of implementation.

The following coefficients were defined (and tuned) to make the 
cost function more sensible (penalizing more) certain terms over others:

 // Cost coefficients
    double cte_coeff = 2000.0;

    double epsi_coeff = 350.0;

    double v_coeff = 4.0;

    double delta_coeff = 15.0;

    double alpha_coeff = 100.0;

    double diff_delta_coeff = 1000.0;

    double diff_alpha_coeff = 10.0;

2) Timestep Length and Elapsed Duration (N & dt)

Initially a big delta and small N was selected thinking on performance, 
but the model was quite unestable, and din't work at all. Then I tried 
with the opposite, and tiny dt and a large number of steps and it worked
but I noticed that increasing the dt a bit (twice the value) 
and decreasing N to a half the result was the same, and the model was less
expensive in terms of proccessing (the library needs to calculate all the permutations
of the N values, hence large valules may be completelly not good at alls in terms of performance).

Finally I seleted the following values:
size_t N = 20;
double dt = 0.04;

3) Polynomial Fitting and MPC Preprocessing
Regarding the preprocessing I simply transformed the wayponts world based coordinates 
to car origin, in order to simplify all the calculations as follows:

  // Reference waypoints to car origin
  int num_waypts = ptsx.size();
  auto ptsx_transf = Eigen::VectorXd(num_waypts);
  auto ptsy_transf = Eigen::VectorXd(num_waypts);
  for (uint i = 0; i < ptsx.size(); i++) {
    x_car_ref = ptsx[i] - px;
    y_car_ref = ptsy[i] - py;
    cos_min_psi = cos(-psi);
    sin_min_psi = sin(-psi);
    ptsx_transf[i] = x_car_ref * cos_min_psi - y_car_ref * sin_min_psi;
    ptsy_transf[i] = x_car_ref * sin_min_psi + y_car_ref * cos_min_psi;
  }

Then fitted the points to a third order polygon as follows:

    Eigen::VectorXd coeffs = polyfit(ptsx_transf, ptsy_transf, 3);

4) Model Predictive Control with Latency

Finally latency was included in the model (100 ms) with a simple predictive
model for 2 of the 3 variables, as follows (note that I assumed that the velocity 
is constant, which is not true, but for such an small perdio of time is a good 
approximation and simplify the model without considering the acceleration 
which may be calculated as an approx function of throttle position):

    double x_predic = v * cos(psi) * latency;

    double y_predic = v * sin(psi) * latency;

    double psi_predic = v * steer_value * latency / mpc.GetLf();

    double v_predic = v;

    double cte_predic = cte + (v * sin(epsi) * latency);

    double epsi_predic = epsi - (v * atan(coeffs[1]) * latency / mpc.GetLf());

    Eigen::VectorXd state(6);

    state[0] = x_predic;

    state[1] = y_predic;

    state[2] = psi_predic;

    state[3] = v_predic;

    state[4] = cte_predic;
    
    state[5] = epsi_predic;

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
