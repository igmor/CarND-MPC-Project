---

**MPC Project**

### Writeup / README

[//]: # (Image References)

[image1]: ./images/kinematic_model.png "Kinematic Model"

### Discussion

The state vector in MPC model has positions x, y, velocity and orientation angle
as well as cross track error and orientation error.

MPC model consists of the following updates:


![alt text][image1]

with the following constraints for orientation and acceleration:
```[-1.0; 1.0]```

Reference state cost model in this MPC implementatin looks like this:
```
    // Reference State Cost
    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 5*CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 30*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 3000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 5*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
and is roughly composed of the following components:
 - cross track and orientation errors
 - actuator's impact
 - relative cost of consequtive actuators updates.

I experimentally found cost values that worked fine in this particular implementation:
30 for stearing angle actuation impact, 3000 for sequential stearing angle update,
5 for orientation error's impact to steer the car into a direction of driving and
5 for sequential accelration update.

I also set hyperparameters N and dt to 10 and 0.1. dt Needs to be frequent enough to accomodate
for timely actuators changes that would be adequate for steering the car in the right direction
and take into account turns and in general sense any envoromental changes. N needs to be long enough
to be able to collect enough samples to reasonably predict future car trajectory and at the same
time not to be too computentionally costly as MPC model scales with the number of parameters in the model
which is N.

Before sending a state vector to a solver I also adjusted it for 100ms latency:
```
          v*=0.44704;
          psi = delta;
          px = v*cos(psi)*LATENCY; 
          py = v*sin(psi)*LATENCY;
          psi = psi + v/Lf*delta*LATENCY;
          cte = cte + v*sin(epsi)*LATENCY;
          epsi = epsi + v/Lf*delta*LATENCY;
          v = v + a*LATENCY;

          Eigen::VectorXd state(6);
          state << px, py, psi, v, cte, epsi;
```

Car's waypoint went through preprocessing step as well and were converted from global map
into a car's coordinates using rotational transformation:
```
          for (int i = 0; i < ptsx.size(); i++)
	  {
              double sx = ptsx[i] - px;
              double sy = ptsy[i] - py;

              ptsx[i] = sx*cos(0-psi) - sy*sin(0-psi);
              ptsy[i] = sx*sin(0-psi) + sy*cos(0-psi);				  
	  }
```
That helped to improve stability of a trajectory and get maximum speed up to 72 mph.
