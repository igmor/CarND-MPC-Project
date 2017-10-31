---

**MPC Project**

### Writeup / README

[//]: # (Image References)

[image1]: ./images/kinematic_model.png "Kinematic Model"

### Discussion

The state vector in MPC model has positios x and y, velocity and orientation angle
as well as cross track error and orientation error.

MPC model consists of the following updates:
![alt text][image1]

with the following constraints for orientation and acceleration:
[-1.0; 1.0]

Reference state cost model in this MPC implementatin looks like this:
```
    // Reference State Cost
    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += 5*CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 5000*CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 100*CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 10000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 5*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
and is roughly composed of the following components:
 - cross track and orientation errors
 - actuator's impact
 - relative cost of consequtive actuators updates.

I experiemntally found cost values that worked fine in this particular implementation:
5 for velocity reference, 5000 for stearing angle actuation impact, 100 for accelration
impact, 10000 for sequential stearing angle updates, 5 for sequential accelaration updates.

I also set hyperparameters N and dt to 25 and 0.03. dt Needs to be frequent enough to accomodate
for timely actuators changes that would be adequate for steering the car in the right direction
and take into account turns and in general sense any envoromental changes. N needs to be long enough
to be able to collect enough samples to reasonably predict future car trajectory and at the same
time not to be too computentionally costly as MPC model scales with the number of parameters in the model
which is N.


---

