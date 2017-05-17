# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Simulation

The code works with original project simulator that can be downloaded from [here](https://github.com/udacity/CarND-MPC-Project/releases)

Somehow it doesn't work with the [enhanced version of simulator](https://github.com/udacity/self-driving-car-sim/releases) - I haven't fingered out why.

## Model Description

- Most of the model part is from the [in-class quizz](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/mpc_to_line/solution/MPC.cpp) from udacity.
- The state is encoded as (x, y, psi, v, cte, epsi), where (x, y) is vehicle current location,
psi is the current vehicle bearing, and v is the current speed. cte and epsi is the cross-track-error (on y axis) and bearing difference with reference waypoints.
- There are two actuators - steering and throttle, both are in [-1, 1]
- The vehicle model can be described as 
```c++
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```

## How to choose `N` and `dt` values 
- 1. If the waypoint model is accurate enough, using a big 'N' will help "foresee" the road further and thus plan driving in a smoother way. 
    However, since most of time the polynomial model of waypoints is just a local approximation, a big N might increases the computation cost and decreases the accuracy - because the model is not accurate for 'N' too far away from 0.
- 2. Using a smaller 'dt' would help find actuators parameters that are optimal to "local approximations" that are not too far away from the current position. However, if 'dt' is too small and 'N' is also small, the driving model won't be able to see "far" enough so it might have difficulties at turning. 
    Similiarly, using 'dt' should not be too small compared to latency.In practice, 'N' or/and 'dt' should be increased when the reference speed is increased - this helps look further ahead on the road, and make the driving smoother.
- 3 In practice, 'N' or/and 'dt' should be increased when the reference speed is increased - this helps look further ahead on the road, and make the driving smoother.

## How to fit polynomials to waypoints
- It seems that ptsx and ptsy contains the nearest way points (up to 6) ahead of current vehicle position (px, py). 
- So it makes sense to transform from global corrdinates to vehicle coordinates, for two reasons:
  - 1. the polynomial fit of waypoints is expected to be more accurate in the vehicle coordinates, as in most cases the road is just linear
  - 2. it helps the optimizer find optimal solutions more easily, because the optimal solutions won't be too far away from initials (all zeros).
- It turns out switching to the vehicle coordnates makes a huge improvement in performance, although I am not sure whether it is still possible to come up with a good solution with original coordinates.
- transform map coordinates to car coordinates
  - (1) centerize to vehicle position
  - (2) rotate -psi to align with vehichle y axis

## Deal with latency
- As mentioned above, picking the right value for `N` and `dt`, e.g., increasing `dt` to plan further ahead, helps with dealing with acutator latency
- I also use different weights for different objective components following the intuition
  - heaviest weights on `cte` and `epsi` to help vehicle stay on the track
  - relatively weak weight on speed v constraint with refence value, specially when driving at high speed, so acceleration can be used more freely
  - steering and throttle should be small 
  - relatively heavier weight on smooth steering to prevent driving from being wiggly
  - relatively heavier weight on throttle change so driving can be as constant as possible

## Results
[![MPC Simulation](https://img.youtube.com/vi/3VUIm6rpM_M/0.jpg)](https://www.youtube.com/watch?v=3VUIm6rpM_M)