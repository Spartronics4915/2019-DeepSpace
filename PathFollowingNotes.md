# Path Planning Notes

<!-- TOC depthFrom:2 depthTo:6 updateOnSave:true withLinks:true -->

- [Intro](#intro)

<!-- /TOC -->


## Intro

Path


# See Also

[Ramesete Nonlinear Controller (section 12.1 of state-space-guide)](https://github.com/calcmogul/state-space-guide/blob/master/modern-control-theory/nonlinear-control/control-law-for-nonholonomic-wheeled-vehicle.tex)

[254 2018 reveal thread](
https://www.chiefdelphi.com/t/team-254-presents-lockdown-technical-binder-2018/166515/69)

```text
The optimization function is simple iterated gradient descent 1. At
iteration N+1, we compute the partial derivatives on rate of change
of curvature w.r.t. arc length for the curve at iteration N, then
take a step in the direction that minimizes the magnitude of the
derivative. Keep iterating until you hit a termination condition (max
number of iterations, or the improvement between the last two iterations
is too small).

For following, we did not use pure pursuit except at our first event. After AZ North we finished our nonlinear feedback controller 3 and used that going forward. This controller generates velocity setpoints based on the current pose of the robot and feedforward velocity and acceleration terms from our trajectory.

Our feedforward generation techniques used a dynamic model 2 of a skid-steer drivetrain to generate voltage setpoints that account for torques due to chassis linear acceleration, chassis angular acceleration, and a variety of friction effects.
```
