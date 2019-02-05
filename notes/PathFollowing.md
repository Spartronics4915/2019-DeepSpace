# Path Planning and Following

<!-- TOC depthFrom:2 depthTo:6 updateOnSave:true withLinks:true -->

- [Introduction](#introduction)
- [From Waypoints to Motor Voltages](#from-waypoints-to-motor-voltages)
- [The Rubber Hits the Road](#the-rubber-hits-the-road)
    - [DriveMotionPlanner.java](#drivemotionplannerjava)
    - [TrajectoryGenerator.java](#trajectorygeneratorjava)
    - [Trajectory.java](#trajectoryjava)

<!-- /TOC -->


## Introduction

Path-following has traditionally been an advanced capability for FRC teams.
This is because the time investment required to obtain accurate results is
quite high.  Here are some of the commitments required to master path following:

* Trajectory authoring
* Robot state estimation and odometry
* Closed-loop trajectory following controller
* [Drivetrain characterization](DriveCharacterization.md)
* PID tuning
    * for path following controller
    * for motor controller

With the advent of [Jaci's Pathfinder](https://github.com/JacisNonsense/Pathfinder)
and [Team 254's DriveMotionPlanner](https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/planners/DriveMotionPlanner.java),
the cost of entry has been significantly reduced and it has become relatively
common for mid and top tier teams to deploy some variant of this technology.

We adopted Team254's codebase primarily due to the leg-up it gave us in
obtaining this capability.  Herein we'll go into the theory of operation
and some of the nitty-gritty details behind their path-follower implementation.

## From Waypoints to Motor Voltages

The idea behind a path follower is actually quite simple.  In order to
describe the trajectory of a robot, we need to describe a sequence of
places (`Waypoints`) on the field that we'd like our robot to visit.  If timing
isn't a concern, we can easily imagine an implementation:

1. orient robot in the direction of the next waypoint
2. travel accurately the distance to the next waypoint
3. if not at final waypoint: goto 1
4. stop

The key problems with this approach are it's lack of speed and "grace".
Breaking the path into linear segments (a-la connect-the-dots) would:

* be slow because:
    * we need to converge two distinct setpoints for each waypoint
    * net to run at slower speeds since it stops and starts at each waypoint.
* lack grace because:
    * wouldn't cut corners, only goes straight, then turn, the goes straight

To resolve these issues we need to


## The Rubber Hits the Road

### DriveMotionPlanner.java

``` java
TrajectoryIterator<TimedState<Pose2dWithCurvature>> mCurrentTrajectory;
mCurrentTrajectory = TrajectoryGenerator.generate("a named trajectory");
```

### TrajectoryGenerator.java

```java
class Pose2dWithCurvature
{
    // Pose2d
    Translation2d xlate;
    Rotation2d rotate;
    // with curvature
    double curvature_;
    double dcurvature_ds_;
}
```

### Trajectory.java


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
