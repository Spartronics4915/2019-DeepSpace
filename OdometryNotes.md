# Odometry Notes

<!-- TOC depthFrom:2 depthTo:6 orderedList:false updateOnSave:true withLinks:true -->

- [Intro](#intro)
- [Sensors](#sensors)
    - [Wheel Encoder](#wheel-encoder)
    - [IMU (Gyro, Accelerometer, Magnetometer)](#imu-gyro-accelerometer-magnetometer)
    - [Lidar](#lidar)
    - [Camera](#camera)
- [Software Components for Odometry](#software-components-for-odometry)
    - [Pose2d](#pose2d)
    - [Twist2d](#twist2d)
        - [Notes On Curvature](#notes-on-curvature)
    - [RobotStateEstimator](#robotstateestimator)
    - [RobotState](#robotstate)
- [Integrating Robot Pose](#integrating-robot-pose)

<!-- /TOC -->

## Intro

Wikipedia [tells us](https://en.wikipedia.org/wiki/Odometry) that Odometry is
the use of data from motion sensors to estimate change in robot position over
time.  An accurate understanding of our robot position _and orientation_ on the
field can enable all sorts of robot capabilities that depend on
interacting with elements on the field at known (and static) locations.
If you want to shoot balls into a boiler, drop off gears at the airship
or drop a cube into the switch on the opposite side of the field, accurate
odometry and careful path-planning are the keys to success.

Most frc robot tracking solutions start with a represention of the robot's
current _pose_ or _state_.  These terms convey the notion the accurate 
awareness of the _position_ of the robot (or robot joints) is central to the
quest for effective robot control.  These terms also suggest that position
alone isn't sufficient to solve most targeting and navigation problems. We need 
to minimally understand the position _and orientation_ of robot elements relative
to the world (ie field) and more generally to other robot components. Additional
understanding of the robot's current velocity and acceleration (along a
straight line, and in terms of its turning radius) helps us to calculate
the amount of force to apply to motors in order to achieve a specific goal with
speed and grace.

In a perfect world, the force that we deliver to our motors would have a direct
correspondance with the motion of the robot.  In that case the problem of
odometry would be rather trivial.  We'd simply track all the commands that
we issue to to our motors over time and compute the robot's currentstate.
In the real world, this correspondance between request and response is
approximate and the accuracy of our tracking decreases with time. To
improve tracking accuracy we must employ sensors. A variety of sensor types
can be employed to measure a variety of physical conditions and these
measurements can be _fused_ to improve our robot state estimate. Our challenge
as robot designers is to select an efficient/appropriate combination of
actuators, sensors, calibration techniques and software algorithms to deliver
the level of speed an accuracy we need to accomplish the FRC tasks for
a given challenge.  Fortunately for us, the FRC community is extremely
collaborative and the wealth of solutions and experiences produced by the
community is readily available.  We've relied on the Team254's code base
and their prior robot designs to learn how to implement an odometry
solution so we'll use their software classes as a guide to understanding
both our current codebase and the details of an actual odometry implementation.

## Sensors

### Wheel Encoder

### IMU (Gyro, Accelerometer, Magnetometer)

[Wikipedia IMU](https://en.wikipedia.org/wiki/Inertial_measurement_unit)

### Lidar
### Camera

## Software Components for Odometry

### Pose2d

``` java
class Pose2d
{
    public Translate2d position;
    public Rotation2d rotation;
}
```

Pose2d is used to capture the static state of the robot at a particular
instant in time.  The position is represented as an x,y pair measured relative
to a particular coordinate frame.  Our primary goal is to understand the robot's
pose on the FRC field and so position is usually in the range of field
coordinates.  We currently use inches as our basic field measurement unit
and so position coordinates usually fall in the range: 324x648in (27x54ft). 

The _orientation_ of the robot is captured with a single angle and we can
think of this angle in degrees or radians relative to our the given coordinate
frame.  If we treat x as the width of the field and y as the length, then a
robot (r1) oriented at 0 degrees will point toward the "lower" side of the field 
and a robot (r2) oriented at 90 degrees will point toward the "right" end.

                            x/y ---------------->
                            |
                            |   r1      r2--
                            |    |
                            |
                            v


### Twist2d

``` java
class Twist2d
{
    public double dx;
    public double dy;
    public double dtheta;
}
```

Twist2d can be used to represent the difference between two poses or, inversely,
the way to move from one pose to another. If we account for the time between
the two pose samples, we can interpret Twist2d as a combination of linear and
angular velocity. In other words its the rate of change of position _and_
orientation. In mathematical terms, the Twist2d includes both _tangent_ and 
_curvature_ information.

Consider these two robot poses below. 

                            x/y ---------------->
                            |
                            |   p1
                            |    |           p2--
                            |
                            v

One way to compute the distance between the two poses would be to apply
the standard Euclidian distance formula to the x,y coordinates of both
poses.

``` java
    double distance(Pose2d p1, Pose2d p2)
    {
        return r1.position.distance(r2.position);
    }
```

If we move the robot along the direction of dx,dy from p1 by our computed
distance our robot would end up at the position of p2 but with the wrong
orientation.  This is where the Twist2d gets interesting.  If we think
of a Twist2d representing a movement along an arc at constant curvature
and velocity, we can use ideas from [differential calculus](https://en.wikipedia.org/wiki/Differential_calculus) to create a new Pose2d
from a Twist2d and visa versa.  In essence this is equivalent to drawing
a portion of a circle (ie _arc_) that connects the two poses.

#### Notes On Curvature

The design of Twist2d is closely related to the dynamics of a differential
drive robot.  When we command different rpm to left and right motors
we end up driving in a circle whose radius is determined by the difference
between left and right rpm. The speed we drive is approximated by the
average of the two rpm. So the circular arc is a fundamental atomic
unit of travel for a differential drive robot.  We can decompose a
complex robot trajectory into a series of small, connected arcs by 
selecting a new Twist2d on each timestep.


From [Mathworld](http://mathworld.wolfram.com/Curvature.html):

``` text
The curvature of a two-dimensional curve is related to the radius of curvature
of the curve's osculating circle. Consider a circle specified parametrically by:

   x = r*cos(t)
   y = r*sin(t)

which is tangent to the curve at a given point.

The curvature turns out to be directly related to the radius of the circle:

   curvature = 1/r
```

So the larger the curvature, the smaller the radius (the tighter the turn).  
Also, as our circle gets large the curvature approaches 0 (ie a line).
[This link](http://mathworld.wolfram.com/OsculatingCircle.html) shows how
the osculating circle changes at points along a trajectory.

Here's the Twist2d implementation. Since we're dealing with the tangent(dx,dy)
and angular velocity (dtheta), the computation takes on the form of eq. 4,
[here](http://mathworld.wolfram.com/Curvature.html).

``` java
public double curvature()
{
    if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
        return 0.0;
    return dtheta / norm();
}
```

### RobotStateEstimator

### RobotState

A Twist can be used to represent a difference between two poses, a velocity,
an acceleration, etc.

## Integrating Robot Pose


