# Odometry Notes

<!-- TOC depthFrom:2 depthTo:6 orderedList:false updateOnSave:true withLinks:true -->

- [Intro](#intro)
- [Software Components for Odometry](#software-components-for-odometry)
    - [Pose2d](#pose2d)
    - [Twist2d](#twist2d)
        - [Notes On Curvature](#notes-on-curvature)
    - [RobotStateMap (oh, the places we've been)](#robotstatemap-oh-the-places-weve-been)
    - [RobotStateEstimator (pulling it all together)](#robotstateestimator-pulling-it-all-together)
- [Path Following (the killer app for odometry)](#path-following-the-killer-app-for-odometry)
- [Notes on Sensors for Odometry](#notes-on-sensors-for-odometry)
    - [Wheel Encoder](#wheel-encoder)
    - [IMU (Gyro, Accelerometer, Magnetometer)](#imu-gyro-accelerometer-magnetometer)
    - [Lidar](#lidar)
    - [Camera](#camera)

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
current _pose_ or _state_.  These terms convey the notion that accurate
awareness of the _position_ of the robot (or robot joints) is central to the
quest for effective robot control.  These terms also suggest that position
alone isn't sufficient to solve most targeting and navigation problems. We need
to minimally understand the position _and orientation_ of robot elements relative
to the world (ie field) and more generally to other robot components. Additional
understanding of the robot's mass, current velocity and acceleration (along a
straight line, and in terms of its turning radius) helps us to calculate
the amount of force to apply to motors in order to achieve a specific goal with
speed and grace.

In a perfect world, the force that we deliver to our motors would have a direct
correspondance with the motion of the robot.  In that case the problem of
odometry would be rather trivial.  We'd simply track all the commands that
we issue to our motors over time and compute the robot's current state.
In the real world, this correspondance between request and response is
approximate so the accuracy of our tracking decreases with time. To
improve tracking accuracy we must employ sensors. A variety of sensor types
can be employed to measure a variety of physical conditions and these
measurements can be _fused_ to improve our robot state estimate. Our challenge
as robot designers is to select an efficient/appropriate combination of
motors, sensors, calibration techniques and software algorithms to deliver
the level of speed and accuracy we need to accomplish the FRC tasks for
a given challenge.  Fortunately for us, the FRC community is extremely
collaborative and the wealth of solutions and experiences produced by the
community is readily available.  We've relied on the Team254's code base
and their prior robot designs to learn how to implement an odometry
solution. We'll use their software classes as a guide to understanding
both our current codebase and the details of an actual odometry implementation.

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
instant in time.  The position is represented as x,y coordinates measured
relative to a particular coordinate frame.  Our primary goal is to understand
the robot's pose on the FRC field and so position is usually in the range of
field coordinates.  We currently use inches as our basic field measurement unit
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
angular velocity. In other words it's the rate of change of position _and_
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
        return p1.position.distance(p2.position);
    }
```

If we move the robot along the direction of dx,dy from p1 by our computed
distance our robot would end up at the position of p2 but with the wrong
orientation.  This is where the Twist2d gets interesting.  If we think
of a Twist2d as representing a movement along an arc at constant curvature
and velocity, we can use ideas from [differential calculus](https://en.wikipedia.org/wiki/Differential_calculus) to create a new Pose2d
from a Twist2d and visa versa.  In essence, this is equivalent to drawing
a portion of a circle (aka _arc_) that connects the two poses.

#### Notes On Curvature

The design of Twist2d is closely related to the dynamics of a differential
drive robot.  When we command different rpm to left and right motors
we end up driving in a circle whose radius is determined by the difference
between left and right rpm. The speed we drive is approximated by the
average of the two rpm. So the circular arc is a fundamental incremental
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
Also as our circle gets large, the curvature approaches 0 (it becomes a line).
[This link](http://mathworld.wolfram.com/OsculatingCircle.html) shows how
the osculating circle changes at points along a trajectory.

Here's the Twist2d implementation.

``` java
public double curvature()
{
    if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
        return 0.0;
    return dtheta / norm();
}
```

Since we're dealing with the tangent(dx,dy) and angular velocity (dtheta),
the computation takes on the form of eq. 4, [here](http://mathworld.wolfram.com/Curvature.html).

For our purposes curvature is valuable only in the context of path-planning
which is separate a topic unto itself. In that context, curvature is a measure
of how much we're currently turning and its derivative conveys the rate at which
we're the turn amount changes.

### RobotStateMap (oh, the places we've been)

Now that we see that the combinatation Pose2d and Twist2d captures the
most salient aspects of our robot's state, we move the question of how
to keep track of the places we've been or, more accurately, the states we've
been in.  For this purpose we employ a RobotStateMap to associate a time
with a Pose2d and Twist2d.

``` java
public class RobotStateMap
{
    class State
    {
        public Pose2d pose;
        public Twist2d velocity;
        double timestamp;
    }

    void addObservation(double timestamp, Pose2d pose, Twist2d velocity);
    State get(double timestamp);
    State getLatest();
}
```

The value of this data structure is that it allows us to keep a historical
record of robot state.  This is useful for diagnosing robot behavior, visualizing
our path plans and for correlating past sensor observations to correct for
communication latency. Our implementation allows us to look up an _interpolated_
robot state at a point in the past.  As alluded to above, this interpolation
relies on some _secret sauce_ in our Pose2d implementation.

### RobotStateEstimator (pulling it all together)

Pulling it all together, the RobotStateEstimator is a special _subsystem_
in our code that feeds and grooms our RobotStateMap(s). Since we can estimate
the robot state via a number of methods we may track these multiple states
separately and later either _fuse_ their results or simply learn which approach
delivers superior results.  

``` java
class RobotStateEstimator extends Subsystem
{
    private RobotStateMap mEncoderRobotState;
    private RobotStateMap mLidarRobotState;
}
```

As with any subsystem RobotStateEstimator implements a custome _Looper_
that periodically updates its estimate of the robot state and stores
it into the RobotStateMap.

``` java
public synchronized void onLoop(double timestamp)
{
    /* two ways to measure current velocity, not sure which is better */

    /* method 1:  
     * Look at the distance traveled since last measurement, consider
     *   current gyro heading rather than our stored state
     * Divide by delta time to produce a velocity.
     */
    final double leftDist = mDrive.getLeftEncoderDistance();
    final double rightDist = mDrive.getRightEncoderDistance();
    final double leftDelta = leftDist - mLeftPrevDist;
    final double rightDelta = rightDist - mRightPrevDist;
    final Rotation2d heading = mDrive.getHeading();
    mLeftPrevDist = leftDist;
    mRightPrevDist = rightDist;
    final Twist2d velocityD = getVelocityFromDeltas(leftDelta, rightDelta, heading);

    /* method 2:
     *  Directly sample the current wheel velocities. 
     *  Convert to a central-axis velocity.
     */
    final Twist2d velocityK = Kinematics.forwardKinematics(
                                mDrive.getLeftLinearVelocity(),
                                mDrive.getRightLinearVelocity());
    final RobotStateMap.State lastState, nextState;
    lastState = mEncoderRobotState.getLatest();

    /* integrateForward: given a last state and a current velocity,
     *  estimate a new state (P2 = P1 + dPdt * dt)
     */
    nextState = Kinematics.integrateForwardKinematics(lastState, velocityD);

    /* record the new state estimate */
    mEncoderRobotState.addObservations(timestamp, nextState);
}
```

We note that `Kinematics.integrateForwardKinematics` invokes the _secret sauce_
built into our Pose2d. Since there is no multiplication by deltaTime here it
should be noted that the units of Twist2d must already embed this information.

``` java
public static Pose2d integrateForwardKinematics(Pose2d pose, Twist2d vel)
{
    return pose.transformBy(Pose2d.exp(vel));
}
```

## Path Following (the killer app for odometry)

At this point in the saga, we're able to make pretty pictures of the path
that the robot takes as we drive it around the field.  This is sorta cool
in and of itself but doesn't really deliver us any new robot capabilities.
Things start to get interesting as soon as we have an opinion about where
and when we want our robot to be.  Path following is one way to express
such an opinion and it is an essential technology in autonomous robot
navigation. Path following is the means by which we cause our robot
to navigate the field in autonomous mode and may even find applications
during teleop as a driver-assistant. The details of path following are
discussed in [PathFollowingNotes](PathFollowingNotes.md) but for our purposes
we can thing of it as an algorithm that:

1. Expresses a moment-to-moment opinion of where the robot needs to be.
2. Implements a strategy that causes the robot to incrementally correct
   its state to match the moment-to-moment state goal.

And here's where odometry comes in!  The means by which the Path Follower measures
the __error__ between expected state and measured state is simply to read our
`RobotStateEstimator`'s `RobotStateMap` and subtract it from our expected state.
Easy peasey.  This error is the input to our _feedback control system_ which
we must carefully tune to ensure reliable results.  Clearly the effectiveness
of our Path Follower is highly dependant upon the accuracy of our state
estimation and this is why its worth our time to both measure its accuracy
in real-world setting and to continue to try to improve it by exploring
alternate sensor modalities.

## Notes on Sensors for Odometry

Now that we understand the centrality of odometry to our autonomous robot
capabilities, it's clear that sensors must also be central to our success
on this front. Exploring new sensor modalities and new applications for
traditional modalities is an important activity for competitive FRC team.
This sort of exploration is especially attractive during the offseason,
where it can present a number of interesting challenges to budding
roboticists.  

Generally we can divide sensor modalities into those that
provide absolute (or _relatively_ absolute) feedback versus those that
only measure relative changes to some condition.  [GPS](https://en.wikipedia.org/wiki/Global_Positioning_System),
[magnetometers](https://en.wikipedia.org/wiki/Magnetometer) and even
thermometers provide data expressed in globally-consistent coordinates.
For odometry we're principally interested in `RobotStateMap.State`
and sadly, GPS and magnetometers suffer from inadequate accuracy and reliability
in many indoor settings. We currently rely heavily on sensors that produce
relative or indirect feedback and the important characteristic of these types
of sensors is that we must _integrate_ sensor results or _infer_ behavior
over time to obtain an estimate of the absolute state.  And anywhere we must
integrate over time we expect to accumulate approximation errors which make
our state estimates cruder and cruder as time goes on.  To grapple with this
problem, _sensor fusion_ can be used to combine the noisy results of multiple
sensors into a single combined-result that has better accuracy than any
single sensor could provide.  This is where the [Kalman Filter](https://en.wikipedia.org/wiki/Kalman_filter) or 
[Particle Filters](https://en.wikipedia.org/wiki/Particle_filter)
come into play.  Some sensors come with sensor fusion capabilities built-in
but we have yet explore these capabilities deeply (see Pigeon IMU, below).

### Wheel Encoder

Wheel encoders are perhaps the most common sensor in mobile wheel-driven
robotics. They come in both absolute and relative flavors (the former is often
referred to as a shaft encoder), but for drivetrain applications the relative,
or incremental, encoders are commonly utilized. Incremental wheel encoders can
be implemented in a variety of ways (eg optically, magnetically) and are useful
to measure the relationship between the power commanded to the motors and the
actual drivetrain output. Wheel encoders can accurately measure the velocity of
the wheels, but if distance is required it must be integrated. Odometry based on
wheel encoders suffers in accuracy each time a wheel rotation doesn't correlate
with robot motion. This happens on bumpy terrain, in tight turns and when
wheels slip or "burn rubber". Under perfect operating conditions where wheels
don't slip and paths are fairly straight, wheel encoders provide sufficient
accuracy to accomplish many autonomous tasks.  As paths get more complex
and curvy, it can be helpful to combine encoders with an IMU.

See: [US Digital](https://www.usdigital.com/products/encoders),
    [CTRE](http://www.ctr-electronics.com/srx-magnetic-encoder.html)

### IMU (Gyro, Accelerometer, Magnetometer)

IMU stands for inertial measurement unit. An IMU is designed to sense movement
and are found in all modern smartphones.  An IMU is a hybrid sensor made up
of a variety of sensor types whose results can be fused to produce a
result with increased accuracy.  Consumer grade IMUs are characterized by
the number of degrees of freedom they measure. We currently use the Pigeon
IMU and there are a few differnt models in common use within FRC. The Pigeon
is a 9-degree-of-freedom sensor and, as such, includes 3-axis accelerometer,
3-axis gyroscope, and 3-axis magnetometer.  Since IMUs are intended to sense
motion we loosely characterize them as relative sensors. The combination
of gyro and magnetometer are fused to produce reasonably reliable understanding
of rotation and the Pigeon software offers an absolute (integrated) measure of
robot heading.  This, in turn, can be combined with our wheel-encoder output
to deliver a more reliable and stable measure of relative robot motion since
it doesn't suffer from wheel-slip inaccuracies.  The accelerometer could, in
theory be integrated into our odometry implementation but currently we haven't
investigated this potential.  One of the problems with the use of the
accelerometer is that it must be calibrated in a manner that is inconvenient
for use in robot competitions.  The gyro and magnetometer must also be calibrated
but this occurs automatically during its startup.

See also:
    [Wikipedia](https://en.wikipedia.org/wiki/Inertial_measurement_unit),
    [Pigeon IMU](http://www.ctr-electronics.com/downloads/pdf/Pigeon%20IMU%20User's%20Guide.pdf)

### Lidar

Lidar is an acronym for LIght Detection And Ranging. It is the most accurate
and high-speed method for measuring distance commercially available and can
be found in a variety of consumer products including digital tape measures.
It has been used extensively in autonomous driving applications where it is
used to continuously _scan_ the environment around a vehicle. We have been
exploring the SlamTech A1 which has a scanning motor integrated with the distance
sensor.  It purports to sample up to 8000 times per second and can sense objects
up to 12m away with an accuracy of .2cm.  There are number of applications
for range sensing and so far we've been focused on odometry applications.
The basic idea is to measure all the visible object around the robot, then
compare how this "map" changes as the robot moves.  Lidar works best on
objects that reflect _diffusely_.  Since the technology is based on light
any transparent surface is appoximately invisible to lidar.  Since there is
a lot of glass/acrylic on the FRC field this represents a concern. Similarly
mirror-like reflective surfaces aren't easy to capture since the range of
angles that reflect light back to the sensor is very small (and within the
nearly perpendicular angle range).  Another challenge will be to figure out
where we can mount a lidar since it needs a clear line of sight in
"most directions".

See also:
    [Wikipedia](https://en.wikipedia.org/wiki/Lidar),
    [SlamTech](http://www.slamtec.com/en/lidar/A1),
    [RobotShop](https://www.robotshop.com/en/compare-scanning-laser-rangefinders.html)

### Camera

* optical flow
* stereo cross-correlation (Microsoft Kinect)
