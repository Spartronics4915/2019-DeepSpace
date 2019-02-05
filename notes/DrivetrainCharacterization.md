# Drivetrain Characterization and Tuning

Wherein we discuss the murky business of getting the robot to move where we
want and at the speed we want.

<!-- TOC depthFrom:2 depthTo:6 orderedList:false updateOnSave:true withLinks:true -->

- [Introduction](#introduction)
- [Drive Characterization](#drive-characterization)
    - [2-minutes of Physics](#2-minutes-of-physics)
        - [Newton's Second Law - `F = m*a`](#newtons-second-law---f--ma)
        - [Rotational Motion - `F = torque/radius`](#rotational-motion---f--torqueradius)
        - [One With the Force](#one-with-the-force)
    - [Back to Robots and Motors](#back-to-robots-and-motors)
- [Collecting the Data](#collecting-the-data)
- [The Rubber Hits the Road](#the-rubber-hits-the-road)
    - [Drive.java](#drivejava)
    - [DriveMotionPlanner](#drivemotionplanner)
    - [DifferentialDrive](#differentialdrive)
    - [DCMotorTransmission](#dcmotortransmission)
- [References](#references)
- [Misc notes from chief delphi](#misc-notes-from-chief-delphi)

<!-- /TOC -->

## Introduction

Our CANTalons can be operated in a confusing variety of modes.  Central to
our autonomous motion and path-planning is the __Velocity Control Mode__ (VCM).
The idea of this mode is that there is a feedback loop (PID) that runs on each
Talon Master whose purpose is to track a target velocity.  In order for our
robot to traverse a curved path, our path-planner's job is to compute a series
of target velocities for each wheel. It turns out, this computation is highly
dependent upon our characterization of the robot's dynamic capabilities.
Different from __Position Control Mode__, VCM relies upon a _feedforward_ term.
The idea is to help the VCM PID be more effective by commanding the motors to be
in _approximately_ the correct power configuration so that the job of
the feedback terms is merely to fine-tune the motor power levels to hone
in on our target velocities. If we select a feedforward value of 0, then
the feedback terms bear the full responsibility of achieving the target
velocity and that makes the PID value tuning process extremely difficult
to achieve.

So we must take some care is selecting values for Kf that make the PID tuning
more efficient and reliable. Since we understand that the purpose of Kf is to
provide approximately the correct amount of power to achieve a target velocity
we can think of it as Volts per Velocity.  We'll delve more into the units for
CTRE's Kf parameter below when the rubber hits the road. The CTRE Kf interface
suggests the notion that a feedforward term can apply for _any velocity_.
As we'll see below, this is not adequate for our requirements and so we must
establish a _`different value of Kf for each target velocity`_.

The question for us, then, is how do we know what values of Kf to choose and
how do we write software to deliver new Kf values "continuously".

Since modeling an FRC drivetrain mathematically is not only difficult but
probably overkill, we adopt the strategy of _characterizing_ it by measuring
its physical properties, then approximating its behavior via a small collection
formulas and constants. Once obtained, the drive characterization is used to
translate a velocity requirement into a voltage requirement and from there to a
feedforward term.

## Drive Characterization

In the simplest terms, we wish to know how many Volts are required
to drive our fully-loaded drivetrain at a particular velocity.
We can easily imagine gathering data that captures the mapping between a
particular velocity and the voltage required to attain it. It gets a little
trickier because no drivetrain is perfect and we thus anticipate that the left
voltage map will differ from the right voltage map. And of course, since the
dynamic behavior of the robot depends heavily upon mass and mass distribution,
we must hold off on final characterization efforts until we have a final robot.
As we'll see, it's even trickier still.

### 2-minutes of Physics

Before proceeding further, we must review a few fundamental physics conepts
that relate to robot motion.

#### Newton's Second Law - `F = m*a`

In order to change the motion of a thing, we must apply a collection of forces
(F). Also, the more massive the thing (m), the more force we must apply to
achieve that same change.  Finally we must only apply a force to change a
thing's velocity and this is called acceleration (a). This formula works
in outer-space where there's no friction, negligible pressure, etc.  To
make it a little more useful in earthly conditions, we can modify it slightly
to include _dissipating effects_ like this: `F = s + f*v + m*a`. This formula
approximates the physical effects of friction, air-pressure, etc by expressing
the idea that we must add some force in proportion to the current velocity
(v) just to keep moving at the same velocity.

#### Rotational Motion - `F = torque/radius`

Newton's second law is often presented in the form of _linear motion_.
In robotics, we commonly deal with wheels and electric motors. Here,
the rotational motion that occurs when we power a motor is best described
with units of angles rather than distance.  A turning motor is said to have
an _angular velocity_, often measured in rotations per minute (RPM) or
radians per second (rad/s).  As in the linear case, if we wish to change
angular velocity we must apply a force and this results in _angular acceleration_.
We have a special name, `torque`, for the force applied to achieve angular
acceleration and it's one trickiest topics in the field of physics. Even
the idea of mass is trickier and physicists use the term `Moment of Inertia`
(often referred to as `J` or `I`) to characterize the amount that the
mass within an object affects its angular motion.

#### One With the Force

One of the coolest things about physics is that it unifies seemingly diverse
phenomena under a concise mathematical framework.  For our purposes, the idea
of Force unites linear and angular motion and we'll see how this helps us
to characterize our robot motion.  From our two simplified equations of motion,
we can understand one relationship between linear and angular motion by
equating the two. And from this we'll see that we can compute how much power
we should apply to our motors to achieve a particular linear motion.

```math
    F = Sum(torques) = m*linear_acceleration = J*angular_accleration
```

### Back to Robots and Motors

Happily, there are a number of FRC-centric resources available to assist us with
drive characterization.  First and foremost is the [FRC Whitepaper On Drive Characterization](https://www.chiefdelphi.com/uploads/default/original/3X/f/7/f79d24101e6f1487e76099774e4ba60683e86cda.pdf5).
This paper is referenced by most other resources and is a great resource if
you want to understand some of the physical principles behind robot characterization.
Another great resource is chapter 4 of [Practical Guide to State-space Control](https://github.com/calcmogul/state-space-guide).  Equations you'll find there
are a little intimidating but some are quite central to our endeavor. Finally,
there are canned drive characterization procedures built into our
Team254-based codebase as well as in the [FRC Drive Characterization github](https://github.com/robotpy/robot-characterization).

At the end of the day, the goal described by these resources is to
characterize a robot drive for straight linear motion via this formula:

```java
    Vapp = Ks + Kv*velocity + Ka*acceleration;
```

This bears a strong resemblence to our physics formula for linear motion and for
our purposes the formulas are equivalent. Experts will be quick to point
out that Volts is _not_ a force but rather a potential and only when electrical
current runs through a motor are actual forces in play.

We can glean from this formula that the voltage we must apply to the robot in
order to achieve a target velocity depends on both its current velocity as
well its acceleration.  In other words, we can't select a voltage
requirement (and thus a feedforward term) without specifying both a target
velocity and a target acceleration. Here are the meanings and units for
the terms of this formula:

* Vapp - the voltage to request, we'll need to convert this to Kf
* Ks - the voltage required to break static friction (ie start moving)
* Kv - a velocity scaling term, measured in Volts/velocity
* Ka - an acceleration scaling term, measured in Volts/accel or Volts/vel/sec

Here's a simple procedure for obtaining an estimate of these constants:

* Build an autonomous routine that drives straight and slowly increases
  the voltage request. A range from 0 to .5 (or even .25) should suffice
  to capture the data we require. In TalonSRX terms this means running in
  "PctVbus" (open loop) mode and slowly increasing the PctOutput value. The
  process of ramping output should take place over, say, 10 seconds and so we
  could compute a "voltage ramp rate" and add that value to the requested pct
  on each robot-loop iteration.
* At each iteration, record the current velocity provided by the wheel encoders.
* After this test completes, you can run the data through a linear-regression
  analysis to find values for Ks and Kv.  This procedure assumes that the
  acceleration applied in our slow ramp is negligible.  The value of Ks
  is obtained by looking up the Voltage applied at the point that the Velocity
  becomes non zero. The value of Kv is simply the slope of the best-fit
  line through our data.  It is quite common to filter out "weird" values
  usually on the front-end of the data collection in order to get a better
  fit for the main portion of the curve.  Intuitively we don't want the
  non-linear forces associated with getting-going to distort our linear
  approximation for typical motion.
* In order to estimate the Ka term, we run another test. Here we apply
  an instantaneous voltage request of, say .5 (ie 6 Volts) for 2 seconds
  and measure how the velocity changes over time.  We obtain an acceleration
  approximation for a given voltage by dividing the measured velocity
  difference between iterations and divide by the time of the iteration.
  Again, we can run a regression analysis of the resulting data and fit
  a line to obtain an estimate for Ka.  It should be noted that this value
  is inherently less accurate (and thus noisier) since we can't really
  eliminate the effects of Kv during acceleration. It may be possible to
  take Kv into account while gathering the Ka data, but since we've not
  seen other teams attmpt this we assume it's either unnecessary or
  a bad idea.

For many uses, obtaining and applying these values is sufficient to obtain
reliable, repeatable control over the robot's autonomous driving. Interestingly
Team254 has taken the idea one step further and suggests that we also characterize
our robot's motion while turning. In physics parlance, we need to characterize
the robot's angular velocity and angular acceleration. In order to do this we
need to estimate the `moment of inertia` for our robot.


[Practical Guide to State-space Control](file:///C:/Users/danab/Desktop/robotics/FRC2019/_references/state-space-guide.pdf) (section 4.5.2) describes this procedure for estimating our robot's moment of
inertia (J).

1. Run the drivetrain forward at a constant voltage. Record the linear
    velocity over time.
2. Compute the linear acceleration from the linear velocity data as the
    difference between each sample divided by the time between them.
3. Rotate the drivetrain around its center by applying the same voltage as
    the linear acceleration test with the motors driving in opposite directions.
    Record the angular velocity over time.
4. Compute the angular acceleration from the angular velocity data as the
    difference between each sample divided by the time between them.
5. Perform a linear regression of linear acceleration versus angular acceleration.
    The slope of this line has the form J/rm as per equation
    ```java
    linearaccel = angularaccel * J / (r*m); // r is wheel radius, m is mass of robot
    ```
6. Multiply the slope by r*m to obtain a least squares estimate of J.

Note that we're not merely applying the ratio of Ka from linear to Ka from
angular.  Instead we plot one versus the other and estimate the slope of this
line.  Since the acceleration data is inherently noisy, we expect that the
resulting plot appears less linear and more "cloudy".  Also note that the
moment of inertia depends on the configuration of any articulable parts on
the robot.  This means that procedure should be performed with robot
components in a typical "competition configuration" where the path-planner
is in operation. Interestingly, Team254 provides an alternate formulation via
a Chief Delphi post [here](https://www.chiefdelphi.com/t/team-254-presents-lockdown-technical-binder-2018/166515/144):

```java
 // Torque/wheelRadius * chassisRadius = angularAccel * J;
 J = Torque*chassisRadius/(wheelRadius * angularAccel);
```

Where:

```java
    Torque/wheelRadius = linearAccel * robotMass; // F = ma = torque/r
```

It's confusing that they include chassisRadius in their formula where
statespace-guide doesn't. In that formulation (eq 4.30) the wheelbase
comes into play when used in conjunction with J but not to compute it.
Since the derivation depends on the sum of forces and since we measure
the wheel forces for linear motion in terms of the force a wheel exerts
on the ground, it seems likely that statespace-guide's formulation is
the correct one.

Team254 also reported that the plot of linear vs angular acceleration
wasn't terribly linear.  They introduced an `angular drag` term:

```text
Angular drag was added later, because we were seeing “velocity losses”
that were unexplainable otherwise. We were seeing a drag force ~proportional
to speed. This is probably due to scrubbing. We tuned the parameter until
our observed plots ~matched our model.
```

## Collecting the Data

In our Team254-based code we find an autonomous mode, `CharacterizeDriveMode`.
This class can be used to run the linear and angular characterizations described
above.  In its current state (2019, week4), it only runs a turning test and
not a linear one. It also offers the possibility of computing different
drive characterizations for each side.  This capability is definitely useful
to assist in debugging the electro-mechanical construction of the drivetrain
but probably shouldn't be employed for final characterizations because the
drivetrain-sides are inherently coupled when operating on the ground. The means
by which the constants are obtained are intimately connected with the physical
characteristics of the robot and the sharing of the load between drivetrain
sides.

One issue with the code is that it performs an automated regression analysis.
Our experience is that results we obtain this way are inferior to those
obtained after some manual cleanup. The good news is that it's very easy
to save the collected data into a ".csv" file that can easily be imported
into a webapp for regression analyis like [this one](https://mycurvefit.com/).
When applied to final production values it seems likely that we should only
run this procedure on the ground and for both linear and angular values and
only code Ka,Kv,Ka,Kt,J values after human inspection.

Another method for obtaining the _linear_ motion data can be found in the
[robotpy characterization framework](https://github.com/robotpy/robot-characterization).
This methodology provides a plethora of graphs to better visualize the
the test results. At the end of the day it seems likely that since
it produces estimates for linear motion in the same manner as our java
implementation it will be more convenient to use our integrated solution
for both linear _and_ angular rather than the python solution for one and
our java code for the other.

```java
List<DriveCharacterization.VelocityDataPoint> linearVelData = new ArrayList<>();
List<DriveCharacterization.AccelerationDataPoint> linearAccData = new ArrayList<>();
List<DriveCharacterization.AccelerationDataPoint> angularVelData = new ArrayList<>();
List<DriveCharacterization.AccelerationDataPoint> angularAccData = new ArrayList<>();

boolean reverse = false;
boolean turnInPlace;
CharacterizeDriveMode.SideToCharacterize side = SideToCharacterize.BOTH;

// first collect linear data
turnInPlace = false;
runAction(new CollectVelocityData(linearVelData, reverse, turnInPlace, side));
runAction(new WaitAction(10));
runAction(new CollectAccelerationData(linearAccData, reverse, turnInPlace, side));
runAction(new WaitAction(10));

// next collect angular data
turnInPlace = true;
runAction(new CollectVelocityData(angularVelData, reverse, turnInPlace, side));
runAction(new WaitAction(10));
runAction(new CollectAccelerationData(angularAccData, reverse, turnInPlace, side));

// next write out the data for regression analysis.  This will result
// in values for Ks, Kv, Ka/Kt and J.  NB: currently angularVelData
// may not be needed.
```

Upon completion of the regression analysis, we place values into
`Constants.java` as `kDriveVIntercept`, `kDriveKv` and `kDriveKa` and
`kRobotAngularInertia`.  The value `kRobotLinearInertia` should be
measured by weighing the robot (including bumper and battery).  The
value for `kRobotAngularDrag` may be less important but if we find ourselves
having troubles following high-curvature paths, we may need to tune this
value.

Another action, `CollectCurvatureData`, captures the relationship between
RobotState's measured dynamics with known (and different) left and right
motor power.  This procedure interacts with the RobotState odometry to
compare the wheel's "predicted curvature" (dx, dtheta) with the voltages
applied to left and right motors. Currently there's no obvious connection
between the results of this test and the tuning of any constants.

## The Rubber Hits the Road

### Drive.java

Here is the line in subsystems/Drive.java that makes things happen.

``` java
mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand,
            DemandType.ArbitraryFeedForward,
            mPeriodicIO.left_feedforward +
            Constants.kDriveVelocityKd * mPeriodicIO.left_accel/1023.0);
```

First, notice that this new entrypoint has appeared since our 2018 season.
As before, the `demand0` parameter is expressed in terms of target velocity
(and in native units). What's new here is that we are given the opportunity
to update our feedforward term at the same time via `demand1`. This makes
sense since our MotionPlanner is continuously updating our target velocity
and it turns out to be necessary because for most drivetrains, the velocity
is not a simple linear function of motor voltage. Most importantly the original
CTRE formulation for Kf assumes Ks is 0 which is definitely not the
case for drivetrains.

The [ctre docs](http://www.ctr-electronics.com/downloads/api/java/html/interfacecom_1_1ctre_1_1phoenix_1_1motorcontrol_1_1_i_motor_controller.html)
say this:

```text
ArbitraryFeedForward: Use demand1 as an arbitrary additive value to the
demand0 output.
```

What isn't clear from this documentation is whether the value is interpretted
as a feedforward value or as something else.  The Team254 code is a better source
of insight and it's clear from the implementation that demand1 is a
pct-vbus signal. Here is the update code during path following:

```java
void updatePathFollower(...)
{
  DriveMotionPlanner.Output output = mMotionPlanner.update(now, robotState);
  DriveSignal demand = new DriveSignal(
                        radiansPerSecondToTicksPer100ms(output.left_velocity),
                        radiansPerSecondToTicksPer100ms(output.right_velocity));
  DriveSignal feedfwd = new DriveSignal(output.left_feedforward_voltage / 12.0,
                        output.right_feedforward_voltage / 12.0);
  this.setPathVelocity(target, feedfwd);
}

void setPathVelocity()
{
    // In velocity-Control mode:
    //      demand is measured in ticksPer100ms
    //      feedfwd is measured in pctVbus [0,1]
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = feedfwd.getLeft();
    mPeriodicIO.right_feedforward = feedfwd.getRight();
}
```

What we learn from this is that the motion planner produces an output velocity
measured in radiansPerSecond and a feedforward term measured in Volts.

### DriveMotionPlanner

At the heart of the system, `DriveMotionPlanner` combines pre-planned paths
with drive characteristics as well as a selectable trajectory-following
algorithm and the measured/estimated state of the robot to produce target
velocities (demand) and target voltages (feedforward). Forming the basis
of the DriveMotionPlanner is a model of the drivetrain kinematics and
dynamics, `DifferentialDrive`.  On each update, we obtain a new target
position, orientation and velocity from the motion trajectory based on
the current time. This target is compared against an estimate of the current
robot state to produce an error. Now depending on the configured follower
algorithm, we produce our `Output` based on the current state and the
`DifferentialDrive.DriveDynamics` which are a function of our new target.

### DifferentialDrive

`DifferentialDrive` is the interface used by `DriveMotionPlanner` to
produce a new power configuration for our motors given the current
state of the robot chassis and its difference (error) from our target.

Before we proceed any deeper, let's define some important terms.
`Kinematics` refers to the way that we convert forces applied in
robot-coordinates (like wheel rotations) into world or field
coordinates (like the linear and angular acceleration of the robot chassis).
`Dynamics` refers to the amount of force required to achieve a requested
velocity and acceleration.  Different drive models require different _kinematics_.
Similarly, robots with the _same_ kinematic model will generally have different
_dynamics_ due to differences in mass, mass distribution and sources of friction.

The DifferentialDrive is the representation of the robot kinematics
for a robot with two independently operating drivetrains located on the
left and right side of the robot.  If we wish to adopt an alternate
drive-train design like mecanum or swerve, we'll need need an alternate
implementation of DifferentialDrive, and likely abstract that interface
so we can easily switch between different drive models. This is the
approach followed by "jaci's pathfollower" - the software package that
has been recently adopted as the standard FRC/WPI path follower implementation.

Here are the kinematic equations for the differential drive.  Note that
these formulas are independent of time.  That means that you can use them
for position, velocity or acceleration calculations.

```java
public ChassisState solveForwardKinematics(final WheelState wmotion)
{
    ChassisState s = new ChassisState();
    s.linear = kWheelRadius * (wmotion.right + wmotion.left) / 2.0;
    s.angular = kWheelRadius * (wmotion.right - wmotion.left) / kWheelbaseDiameter;
    return s;
}
public WheelState solveInverseKinematics(final ChassisState cmotion)
{
    WheelState s = new WheelState();
    w.left = (cmotion.linear - kWheelbaseRadius * cmotion.angular) / kWheelRadius;
    s.right = (chassis_motion.linear + kWheelbaseRadius * cmotion.angular) / kWheelRadius;
    return s;
}
```

In the `forward kinematics`, we convert the wheel velocities into robot velocity.
The key point here is that the `linear` term is just the average of the left and
right wheel terms.  If one side is rotating in the opposite direction of the other
then we get a linear velocity of 0. The robot is chasing its tail. The `angular`
term is a function of the _difference_ between left and right and so if they are
equal, the robot is driving straight. Also note that any calculations involving
robot turning involve the effective robot wheelbase. As the wheelbase increases,
larger changes of wheel values make smaller changes is chassis angular values.

In `inverse kinematics` we convert the current chassis linear and angular
velocity into wheel velocities. We can use inverse kinematics to determine
the left and right wheel rotation rate for a given robot motion specification.
This is a key requirement for our path-follower since its paths are defined
relative to the robot and not to each wheel.

Note that in order to solve kinematics we don't require access to physical
properties besides wheel and wheelbase and we have arrived at a place where
we know how many turns or turns/sec of each wheel required to move the robot
the requested distance or at the requested velocity. We're still missing a
key ingredient. In order to command the robot to move at a particular speed
(linear and angular), we need to know how much voltage to provide to each
motor.  And to do this we need to consider the robot _dynamics_ and consider
the other physical characteristics of our robot.

The key polymorphic method, `DifferentialDrive.solveInverseDynamics`, is
responsible for converting the requested robot/chass velocity and acceleration
request into `DriveDynamics`.  It should be noted that in order to even produce
these requests, the Motion Planner must have already taken into account some
physical properties of our robot in order to plan an _achievable path_.
This machinery is discussed [elsewhere](PathFollowing.md). For now,
the key idea is that the MotionPlanner needs to convert its field and
robot-centeric velocity and acceleration requirements into wheel velocity
and acceleration and from there into motor output requirements in the
form of wheel velocity setpoint and wheel voltage feedforward terms.

```java
public static class DriveDynamics
{
    public double curvature = 0.0; // m^-1
    public double dcurvature = 0.0; // m^-1/m
    public ChassisState chassis_velocity = new ChassisState(); // m/s
    public ChassisState chassis_acceleration = new ChassisState(); // m/s^2
    public WheelState wheel_velocity = new WheelState(); // rad/s
    public WheelState wheel_acceleration = new WheelState(); // rad/s^2
    public WheelState voltage = new WheelState(); // V
    public WheelState wheel_torque = new WheelState(); // N m
}
```

This class encapsulates all the information required to produce a feedforward
voltage term for our path follower.  The method, `solveInverseDynamics`, must
account for the current velocity and acceleration in order to produce values
for wheel/motor voltage, velocity, acceleration, etc.  It relies on the
services of `DCMotorTransmission` for the the drive characterization
constants.  Note that by converting the required dynamics into wheel torques,
we encapsulate/segregate the details of the chassis dynamics from the details
of the actual dc motor.

```java
// Convert chassis acceleration to wheel torques.
public void solveInverseDynamics(DriveDynamics d)
{
    //  J ia moment of inertia
    //  m is mass: linear intertia
    //  effectiveWheelbase radius: might be larger than actual to compensate
    //      for skid steer.  Measure by turning the robot in place several
    //      times and figuring out what the equivalent wheelbase radius is.
    d.wheel_torque.left = wheelRadius/2.0 *
        (d.chassis_acceleration.linear * m +
         d.chassis_acceleration.angular * J/effectiveWheelbase -
         d.chassis_velocity.angular * angularDrag/effectiveWheelbase);

   d.wheel_torque.right = wheelRadius/2.0 *
        (d.chassis_acceleration.linear * m +
         d.chassis_acceleration.angular * J/effectiveWheelbase +
         d.chassis_velocity.angular * angularDrag/effectiveWheelbase);

    // Solve for input voltages.
    d.voltage.left = left_transmission_.getVoltageForTorque(
                        d.wheel_velocity.left, d.wheel_torque.left);
    d.voltage.right = right_transmission_.getVoltageForTorque(
                        d.wheel_velocity.right, d.wheel_torque.right);
}
```

### DCMotorTransmission

Model of a DC motor rotating a shaft. All parameters refer to the output
(e.g. should already consider gearing and efficiency losses). The motor
is assumed to be symmetric forward/reverse.

Here is its state/member variables:

``` java
final double speed_per_volt;
final double torque_per_volt;
final double friction_voltage;
```

Here's how `DriveMotionPlanner` constructs instances of this class:

```java
private DCMotorTransmission makeTransmission(double vIntercept, double kv, double ka)
{
    double wheelRadiusM = Units.inches_to_meters(Constants.kDriveWheelRadiusInches);
    double speedPerVolt =  1/kv; // volts/vel -> vel/volts
    double torquePerVolt = wheelRadiusM*wheelRadiusM*m / (2.0 * ka),
    return new DCMotorTransmission(speedPerVolt, torquePerVolt, vIntercept);
}
```

And here, _finally_, is where the voltages are computed. Note that the
function requires both a speed and an acceleration.

```java
{
public double getVoltageForTorque(final double output_speed, final double torque)
{
    double ks;
    if (output_speed > Util.kEpsilon)
        ks = friction_voltage; // Forward motion, rolling friction.
    else if (output_speed < -Util.kEpsilon)
        ks = -friction_voltage; // Reverse motion, rolling friction.
    else if (torque > Util.kEpsilon)
        ks = friction_voltage; // System is static, forward torque.
    else if (torque < -Util.kEpsilon)
        ks = -friction_voltage; // System is static, reverse torque.
    else
        return 0.0; // System is idle.
    return torque/torque_per_volt + output_speed/speed_per_volt + ks;
}
```


## References

* [FRC Whitepaper On Drive Characterization](https://www.chiefdelphi.com/t/paper-frc-drivetrain-characterization/160915)
* [Practical Guide to State-space Control](https://github.com/calcmogul/state-space-guide) (ch 4)
* [RobotPy characterization script](https://github.com/robotpy/robot-characterization)


## Misc notes from chief delphi
Great discussion of acceleration variability:

``` text
However, the dynamic case is not analogous: friction between the gears will
clearly vary with acceleration, as the normal force between gear teeth is
proportional to the output torque of the motors, and thus to robot acceleration.
Thus, the amount by which our empirical value of ka is inﬂated from the
theoretical is representative of the torsional loss from the motors to
the wheels as a result of frictional eﬀects that vary with load. From
these data, it appears that the drive of our test robot loses over half
of its motor torque to such losses.
```

On batteries and voltage compensation:  (we currently enable voltage compensation in drive and *don't employ voltage ramp-rate*)

```text
A number of caveats apply to our suggested methodology, however. Firstly,
it is absolutely crucial that some step be taken to account for "voltage sag"
due to the internal resistance of the battery and resistance of the robot's
wiring. The Talon SRX motor controller oﬀers a "closed-loop voltage compensation"
option that does a ﬁne job of accomplishing this; battery voltage monitoring
via the PDP can also be used to apply the necessary correction.
```

* example data [here]( https://gist.github.com/JayShower/eeda672aa3033b457da113aa625c85e0).
Their Ks was 1.4V

On trimming the data prior to linear regression:

```text
Additionally, I looked through your data a bit and it seems you didn't
quite trim the quasi-static data enough; there are still a number of
points at which the robot wasn't moving (visible as a "flat" portion
at the start of the velocity plot). I've re-calculated your values with a
more-appropriate trimming (I removed all velocities under 2 inches per second),
```

Conclusions from [related thread]( https://www.chiefdelphi.com/t/the-great-drive-base-performance-characterization-data-sharing-thread/162243/3)

```text
... from the data I've seen in this thread (and some additional data from
some of our other robots that I'll get around in posting eventually), it
seems the standard for a "healthy" robot drive is a velocity efficiency in
the neighborhood of 90-100%, an acceleration efficiency around 70%, and a
voltage intercept in the neighborhood of 1-1.5 volts (tending to be lower
if gearing is higher).

In practice, this means Kv will generally have a value of ~1-1.1 times that of
theoretical, and Ka will generally have a value of ~1.4 times that of
theoretical. So, if you need to "bootstrap" values when you don't have time
to run the actual tests, this is probably a safe bet.

If you see acceleration efficiencies much below 70%, you should probably check
your drive for sources of inefficiency.
```

And the important constraints of maxV vs maxA:

```text
From the voltage balance equation, we see that:

    Vapp = Kv * velocity + Ka * acceleration * Vintercept

If we limit ourselves to 9V (to allow room for voltage sag and extra headroom
for the controller), we thus must pick our maximum velocity and acceleration
values such that Kv*vmax + Ka*amax + Vintercept = 9 volts. As I mentioned
earlier, this equation clearly shows that we have a "tradeoff" between maximum
velocity and maximum acceleration - the higher we allow one to be, the lower we
must make the other to ensure that our setpoints are achievable.

For instance, if we pick a maximum velocity of 6 ft/s, then our maximum
acceleration must be:

    (9 - .76 * 6 - .78)/.27 = 13.5 feet per second squared.
```

Comments from `DiffererentialDrive.java`

``` java
// Equivalent moment of inertia when accelerating purely angularly, in kg*m^2.
// This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
// Measure by doing drivetrain acceleration characterization while turning
// in place.
protected final double moi_;

// Drag torque (proportional to angular velocity) that resists turning, in N*m/rad/s
// Empirical testing of our drivebase showed that there was an unexplained loss in torque ~proportional to angular
// velocity, likely due to scrub of wheels.
// NOTE: this may not be a purely linear term, and we have done limited testing, but this factor helps our model to
// better match reality.  For future seasons, we should investigate what's going on here...
    protected final double angular_drag_;
```

On angular drag?

``` text

Gathering data, then fitting the parameters to what we saw.
Our exact procedure for this was something like:

Determine the parameters for straight line motion (the Kv, Ka, and friction
params that are now well known from the drivetrain characterization paper).

Assume that we know the load in the linear case (given that it's mostly mass,
assumes drivetrain inertia is negligible). This lets us derive a torque
constant Kt from the Ka parameter above.

Do a drivetrain characterization run (constant velocity run for Kv, then slow
ramp of motor voltage for friction and Ka) in a turn-in-place test to measure
an angular Ka parameter.

Assume the same amount of torque is available in the turn-in-place case as in
the linear case. Now use algebra to figure out what the moment of inertia
must be to account for the angular Ka we measured.

Angular drag was added later, because we were seeing "velocity losses" that
were unexplainable otherwise. We were seeing a drag force ~proportional to
speed. This is probably due to scrubbing. We tuned the parameter until our
observed plots ~matched our model.
```

In regards to this, could you clarify what equation you used to turn these two Ka values into moment-of-inertia? It's not immediately apparent from the drivetrain characterization paper or the code used to do the characterization how this conversion is made.


```text
The Ka "acceleration constant" you get from the drivetrain characterization
paper is a convenient representation, and mathematically sound, but it doesn't
really decompose into real-world engineering units. That paper basically comes
up with the formula:

    V = Ks + Kv * velocity + Ka * acceleration

Make one subtle change and you'll obtain the formulas we used. Instead of

    Ka * acceleration

we used a

    Kt * torque

The relationship between the two is straightforward in the linear case:


    Torque / wheel radius = linear acceleration * robot mass

(technically you can add other loads, like the inertia of the drive transmission
itself, into the robot "mass". Also, be sure to account for gear ratios
somewhere.)

We simply used our measured robot mass and wheel radius to convert from the
observed Ka to a Kt value.

Now, run the same experiment again but turn-in-place, and you'll observe a
different Ka value than you did in the linear case. Why? Because the load is
different (now the moment of inertia of the robot turning about its center vs. the mass):

    Torque / wheel radius * effective wheelbase radius = angular acceleration * robot moi

```
