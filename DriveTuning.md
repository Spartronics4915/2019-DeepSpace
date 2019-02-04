# Drive Tuning

Wherein we discuss the murky business of getting the robot to move where we
want and at the speed we want.

<!-- TOC depthFrom:2 depthTo:6 orderedList:false updateOnSave:true withLinks:true -->

- [Introduction](#introduction)
- [Drive Characterization](#drive-characterization)
    - [From Theory to Practice](#from-theory-to-practice)
- [Delivering Target Velocities](#delivering-target-velocities)
    - [DCMotorTransmission](#dcmotortransmission)
    - [DifferentialDrive](#differentialdrive)
- [Computing Target Velocities For a Planned Path](#computing-target-velocities-for-a-planned-path)
- [References](#references)

<!-- /TOC -->

## Introduction

Our CANTalons can be operated in a confusing variety of modes.  Central to
our autonomous motion and path-planning is the __Velocity Control Mode__ (VCM).
The idea of this mode is that there is a feedback loop (PID) that runs on each
Talon Master whose purpose is to track a target velocity.  As our robot traverses
a curved path, our path-planner's job is to repeatedly compute target
velocities for each wheel. Different from the __Position Control Mode__, VCM
relies upon a _feedforward_ term. The idea is to help the VCM PID
be more effective by commanding the motors to be in _approximately_ the
correct power configuration so that the job of the feedback terms is merely
to fine-tune the motor power levels to hone in on our target velocities.
If we select a feedforward value of 0, then the feedback terms bear the full
responsibility of achieving the target velocity and that makes the PID value
tuning process extremely difficult to achieve.

So we must take some care is selecting values for Kf that make the PID tuning
more efficient and reliable. We now consider _what are the units of Kf_?
We know that PID values are multipliers for the error terms but
since Kf is independent of the error, it makes sense that its meaning (and units)
are different. CTRE docs, [Feed Forward, Kf](https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/README.md#feed-forward-kf),
provide the following formula for selecting Kf.

    Kf = ([Percent Output] * 1023) / [Velocity]

Careful inspection reveals that Kf is expressed in units of motor-controller
output/velocity. In other words, Volts/inches/second. Rewriting this we see:

    [Percent Output] * 1023 = Kf * Velocity

Recalling that internally velocity is expressed in encoder ticks/100ms,
we conclude that useful _values_ for Kf are hardly intuitive.  But leaving
out some constants, we can think of it as the _power_ required to drive the
motor at a particular speed.

The apparent goal for this interface conveys the notion that a feedforward term
can apply for _any velocity_. As we'll see below, this is inadequate for our
requirements and so we must establish a _`different value of Kf for each
target velocity`_.

So the question for us is how do we know what values of Kf to choose and
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
voltage map will differ from the right voltage map. And of course, until we
have a final robot with all subsystems mounted and operating, any
characterization efforts will be placeholders.

There are a number of FRC-centric resources available to assist with
drive characterization.  First and foremost is the [FRC Whitepaper On Drive Characterization](https://www.chiefdelphi.com/uploads/default/original/3X/f/7/f79d24101e6f1487e76099774e4ba60683e86cda.pdf5).
This paper is referenced by most other resources and is a great resource if
you want to understand some of the physics principles behind robot characterization.
Another great resources is chapter 4 of [Practical Guide to State-space Control](https://github.com/calcmogul/state-space-guide).  Finally, there are
canned drive characterization implementations built into our Team254-based codebase
as well as via the [FRC Drive Characterization github](https://github.com/robotpy/robot-characterization).

At the end of the day, the goal described by these resources it to characterize
a robot drive for straight linear motion via this formula:

```java
    Vapp = Ks + Kv*velocity + Ka*acceleration;
```

Note that the voltage applied to the robot depends on both its current
velocity as well its current acceleration.  In other words, we can't select
a voltage requirement (and thus a feedforeward term) without specifying
both a target velocity and a target acceleration. Here are the meanings
and units for the terms of this formula:

* Vapp - the voltage to request, we'll need to convert this to Kf
* Ks - the voltage required to break static friction (ie start moving)
* Kv - a velocity scaling term, measured in Volts/velocity
* Ka - an acceleration scaling term, measured in Volts/accel or Volts/vel/sec

Here's a simple procedure for obtaining an estimate of these constants:

* Build an autonomous routine that drives straight and slowly increases
  the voltage request. A range from 0 to .5 (or event .25) should suffice
  to capture the data we require. In TalonSRX terms this means running in "PctVbus"
  (open loop) mode and slowly increasing the PctOutput value. The process
  of ramping output should take place over, say, 10 seconds and so we could
  compute a "voltage ramp rate" and add that value to the requested pct
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
  non-linear forces associated with getting going to distort our linear
  approximation whilst in motion.
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
  otherwise a bad idea.

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
moment of intertia depends on the configuration of any articulable parts on
the robot.  This means that procedure should be performed with robot
components in a typical "competition configuration" where the path-planner
is in operation. Interestingly, Team254 provides an alternate formulation via
a Chief Delphi post [here](https://www.chiefdelphi.com/t/team-254-presents-lockdown-technical-binder-2018/166515/69):

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

### From Theory to Practice

In our Team254-based code we find an autonomous mode, `CharacterizeDriveMode`.
This class can be used to run the linear and angular characterizations described
above.  In its current state (2019, week4), it only runs a turning test and
not a linear one. It also offers the possibility of storing different
drive characterizations.  This capability is probably useful to assist
debugging the electro-mechanical construction of the drivetrain but shouldn't
be employed for final characterizations because the drivetrain is inherently
coupled when operating on the ground. The means by which the constants are
obtained are intimately connected with the physical characteristics of the robotg
and the sharing of the load between drivetrain sides.tly. In addition, it only runs a turning
test and not a linear one.  This can easily be modified to perform both
linear and angular acceleration tests.  One issue with the code is that it
performs an automated regression analysis.  Our experience is that this results
we obtain this way are inferior to those obtained after some manual cleanup.
When applied toward final production values it seems likely that we should
only run this procedure on the ground and for both

Another method for obtaining these data can be found in the
[robotpy characterization framework](https://github.com/robotpy/robot-characterization).
This methodology provides a plethora of graphs to better visualize the

``` java
class CharacterizationConstants
{
    public double ks; // V to break static friction
    public double kv; // V / rad / s
    public double ka; // V / rad / s^2
}
```

T

These values, in turn, should be placed into Constants.java as
`kDriveVIntercept`, `kDriveKv` and `kDriveKa`.  Note that these
physical values are represented in "SI units" and that our standard
for representing velocity is currently inches per second.

In our Team254-based code, we see an autonomous action `CollectVelocityData.java`.
This action moves from 0 to 25% power and measures the velocity associated with
each power level.  The data is written to a csv file and can be used to produce
a polynomial (or line) that maps velocity to voltage.  Similarly the action
`CollectAcclerationData.java` runs for a fixed time interval and samples the
rate at which velocity changes over time. The results of these two tests are
interpretted by another class, `DriveCharacterization.java` to produce an
efficient approximation of this data which, in turn, is used to help produce
the demand and feedforward values for our MotionPlanner. Another action, `CollectCurvatureData`, captures the relationship between RobotState's
measured dynamics with known (and different) left and right motor power
_(and it's my suspicion that CollectCurvatureData can be used to
validate the results of all the characterization machinery)_.
Currently, `DriveCharacterization` doesn't utilize the curvature dataset
but its method,  `characterizeDrive` does combine the velocity and
acceleration samples to produce this:


So far, we've characterized our drive in terms of its linear motion.
It turns out that mechanically, the motion of turning a robot is
different from moving a robot along a line.  In a frictionless setting,
in order to change velocity we must accelerate and this requires
application of force.  Angular velocity and angular momentum are terms we use to
describe the amount and rate of change of our robots' turning  and
just as with linear motion, we must apply a force to change our heading.



## Delivering Target Velocities

Here is the line in subsystems/Drive.java that makes things happen.

``` java
mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand,
            DemandType.ArbitraryFeedForward,
            mPeriodicIO.left_feedforward +
            Constants.kDriveVelocityKd * mPeriodicIO.left_accel/1023.0);
```

First, notice that this new entrypoint has appeared since our 2018 season.
As before, the `demand` parameter is expressed in terms of target velocity.
What's new here is that we are given the opportunity to update our feedforward
term at the same time. This makes sense since our MotionPlanner is continuously
updating our target velocity and it turns out to be necessary because for most
drivetrains, the velocity is not a simple linear function of motor voltage.

The [ctre docs](http://www.ctr-electronics.com/downloads/api/java/html/com/ctre/phoenix/motorcontrol/can/BaseMotorController.html#set-com.ctre.phoenix.motorcontrol.ControlMode-double-com.ctre.phoenix.motorcontrol.DemandType-double-)
indicate that the `ArbitraryFeedForward` term is expressed in motor output
units.  This value is directly added to the value computed by the current
PID control loop which is always in the range [-1024, 1024].  Thus it
falls upon us to convert our feedforward term (intuitively power) into ctre
motor output units. As discussed below, this value is computed by
MotionPlanner and updated frequently. Finally, notice that we apply a local
Kd multiplier to the computed acceleration term. We can think of this term
as a sort of mini-PD-controller that is under our direct control via the
feedforward term.  Since we are operating in Velocity mode, the Kp term
would to the velocity error and the Kd term can be applied to the
_acceleration_ (derivative of velocity) in order to smooth out the
oscillations typical of a P-only controller.


### DCMotorTransmission

Model of a DC motor rotating a shaft. All parameters refer to the output
(e.g. should already consider gearing and efficiency losses). The motor
is assumed to be symmetric forward/reverse.

``` java
double speedPerVolt = 1.0/Constants.kDriveKv; // speed_per_volt (rad/s/V no load)
double r = Units.inches_to_meters(Constants.kDriveWheelRadiusInches);
double rsq = r * r;
double torquePerVolt = rsq * Constants.kRobotLinearInertia /
                            (2.0 * Constants.kDriveKa); // N*m/V (stall)
double frictionVoltage = Constants.kDriveIIntercept; // V
new DCMotorTransmission(speedPerVolt, torquePerVolt, frictionVoltage);
```

### DifferentialDrive

Dynamic model a differential drive robot. Note: to simplify things, this math
assumes the center of mass is coincident with the kinematic center of rotation
(e.g. midpoint of the center axle).

## Computing Target Velocities For a Planned Path

At the heart of the system, `DriveMotionPlanner` combines pre-planned paths
with drive characteristics as well as a selectable trajectory-following
algorithm and the measured/estimated state of the robot to produce target
velocities (demand) and target voltages (feedforward). Forming the basis
of the DriveMotionPlanner is a model of the drivetrain kinematics and
dynamics, `DifferentialDrive`.  On each update, we obtain a new target
position, orientation and velocity from the motion trajectory based on
the current time. This target is compared against an estimate of the current
state to produce an error. Now depending on the follower algorithm, we produce
our `Output` based on the current state and the dynamics (which are a function
of our new target).


## References

* [FRC Whitepaper On Drive Characterization](https://www.chiefdelphi.com/t/paper-frc-drivetrain-characterization/160915)
* [RobotPy characterization script](https://github.com/robotpy/robot-characterization)
* [Practical Guide to State-space Control](https://github.com/calcmogul/state-space-guide) (ch 4)


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
