# Drive Tuning

Wherein we discuss the murky business of getting the robot to move where we 
want and at the speed we want.

## Velocity Control Mode

Our CANTalons can be operated in a confusing variety of modes.  Central to
our autonomous motion and path-planning is the __Velocity Control Mode__ (VCM).
The idea of this mode is that there is a feedback loop (PID) that runs on each
Talon Master whose purpose is to track a target velocity.  As our robot traverses
a curved path, our path-planner's job is to repeatedly compute and command target
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
are different. CTRE docs, [Feed Forward, Kf](https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/README.md#feed-forward-kf), provide the following
formula for selecting Kf.

    Kf = ([Percent Output] x 1023) / [Velocity]

Careful inspection reveals that Kf is expressed in units of motor output/velocity.
In other words, Volts/inches/second. Recalling that internally velocity is
expressed in encoder ticks/100ms, we conclude that useful _values_ for Kf are
hardly intuitive.  But leaving out some constants, we can think of it as the
_power_ required to drive the motor at a particular speed. And importantly,
we must establish a _different_ value of Kf for each target velocity.

So the question for us is how do we know what values of Kf to choose and
how do we write software to deliver new Kf values "continuously".

## Drive Characterization

In the simplest terms, we wish to know how many volts are required
to drive a wheel on our fully-loaded drivetrain at a particular velocity.
We can easily imagine gathering data that captures the mapping between a
particular velocity and the voltage required to attain it. It gets a litte
trickier because no drive train is perfect and thus anticipate that the left
voltage map will differ from the right voltage map.

## Computing Target Velocities

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
drivetrains, the velocity is not a linear function of motor voltage. Our drive
characterization database tells us how much power is required by the motor to
achieve a target velocity and we employ it here to determine the feedforward
term required to achieve our demanded velocity.

The [ctre docs](http://www.ctr-electronics.com/downloads/api/java/html/com/ctre/phoenix/motorcontrol/can/BaseMotorController.html#set-com.ctre.phoenix.motorcontrol.ControlMode-double-com.ctre.phoenix.motorcontrol.DemandType-double-)
indicate that the `ArbitraryFeedForward` term is expressed in motor output
units.  This value is directly added to the value computed by the current
PID control loop which is always in the range [-1024, 1024].  Thus it
falls upon us to convert our feedforward term (conceptually power) into ctre
motor output units. As discussed earlier, this value is computed by
MotionPlanner and updated frequently. Finally, notice that we apply a Kd
multiplier to the computed acceleration term. We can think of this term
as a sort of mini-PID that is under our direct control.  Since we are
operating in Velocity mode, the Kd term can be applied to the _acceleration_
(acceleration is the derivative of velocity) in order to smooth out the
oscillations typical of a P-only controller.