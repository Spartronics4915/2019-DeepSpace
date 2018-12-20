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
are different. CTRE docs, [Feed Forward, Kf](https://github.com/CrossTheRoadElec/Phoenix-Documentation/blob/master/README.md#feed-forward-kf), suggest the following formula for selecting Kf.

    Kf = ([Percent Output] x 1023) / [Velocity]

Careful inspection reveals that Kf is expressed in units of
motor output / velocity.  Recalling that velocity is expressed in
encoder ticks / 100ms, we conclude that useful _values_ for Kf are hardly
intuitive.  But leaving out some constants we can think of it as the _power_
required to drive the motor at a particular speed. And most importantly, we
must establish a _different_ value of Kf for each target velocity.

So the question for us is how do we know what values of Kf to choose and
how do we write software to deliver new Kf values "continuously".

## Drive Characterization

## Computing Target Velocities

## Delivering Target Velocities
