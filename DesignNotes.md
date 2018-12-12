
# Notes on Team 254 FRC-2018 codebase

(a _work in progress_)

<!-- TOC depthFrom:2 depthTo:6 orderedList:false updateOnSave:true withLinks:true -->

- [RobotCode](#robotcode)
    - [Robot](#robot)
    - [ControlBoardInterface](#controlboardinterface)
    - [SubsystemManager](#subsystemmanager)
    - [VisionServer](#visionserver)
    - [VisionServer::ServerThread](#visionserverserverthread)
    - [VisionProcessor](#visionprocessor)
    - [VisionUpdate](#visionupdate)
    - [TargetInfo](#targetinfo)
    - [RobotStateEstimator](#robotstateestimator)
    - [RobotState](#robotstate)
    - [RigidTransform2d](#rigidtransform2d)
    - [Superstructure](#superstructure)
    - [Superstruct::SystemState](#superstructsystemstate)
    - [Drive](#drive)
    - [TrajectoryGenerator](#trajectorygenerator)
    - [DriveMotionPlanner](#drivemotionplanner)
    - [TrajectoryIterator](#trajectoryiterator)
    - [Pose2d](#pose2d)
    - [Pose2dWithCurvature](#pose2dwithcurvature)
    - [TimedState(Pose2dWithCurvature)](#timedstatepose2dwithcurvature)
    - [TimingConstraint(State) _interface_](#timingconstraintstate-_interface_)
    - [DifferentialDrive](#differentialdrive)
    - [DCMotorTransmission](#dcmotortransmission)
    - [CheesyDriveHelper](#cheesydrivehelper)
    - [Looper](#looper)
    - [Action](#action)
    - [AutoModeBase](#automodebase)
- [254 Vision_app](#254-vision_app)
    - [CameraTargetInfo](#cameratargetinfo)

<!-- /TOC -->

## RobotCode

### Robot

- is-a IterativeRobot
- has-a ControlBoardInterface (OI)
  - currently constructs ControlBoard, not GamepadControlBoard
- has-a CheesyDriveHelper
- has-a RobotState
- has-a VisionServer
- has-a mEnabledLooper, mDisabledLooper
  - initialized by mSubsystemManager,
  - disabledLooper ignores subsystems' loop, but still delivers periodic
      inputs and outputs
- has-a TrajectoryGenerator
- has-a AutoModeExecutor
- has-a SubsystemManager
- has-a instance of all Subsystems
  - has-a Drive
  - uses mSuperstructure to manage state conflicts
- implements robotInit()
  - invokes mTrajectoryGenerator.generateTrajectories()
- implements automousInit(), autonomousPeriodic()
  - samples the autoMode and starts mEnabledLooper, AutoModeExecuter
- implements teleopInit(), teleopPeriodic()
  The code uses state machines to ensure that no matter what buttons
  the driver presses, the robot behaves in a safe and consistent manner.
  Based on driver input, the code sets a desired state for each
  subsystem. Each subsystem will constantly compare its desired and
  actual states and act to bring the two closer.
- implemente disabledInit(), disabledPeriodic()
  - installs mDisabledLooper
- implements testInit(), testPeriodic()
- all periodic methods:
  - invoke: outputToSmartDashboard() (nb: testPeriodic doesn't)
  - updates ConnectionManager timestamp

### ControlBoardInterface

- is-a abstract base for ControlBoard and GamepadControlBoard
- implements sampling functions for each OI operation.
  Names are abstract - ie:  getAimButton(), not getButton3()

### SubsystemManager

Used to reset, start, stop and update all subsystems at once.

- has-a list of all subsystems
- implements EnabledLoop:
  - for s in subsystems:  s.readPeriodicInputs()
  - for l in subsysLoops: l.onLoop()
  - for s in subsystems: s.writePeriodicOutputs()
- implements DisabledLoop:
  - for s in subsystems:  s.readPeriodicInputs()
  - (no loops)
  - for s in subsystems: s.writePeriodicOutputs()

### VisionServer

- is-a CrashTrackingRunnable
- has-a List\<VisionUpdateReceivers\>
- has-a List\<ServerThread\> (but uses only one)

### VisionServer::ServerThread

- is-a CrashTrackingRunnable (Runnable implements run())
- implements handleMessage, called via runCrashTracxked()
  - timestamps messages when they are received (over the wire)
  - produces a VisionUpdate object which is distributed to all receivers
- has-a VisionServer::AppMaintenanceThread to keep adb in healthy state

### VisionProcessor

- is-a singleton
- is-a Loop (-> onLoop, etc)
- is-a VisionUpdateReceiver (->gotUpdate)
- registered with mEnabledLooper
- onLoop: invokes robotState->addVisionUpdate(VisionUpdate)

### VisionUpdate

- parses json string from droidphone
- has-a capturedAgoMs
- has-a capturedAtTimestamp - (presumes no comm latency)
- has-a TargetInfo[]

### TargetInfo

- has-a x, y, z (represents location in space)

### RobotStateEstimator

- is-a singleton
- is-a Loop (-> onLoop)
  - samples drive encoders (left/right)
  - samples gyro
  - sampledV = generateOdometryFromSensors(deltaLeft/Right, gyro)
  - predictedV = Kinematics.fwd(leftV, rightV)
  - -> addObservations(sampledV, predictedV)

### RobotState

keeps track of the poses of various coordinate frames throughout
the match. A coordinate frame is simply a point and direction in space that
defines an (x,y) coordinate system. Transforms (or poses) keep track of the
spatial relationship between different frames.

Robot frames of interest (from parent to child):

- Field frame: origin is where the robot is turned on
- Vehicle frame: origin is the center of the robot wheelbase, facing fwd
- Camera frame: origin is the center of the camera imager relative to
  the robot base.
- Goal frame: origin is the center of the boiler (note that orientation
  in this frame is arbitrary). Also note that there can be multiple goal
  frames.

As a kinematic chain with 4 frames, there are 3 transforms of interest:

- Field-to-vehicle: This is tracked over time by integrating encoder and
  gyro measurements. It will inevitably drift, but is usually accurate
  over short time periods.
- Vehicle-to-camera: This is a constant.
- Camera-to-goal: This is a pure translation, and is measured by the vision
  system.

RobotState class definition

- is-a singleton
- maintains list of 100 most recent observations
- has-a GoalTracker
- has-a InterpolatingTreeMap\<time, transform\> field_to_vehicle;
- implements addObservations(timestamp, measured_V, predicted_V);
  - addFieldToVehicleObs(timestamp, Kinematics.integFwd(mv,pV)
- implements getFieldToVehicle(timestamp)
  - interpolates the field
- implements getFieldToCamera(timestamp)
- implements addVisionUpdate(TargetInfo[])
  - corrects for camera pitch and yaw as installed on robot
  - estimates distance to Boiler via known height to target
  - transforms from field_to_camera to field_to_goals
  - invokes goalTracker.update()

### RigidTransform2d

- has-a Translate2d
- has-a Rotation2d

### Superstructure

The superstructure subsystem is the overarching superclass containing
all components of the superstructure: the intake, hopper, feeder, shooter
and LEDs. The superstructure subsystem also contains some miscellaneous
hardware that is located in the superstructure but isn't part of any
other subsystems like the compressor, pressure sensor, and hopper wall
pistons.  Instead of interacting with subsystems like the feeder and
intake directly, the Robot class interacts with the superstructure,
which passes on the commands to the correct subsystem. The superstructure
also coordinates actions between different subsystems like the feeder
and shooter.

- is-a Subsystem
- has-a SystemState


### Superstruct::SystemState

``` java
IDLE,
WAITING_FOR_ALIGNMENT, // waiting for the drivebase to aim
WAITING_FOR_FLYWHEEL, // waiting for the shooter to spin up
SHOOTING, // shooting
SHOOTING_SPIN_DOWN, // short period after the driver releases the shoot button where the flywheel
                    // continues to spin so the last couple of shots don't go short
UNJAMMING, // unjamming the feeder and hopper
UNJAMMING_WITH_SHOOT, // unjamming while the flywheel spins
JUST_FEED, // run hopper and feeder but not the shooter
EXHAUSTING, // exhaust the feeder, hopper, and intake
HANGING, // run shooter in reverse, everything else is idle
RANGE_FINDING // blink the LED strip to let drivers know if they are at an optimal shooting range

```

### Drive

- is-a Subsystem
- has-a DriveControlState (enumerating all possible drive states)
    (OPEN_LOOP, PATH_FOLLOWING, VELOCITY)
- has-a DriveMotionPlanner
- has-a Loop subclass, delivered via registerEnabledLoops
  - implements onLoop which behaves according to mDriveControllerState
    - PATH_FOLLOWING: -> updatePathFollower
      - invokes drive.setPathVelocity with mDriveMotionPlanner.output
        which sets periodicIO values (later sent to talons)

### TrajectoryGenerator

- singleton constructed during Robot constructor (yucky-construction sequence)
- has-a DriveMotionPlanner (separate instance from Drive's)
- has-a TrajectorySet
- implements generateTrajectories
  - instantiates a TrajectoryGenerator::TrajectorySet
- implements generateTrajectory with inputs:
  - List\<Pose2d\> waypoints (no timing)
  - List\<TimingConstraint\<Pose2dWithCurvature\>\>

### DriveMotionPlanner

- is-a CSVWritable
- has-a FollowerType (FEEDFORWARD_ONLY, PURE_PURSUIT, PID, NONLINEAR_FEEDBACK)
- has-a DifferentialDrive (more generally a drive-model)
- has-a setPoint (TimeState\<Pose2dWithCurvature\>)
- has-a mError (Pose2d)
- has-a mLastTime, mDt (deltaT)
- has-a mCurrentTrajectory: TrajectoryInterator\<TimedState\<Pose2dWithCurvature\>\>
- has-a Output:
  - has-a left/right velocity (rad/sec)
  - has-a left/right access (rad/sec^2)
  - has-a left/right feedforward_voltage (V)
  - implements Output update(timestamp, Pose2d current_state)
    - computes setPoint (timestamp, trajectory)
    - computes mError (distance from setPoint to current_state)
  - implements updatePID() (for PID followertype)
  - implements updatePurePursuit
  - implements updateNonlinearFeedback (kBeta:2.0, kZeta:.7)
- implements setTrajectory
- implements generateTrajectory(...List\<Pose2d\> waypoints,
                                List\<TimingContraint\<Pose2dWithCurvature\>\>,
                                initialConditions...)
  - returns Trajectory\<Pose2dWithCurvature\>
  - mVelocityController.setGoalAndConstraints

### TrajectoryIterator

### Pose2d

- has-a Translation2d
- has-a Rotation2d

### Pose2dWithCurvature

- is-a State (interpolable, distance-calculable)
- has-a Pose2d
- has-a curvature
- has-a dcurvature_ds

### TimedState(Pose2dWithCurvature)

- has-a State (Pose2dWithCurvature)
- has-a time (associated with target state)
- has-a velocity (dState/dt)
- has-a acceleration (d2State/dt2)
- implements interpolate(TimedState\<Pose2dWithCurvature\>, double pct)
  1. compute intermediate t (between this and other)
  1. compute newV and newState, givin state, vel and acc

### TimingConstraint(State) _interface_

- implements getMinMaxAcceleration(State, velocity) (returns minmax pair)
- implements getMaxVelocity(State, velocity), given a current state and
  target velocity, produce a constrained acceleration range.
- subclasses:
  - __CentripetalAccelerationConstraint__
    - has-a maxCentripetalAccel, limits hard turns.
    - limits max velocity according to combination of maxCentripetalAccel
      and the current curvature.  
  - __DifferentialDriveDynamicsConstraint__
    - has-a Drive instance
    - has-a abs-voltage-limit
    - limits velocity so it doesn't consume more than abs-voltage-limit.
      - (uses: drive.getMaxAbsVelocity())
    - limits min/max accel given
      - (uses: drive.getMinMaxAcceleration())
  - __VelocityLimitRegionConstraint__
    - has-a min_corner, max_corner (Translation2d)
    - has-a velocity_limit
    - limits velocity when state is within a field area
    - no limit on accel

### DifferentialDrive

### DCMotorTransmission

### CheesyDriveHelper

Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means
that the "turning" stick controls the curvature * of the robot's path
rather than its rate of heading change. This helps make the robot more
controllable at high speeds. Also handles the robot's quick turn
functionality - "quick turn" overrides constant-curvature turning for
turn-in-place maneuvers. Values returned are DriveSignals which are
intended to be passed directly to motors running in Open Loop mode,
bypassing entirely WPI's RobotDrive class and its arcadeDrive method.

- implements cheesyDrive(throttle, turn, isQuick, isHigh)

### Looper 

runs all of the robot's loops. Loop objects are stored in a List object.
They are started when the robot powers up and stopped after the match.

- has-a Notifier
- has-a List\<Loop\>

### Action

an interface that describes an iterative action. It is run by an
autonomous action, called by the method runAction in AutoModeBase
(or more commonly in autonomous modes that extend AutoModeBase)

### AutoModeBase

an abstract class that is the basis of the robot's autonomous routines. 
This is implemented in auto modes (which are routines that perform actions).

## 254 Vision_app

### CameraTargetInfo

- has a m_y, m_z (x is "optical axis", y is left/right, z is top/bottom)
  (m_x is always 1, so distance detection occurs on client)
