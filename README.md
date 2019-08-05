Spartronics 2019 Destination: Deep Space robot code
=======

Team 4915's code for DeepSpace. Derived from Team254's 2018 codebase.

The code is divided into several packages, each responsible for a different
aspect of the robot function. This README explains setup instructions, the
function of each package, and some of the variable naming conventions used.
Additional information about each specific class can be found in that class'
Java file.

## Commands and Subsystems

CHAOS, 2019's robot, can be broken down into various **subsystems** and **commands**.

### Subsystems

- Drive
- PanelHandler
- CargoChute
- CargoIntake
- Climber

### Commands

- Climb
- ClimbExtendAllPneumatics
- ClimbIntake
- ClimbRetractFrontPneumatics
- ClimbRetractBackPneumatics

- GroundEjectCargo
- ManualIntakeCargo
- IntakeArmDown
- IntakeHold
- IntakeStopMotors

- ManualRamp
- ManualShootCargoBay
- ManualShootCargoRocket
- ManualChuteUp
- ManualChuteDown

- ManualEjectPanel

- InsideFramePerimeter

- AssistedIntakeCargo
- AssistedShootRocket
- AssistedShootBay
- SelectLeftVisionTarget
- SelectRightVisionTarget
- AssistedEjectPanel
- AssistedIntakePanel
- ChangeSelectedVisionIndex

## Construction :construction:

My todo list.

- Reimplement our own Subsystem commands in Subsystem.java\
    - Uncomment and change subsystems to rely on Subsystem.java again


## Setup Instructions

### General

- Clone this repo
- Run `./gradlew` to download gradle and needed FRC libraries
- Run `./gradlew tasks` to see available build options
- Enjoy!

### Building/Deploying to the Robot

- Run `./gradlew build` to build the code. Use the `--info` flag for more details
- Run `./gradlew deploy` to deploy to the robot in Terminal (Mac) or Powershell (Windows)

## Code Highlights

- Building with Gradle - this is now standard for FRC in 2019.
- Path following with a nonlinear feedback controller and splines.
    To control autonomous driving, the robot utilizes a [nonlinear feedback controller](src/main/java/com/spartronics4915/frc2019/planners/DriveMotionPlanner.java#L263)
    and drives paths constructed of [quintic Hermite splines](src/main/java/com/spartronics4915/lib/spline/QuinticHermiteSpline.java).
- Path generation and visualization via Java app.
    Cheesy Path, a Java webapp, allows a user to quickly and easily create and
    visualize autonomous paths. It is located in the [`src/main/webapp`](src/main/webapp)
    directory and the [com.spartronics4915.path](src/main/java/com/spartronics4915/path)
    package.  Run with `./gradlew tomcatRunWar` and
    open [`http://localhost:8080`](http://localhost:8080). To stop the server,
    run `./gradlew tomcatStop`.
- Self-test modes for each subsystem
    Each subsystem contains a [`checkSystem()`](src/main/java/com/team254/frc2018/subsystems/Drive.java#L464)
    method that tests motor output currents and RPMs. These self tests allow us
    to quickly verify that all motors are working properly.
- Lidar Processing
    Even though this was not used on the final iteration of our robot code, we
    are still releasing our lidar processing code. This consisted of ICP
    algorithms to detect the scale within the points detected and sent by the
    [Slamtec RPLIDAR A2](http://www.slamtec.com/en/support#rplidar-a2) and can
    be found in the [`com.spartronics4915.frc2019.lidar`](src/main/java/com/spartronics4915/frc2019/lidar) package. Our RPLIDAR driver can be found [here](https://github.com/Team254/rplidar_sdk).

## Package Functions

- com.spartronics4915.frc2019
    Contains the robot's central functions and holds a file with all numerical
    constants used throughout the code. For example, the `Robot` class controls
    all routines depending on the robot state.

- com.spartronics4915.frc2019.auto
    Handles the execution of autonomous routines and contains the `actions`,
    `creators`, and `modes` packages.

- com.spartronics4915.frc2019.auto.actions
    Contains all actions used during the autonomous period, which all share a
    common interface, [`Action`](src/main/java/com/team254/frc2018/auto/actions/Action.java)
    (also in this package). Examples include shooting cubes, driving a trajectory,
    or moving the elevator. Routines interact with the Subsystems, which
    interact with the hardware.

- com.spartronics4915.frc2019.auto.creators
    Contains all the auto mode creators, which select the correct auto mode to
    run based on user input and FMS data.

- com.spartronics4915.frc2019.auto.modes
    Contains all autonomous modes. Autonomous modes consist of a list of
    autonomous actions executed in a certain order.

- com.spartronics4915.frc2019.controlboard
    Contains all the code for the different control boards. This allows any
    combination of driver station joysticks, button board, and Xbox Controllers
    to be used for both driving and operating. These are controlled by booleans
    in `Constants.java`.

- com.spartronics4915.frc2019.lidar
    Contains classes that are used to communicate with the Slamtec RPLIDAR A2
    and to store and process points sent by the lidar.

- com.spartronics4915.frc2019.lidar.icp
    Contains the algorithms for processing points sent by the lidar.

- com.spartronics4915.frc2019.loops
    Loops are routines that run periodically on the robot, such as calculating
    robot pose, processing vision feedback, or updating subsystems. All loops
    implement the `Loop` interface and are handled (started, stopped, added) by
    the `Looper` class, which runs at 200 Hz. The `Robot` class has one main
    looper, `mEnabledLooper`, that runs all loops when the robot is enabled.

- com.spartronics4915.frc2019.paths
    Contains the generator for all of the trajectories that the robot drives
    during autonomous period.

- com.spartronics4915.frc2019.planners
    Loops are routines that run periodically on the robot, such as calculating
    robot pose, processing vision feedback, or updating subsystems. All loops
    implement the `Loop` interface and are handled (started, stopped, added) by
    the `Looper` class, which runs at 200 Hz. The `Robot` class has one main
    looper, `mEnabledLooper`, that runs all loops when the robot is enabled.

- com.spartronics4915.frc2019.statemachines
    Contains the state machines for the intake and overall superstructure.

- com.spartronics4915.frc2019.states
    Contains states and other classes used in the subsystem and state machine classes.

- com.spartronics4915.frc2019.subsystems
    Subsystems are consolidated into one central class per subsystem, all of
    which extend the Subsystem abstract class. Each subsystem uses state
    machines for control. Each subsystem is also a singleton, meaning that
    there is only one instance of each. To modify a subsystem, one would get
    the instance of the subsystem and change its state. The `Subsystem` class
    will work on setting the desired state.

- com.spartronics4915.lib.drivers
    Contains a set of custom classes for TalonSRXs.

- com.spartronics4915.lib.geometry
    Contains a set of classes that represent various geometric entities.

- com.spartronics4915.lib.physics
    Contains classes that model DC motor transmissions and differential drive
    characterization.

- com.spartronics4915.lib.spline
    Contains the code for generating and optimizing splines.

- com.spartronics4915.lib.trajectory
    Contains classes for following and storing trajectories.

- com.spartronics4915.lib.trajectory.timing
    Contains classes for fitting trajectories with time profiles.

- com.spartronics4915.lib.util
    A collection of assorted utilities classes used in the robot code. Check
    each file for more information.

## Variable Naming Conventions

- k*** (i.e. `kDriveWheelTrackWidthInches`): Final constants, especially those
    found in the Constants.java file
- m*** (i.e. `mIsHighGear`): Private instance variables
