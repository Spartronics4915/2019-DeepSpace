package com.spartronics4915.frc2019;

import com.spartronics4915.lib.geometry.Translation2d;
import com.spartronics4915.lib.lidar.icp.IReferenceModel;
import com.spartronics4915.lib.lidar.icp.Point;
import com.spartronics4915.lib.lidar.icp.Segment;
import com.spartronics4915.lib.lidar.icp.SegmentReferenceModel;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants
 * determined through calibrations.
 */
public class Constants
{

    public static final double kLooperDt = 0.01;

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 25.75;
    public static final double kDriveWheelDiameterInches = 6;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 0.624; // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 27.22; // kg (robot's mass, guessed to be 60 lbs == 27.22 kg) TODO tune
    public static final double kRobotAngularInertia = 461; // kg m^2 (from an online calculator) TODO tune
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune

    // Uncomment for per-wheel constants
    // Left
    // public static final double kDriveLeftVIntercept = 0.646; // V
    // public static final double kDriveLeftKv = 0.242; // V per rad/s
    // public static final double kDriveLeftKa = 0.0008; // V per rad/s^2
    // // Right
    // public static final double kDriveRightVIntercept = 0.386; // V
    // public static final double kDriveRightKv = 0.233; // V per rad/s
    // public static final double kDriveRightKa = 0.0008; // V per rad/s^2

    // Left
    // public static final double kDriveLeftVIntercept = 1.59634357933506; // V
    // public static final double kDriveLeftKv = 0.199070536239607; // V per rad/s
    // public static final double kDriveLeftKa = 0.001200902662294194; // V per rad/s^2
    public static final double kDriveLeftVIntercept = 0.8029026946682132 ; // V
    public static final double kDriveLeftKv = 0.1880282238461718; // V per rad/s
    public static final double kDriveLeftKa = 0.0008; // V per rad/s^2
    // Right
    public static final double kDriveRightVIntercept = kDriveLeftVIntercept; // V
    public static final double kDriveRightKv = kDriveLeftKv; // V per rad/s
    public static final double kDriveRightKa = kDriveLeftKa; // V per rad/s^2

    public static final double kDriveLeftDeadband = 0.078;
    public static final double kDriveRightDeadband = 0.068;

    // Geometry
    public static final double kCenterToFrontBumperDistance = 38.25 / 2.0;
    public static final double kCenterToRearBumperDistance = 38.25 / 2.0;
    public static final double kCenterToSideBumperDistance = 33.75 / 2.0;

    // LIDAR CONSTANTS ----------------
    public static final IReferenceModel kSegmentReferenceModel = new SegmentReferenceModel(
        Segment.makeInRectangle(new Point(2, 2), new Point(0, 0))
    );

    // Pose of the LIDAR frame w.r.t. the robot frame
    public static final double kLidarXOffset = -11;
    public static final double kLidarYOffset = -6;
    public static final double kLidarYawAngleDegrees = -90;

    /* CONTROL LOOP GAINS */
    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0; // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0; // inches per second

    public static final double kPathKX = 10.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final int kPositionPIDSlot = 0; // for compat with 2018
    public static final int kVelocityPIDSlot = 1; // for compat with 2018
    public static final double kDriveVelocityKp = 5.0;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 50.0;
    // The below should always be zero, because feedforward is dynamically produced by DriveMotionPlanner
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;

    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors

    // Number of CAN devices for the CAN probe
    public static final int kNumTalons = 5; // total talon count on robot (not testbed)
    public static final int kNumPDPs = 1; // doesn't always show up in CANProbe
    public static final int kNumPCMs = 1; // Pressure control module (pneumatics)
    public static final int kNumCANDevices = kNumTalons + kNumPCMs; // don't count PDP

    // Drive
        
        //IDs
        public static final int kLeftDriveMasterId = 3;
        public static final int kLeftDriveSlaveAId = 4;
        public static final int kRightDriveMasterId = 1;
        public static final int kRightDriveSlaveAId = 2;
    
        //Numbers
        public static final double kDriveEncoderPPR = 1440.0; // PPR (1440) = CPR (360) * 4 (because quadrature)

    // Control Board
    public static final int kDriveJoystickPort = 0;
    public static final int kMainButtonBoardPort = 1;
    public static final double kJoystickThreshold = 0.5;

    // Panel Handler
        
        //IDs
        public static final int kPanelHandlerSolenoid = 2;

    // Cargo Intake
       
        //IDs
        public static final int kCargoIntakeSolenoid = 0;
        public static final int kCargoIntakeSolenoidClimb = 1;
        public static final int kCargoIntakeMotorRight = 6;
        public static final int kCargoIntakeMotorLeft = 7;
        public static final int kCargoIntakeSensor = 7;

        //Distances
        public static final int kCargoIntakeSensorMinDistance = 100; //TODO: unit/value
        public static final int kCargoIntakeSensorMaxDistance = 1000; //TODO: unit/value

    // Cargo Ramp
    public static final int kRampMotorId = 7;
    public static final int kFlipperSolenoidId = 3;
    public static final int kRampSensorId = 6;
    public static final double kShootSpeed = 1.0; // tune
    public static final double kRampSpeed = 1.0;
    public static final double kBallInPositionThreshold = 1.0; // TODO: tune
    public static final boolean kSolenoidExtend = false;
    public static final boolean kSolenoidRetract = true;

    // Climber
    public static final int kClimberPWMId = 2;
    public static final int kFrontLeftSolenoidId1 = 0; //Extend
    public static final int kFrontLeftSolenoidId2 = 1; //Retract
    public static final int kFrontRightSolenoidId1 = 2; //Extend
    public static final int kFrontRightSolenoidId2 = 3; //Retract
    public static final int kRearLeftSolenoidId1 = 4; //Extend
    public static final int kRearLeftSolenoid2 = 5; //Retract
    public static final int kRearRightSolenoidId1 = 6; //Extend
    public static final int kRearRightSolenoidId2 = 7; //Retract
    public static final int kFrontLeftIRSensorId = 0;
    public static final int kFrontRightIRSensorId = 1;
    public static final int kDownwardFrontRightIRSensorId = 2;
    public static final int kDownwardFrontLeftIRSensorId = 3;
    public static final int kDownwardRearRightIRSensorId = 4;
    public static final int kDownwardRearLeftIRSensorId = 5;
}
