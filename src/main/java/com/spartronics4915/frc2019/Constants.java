package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.lidar.icp.IReferenceModel;
import com.spartronics4915.lib.lidar.icp.Point;
import com.spartronics4915.lib.lidar.icp.Segment;
import com.spartronics4915.lib.lidar.icp.SegmentReferenceModel;
import com.spartronics4915.lib.util.Units;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants
 * determined through calibrations.
 */
public class Constants
{

    public static final double kLooperDt = 0.01;

    /**** Careful! Measurement units are in millimeters ****/
    public enum ScorableLandmark
    {
        LEFT_LOADING_STATION(0.0, 3436.239, 0.0),
        RIGHT_LOADING_STATION(LEFT_LOADING_STATION),
        LEFT_ROCKET_CLOSE_FACE(5448.066, 3629.264, 28.75),
        RIGHT_ROCKET_CLOSE_FACE(LEFT_ROCKET_CLOSE_FACE),
        LEFT_ROCKET_MIDDLE_FACE(5819.775, 3392.382, 90),
        RIGHT_ROCKET_MIDDLE_FACE(LEFT_ROCKET_MIDDLE_FACE),
        LEFT_ROCKET_FAR_FACE(6191.484, 3629.264, 118.75),
        RIGHT_ROCKET_FAR_FACE(LEFT_ROCKET_FAR_FACE),
        LEFT_DRIVERSTATION_PARALLEL_CARGO_BAY(5598.319, 276.225, 180),
        RIGHT_DRIVERSTATION_PARALLEL_CARGO_BAY(LEFT_DRIVERSTATION_PARALLEL_CARGO_BAY),
        LEFT_CLOSE_CARGO_BAY(6623.05, 708.25, 90.0),
        RIGHT_CLOSE_CARGO_BAY(LEFT_CLOSE_CARGO_BAY),
        LEFT_MIDDLE_CARGO_BAY(7175.50, 708.025, 90.0),
        RIGHT_MIDDLE_CARGO_BAY(LEFT_MIDDLE_CARGO_BAY),
        LEFT_FAR_CARGO_BAY(7727.95, 708.025, 90.0),
        RIGHT_FAR_CARGO_BAY(LEFT_FAR_CARGO_BAY);

        public final Pose2d fieldPose;
        public final Pose2d robotLengthCorrectedPose;

        private ScorableLandmark(double x, double y, double rotationDegrees)
        {
            this.fieldPose = new Pose2d(Units.millimeters_to_inches(x), Units.millimeters_to_inches(y), Rotation2d.fromDegrees(rotationDegrees));
            this.robotLengthCorrectedPose = getRobotLengthCorrectedPose(this.fieldPose);
        }

        private ScorableLandmark(ScorableLandmark other)
        {
            this.fieldPose = new Pose2d(other.fieldPose.mirror());
            this.robotLengthCorrectedPose = getRobotLengthCorrectedPose(this.fieldPose);
        }
    }

    // Not technically a scorable landmark (this is in inches)
    public static final Pose2d kRightRobotLocationOffPlatform = new Pose2d(95.27523622+Constants.kRobotCenterToForward, -64.0+Constants.kRobotCenterToSide, Rotation2d.fromDegrees(180));


    public static Pose2d getRobotLengthCorrectedPose(Pose2d oldpose)
    {
        return new Pose2d(
                oldpose.getRotation().cos() * Constants.kRobotCenterToForward + oldpose.getTranslation().x(),
                oldpose.getRotation().sin() * Constants.kRobotCenterToForward + oldpose.getTranslation().y(),
                oldpose.getRotation());
    }

    /* ROBOT PHYSICAL CONSTANTS */

    // Wheels
    public static final double kDriveWheelTrackWidthInches = 25.75; // TODO tune
    public static final double kDriveWheelDiameterInches = 6;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.063; // TODO tune

    // Chassis with bumper size
    public static final double kRobotCenterToForward = 15.0; // in TODO tune
    public static final double kRobotCenterToSide = 15.0; // in TODO tune

    // Tuned dynamics
    public static final double kRobotLinearInertia = 27.93; // kg (robot's mass) TODO tune
    public static final double kRobotAngularInertia = 1.7419; // kg m^2 (use the moi auto mode) TODO tune
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune

    // Right
    public static final double kDriveRightVIntercept = 0.7714; // V TODO tune
    public static final double kDriveRightKv = 0.1920; // V per rad/s TODO tune
    public static final double kDriveRightKa = 0.0533; // V per rad/s^2 TODO tune

    // Left
    public static final double kDriveLeftVIntercept = 0.7939; // V TODO tune
    public static final double kDriveLeftKv = 0.1849; // V per rad/s TODO tune
    public static final double kDriveLeftKa = 0.0350; // V per rad/s^2 TODO tune

    public static final double kDriveLeftDeadband = 0.04;
    public static final double kDriveRightDeadband = 0.04;

    // LIDAR CONSTANTS ----------------
    public static final IReferenceModel kSegmentReferenceModel = new SegmentReferenceModel(
            Segment.makeInRectangle(new Point(2, 2), new Point(0, 0)));

    // Pose of the LIDAR frame w.r.t. the robot frame
    public static final double kLidarXOffset = -11;
    public static final double kLidarYOffset = -6;
    public static final double kLidarYawAngleDegrees = -90;

    /* CONTROL LOOP GAINS */
    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0; // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0; // inches per second

    // Adaptive pure pursuit
    public static final double kPathKX = 10.0; // units/s per unit of error... This is essentially a P gain on longitudinal error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final int kPositionPIDSlot = 0; // for compat with 2018
    public static final int kVelocityPIDSlot = 1; // for compat with 2018

    public static final double kDriveVelocityKp = 3.0;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 50.0;
    // The below should always be zero, because feedforward is dynamically produced by DriveMotionPlanner
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;

    public static final double kDrivePositionKp = 8;
    public static final double kDrivePositionKi = 0.0;
    public static final double kDrivePositionKd = 0.0;
    // Don't do feedforward on position control loops
    public static final double kDrivePositionKf = 0.0;
    public static final int kDrivePositionIZone = 0;
    public static final double kTurnDegreeTolerance = 2; // Degrees
    public static final double kTurnVelTolerance = 0.2; // in/sec

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
    public static final int kLeftDriveMasterId = 3;
    public static final int kLeftDriveSlaveAId = 4;
    public static final int kRightDriveMasterId = 1;
    public static final int kRightDriveSlaveAId = 2;
    public static final double kDriveEncoderPPR = 1440.0; // PPR (1440) = CPR (360) * 4 (because quadrature)
    public static final int kPidgeonId = 10;

    // Control Board
    public static final int kDriveJoystickPort = 0;
    public static final int kMainButtonBoardPort = 1;
    public static final double kJoystickThreshold = 0.5;

    // PCMs
    public static final int kCargoHatchArmPCMId = 0;
    public static final int kClimberPCMId = 1;

    // Panel Handler
    public static final int kPanelHandlerSolenoid = 2;

    // Cargo Intake
    public static final int kCargoIntakeSolenoid = 0;
    public static final int kCargoIntakeSolenoidClimb = 1;
    public static final int kCargoIntakeMotorRight = 6;
    public static final int kCargoIntakeMotorLeft = 7;
    public static final int kCargoIntakeSensor = 7;
    public static final int kCargoIntakeSensorMinDistance = 100; // TODO: unit/value
    public static final int kCargoIntakeSensorMaxDistance = 1000; // TODO: unit/value

    // Cargo Chute
    public static final int kRampMotorId = 5;
    public static final int kRampSolenoidId = 3;
    public static final int kRampSensorId = 0;
    public static final double kRampSpeed = 1.0; // TODO: tune
    public static final double kShootSpeed = 1.0; // TODO: tune
    public static final double kShootTime = 4.0;
    public static final double kExtendTime = 1.0; //Waits for the solenoids to extend TODO: tune
    public static final double kTransitionTime = 1.0;
    public static final double kMaxChuteBallDistanceThreshold = 1.0; // TODO: tune
    public static final double kMinBallInChuteVoltage = 1.3; // This SHOULD be good
    public static final boolean kRampSolenoidExtend = true;
    public static final boolean kRampSolenoidRetract = false;

    // Climber
    public static final int kFrontLeftSolenoidId1 = 0; // Extend
    public static final int kFrontLeftSolenoidId2 = 1; // Retract
    public static final int kFrontRightSolenoidId1 = 2; // Extend
    public static final int kFrontRightSolenoidId2 = 3; // Retract
    public static final int kRearLeftSolenoidId1 = 4; // Extend
    public static final int kRearLeftSolenoid2 = 5; // Retract
    public static final int kRearRightSolenoidId1 = 6; // Extend
    public static final int kRearRightSolenoidId2 = 7; // Retract
    public static final int kClimberFrontIRSensorID = 2;
    public static final int kClimberRearIRSensorID = 3;
    public static final double kClimberSensorFrontMaxDistance = 2.5; // actual amount around 3
    public static final double kClimberSensorRearMaxDistance = 1.5; // actual amount around 2
}
