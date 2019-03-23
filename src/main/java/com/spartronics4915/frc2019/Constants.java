package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.subsystems.Drive;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.lidar.icp.IReferenceModel;
import com.spartronics4915.lib.lidar.icp.Point;
import com.spartronics4915.lib.lidar.icp.Segment;
import com.spartronics4915.lib.lidar.icp.SegmentReferenceModel;
import com.spartronics4915.lib.util.Units;
import com.spartronics4915.lib.util.Logger;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.FileSystems;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
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
            this.robotLengthCorrectedPose = correctPoseForRobotLength(this.fieldPose);
        }

        private ScorableLandmark(ScorableLandmark other)
        {
            this.fieldPose = new Pose2d(other.fieldPose.mirror());
            this.robotLengthCorrectedPose = correctPoseForRobotLength(this.fieldPose);
        }
    }

    // Not technically a scorable landmark (this is in inches)
    public static final Pose2d kRightRobotLocationOnPlatform;
    public static final Pose2d kMiddleRobotLocationOnPlatformReverse;
    public static final Pose2d kMiddleRobotLocationOnPlatformForward;

    public static final Pose2d kRightRobotLocationOffPlatform;
    public static final Pose2d kMiddleRobotLocationOffPlatformReverse;
    public static final Pose2d kMiddleRobotLocationOffPlatformForward;

    public static final double kDriveOffHabXFudgeAmount = -10.0; // inches

    public static Pose2d correctPoseForRobotLength(Pose2d oldpose)
    {
        return oldpose.transformBy(Constants.kRobotCenterToForward);
    }

    /* ROBOT PHYSICAL CONSTANTS */

    public static boolean kIsTestChassis; // todo: eliminate dependencies

    // Wheels
    public static final double kDriveWheelTrackWidthInches;
    public static final double kDriveWheelDiameterInches;
    public static final double kDriveWheelRadiusInches;
    public static final double kTrackScrubFactor;

    // Chassis with bumper size
    public static final double kRobotCenterToForward;
    public static final double kRobotCenterToSide;

    // Tuned dynamics
    public static final double kRobotLinearInertia;
    public static final double kRobotAngularInertia;
    public static final double kRobotAngularDrag;

    // Right
    public static final double kDriveRightVIntercept;
    public static final double kDriveRightKv;
    public static final double kDriveRightKa;

    // Left
    public static final double kDriveLeftVIntercept;
    public static final double kDriveLeftKv;
    public static final double kDriveLeftKa;

    public static final double kDriveLeftDeadband = 0.04;
    public static final double kDriveRightDeadband = 0.04;
    public static final boolean kDefaultBrakeMode = true; // on

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

    public static final double kDriveVelocityKp;
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
    // (Note that if multiple talons are dedicated to a mechanism, any sensors

    public static final double kDriveVisionHeadingKp = 1.0; // rad/s
    public static final double kDriveVisionHeightKp = 40.0; // ft/s
    public static final double kVisionTargetMaxStaleTime = 0.5;
    public static final String kVisionSelectedIndexKey = "Vision/selectedIdx";
    public static final int kMaxVisionTargets = 2;
    public static final Pose2d kReverseVisionCameraOffset;

    public static final double kDriveVoltageRampRate = 0.0;

    public static final double kTeleopMaxChassisVel = 3 * 2 * Math.PI; // rad/s

    /* I/O */
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
    public static final double kPanelEjectTime = 2.5; // Seconds
    public static final boolean kPanelSolenoidExtend = true;
    public static final boolean kPanelSolenoidRetract = false;

    // Cargo Intake
    public static final int kCargoIntakeSolenoid = 0;
    public static final int kCargoIntakeSolenoidClimb = 1;
    public static final int kCargoIntakeMotorRight = 6;
    public static final int kCargoIntakeMotorLeft = 7;
    public static final int kCargoIntakeSensor = 7;
    public static final double kCargoIntakeSpeed = -0.5;
    public static final double kCargoEjectSpeed = 0.5;
    public static final double kCargoIntakeClimbSpeed = -1.0;
    public static final boolean kCargoIntakeSolenoidExtend = true;
    public static final boolean kCargoIntakeSolenoidRetract = false;

    // Cargo Chute
    public static final int kRampMotorId = 5;
    public static final int kRampMotorSlaveId = 13;
    public static final int kRampSolenoidId = 3;
    public static final int kRampSensorId = 0;
    public static final double kRampSpeed = 1.0;
    public static final double kShootSpeed = 1.0;
    public static final double kShootTime = 4.0;
    public static final double kChuteHighExtendTime = 0.5; // Waits for the solenoids to extend
    public static final double kChuteLowRetractTime = 0.8;
    public static final double kTransitionTime = 1.0;
    public static final double kMinBallInChuteVoltage = 1.65; // 1.5 read
    public static final boolean kRampSolenoidExtend = true;
    public static final boolean kRampSolenoidRetract = false;
    public static final double kShootIntoBayBackupDistance = 4.0; // Inches

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
    public static final double kClimberSensorFrontMinVoltage = 2.5; // actual amount around 3
    public static final double kClimberSensorRearMinVoltage = 1.5; // actual amount around 2
    public static final double kClimberFrontSolenoidDelay = 0.0; // Seconds

    static
    {
        // per-config-name constants
        String config = "";
        String home = System.getProperty("user.home");
        Path machineIdFile = FileSystems.getDefault().getPath(home, "machineid");
        try
        {
            // file only exists to override defaults
            config = new String(Files.readAllBytes(machineIdFile));
        }
        catch (Exception e)
        {
            config = "default";
        }
        Logger.notice("running on " + config + " constants");

        switch (config)
        {
            case "TestChassis":
            case "TestChassis\n":
                kIsTestChassis = true;
                kDriveWheelTrackWidthInches = 23.75;
                kDriveWheelDiameterInches = 6;
                kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
                kTrackScrubFactor = 1.063;
                kRobotCenterToForward = 16.125; // inches TODO tune
                kRobotCenterToSide = 13.75; // inches TODO tune
                kReverseVisionCameraOffset = new Pose2d(-7.5, 0, Rotation2d.fromDegrees(180));

                kRobotLinearInertia = 27.93; // kg (robot's mass)
                kRobotAngularInertia = 1.7419; // kg m^2 (use the moi auto mode)
                kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune

                kDriveRightVIntercept = 0.7714; // V TODO tune
                kDriveRightKv = 0.1920; // V per rad/s TODO tune
                kDriveRightKa = 0.0533; // V per rad/s^2 TODO tune

                kDriveLeftVIntercept = 0.7939; // V TODO tune
                kDriveLeftKv = 0.1849; // V per rad/s TODO tune
                kDriveLeftKa = 0.0350; // V per rad/s^2 TODO tune

                kDriveVelocityKp = 0.2;
                break;

            case "SecondRobot":
            case "SecondRobot\n":
                kIsTestChassis = false;
                kDriveWheelTrackWidthInches = 25.75;
                kDriveWheelDiameterInches = 6;
                kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
                kRobotCenterToForward = 17.625; // inches
                kRobotCenterToSide = 18.75; // inches
                kReverseVisionCameraOffset = new Pose2d(-1.0, 0.0, Rotation2d.fromDegrees(180));

                // TODO: Below
                kTrackScrubFactor = 1.1982;

                kRobotLinearInertia = 67.81205 + 5; // kg (robot's mass)
                kRobotAngularInertia = 5.57937; // kg m^2 (use the moi auto mode)
                kRobotAngularDrag = 12.0; // N*m / (rad/sec)

                kDriveRightVIntercept = 0.9167; // V
                kDriveRightKv = 0.2405; // V per rad/s
                kDriveRightKa = 0.0651; // V per rad/s^2

                kDriveLeftVIntercept = 0.9238; // V
                kDriveLeftKv = 0.2448; // V per rad/s
                kDriveLeftKa = 0.0643; // V per rad/s^2

                kDriveVelocityKp = 0.3;
                break;

            default:
                kIsTestChassis = false;
                kDriveWheelTrackWidthInches = 25.75;
                kDriveWheelDiameterInches = 6;
                kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
                kRobotCenterToForward = 17.625; // inches
                kRobotCenterToSide = 18.75; // inches
                kReverseVisionCameraOffset = new Pose2d(-1.0, 0.0, Rotation2d.fromDegrees(180));

                kTrackScrubFactor = 1.037;

                kRobotLinearInertia = 67.81205; // kg (robot's mass)
                kRobotAngularInertia = 4.9698; // kg m^2 (use the moi auto mode)
                kRobotAngularDrag = 12.0; // N*m / (rad/sec)

                kDriveRightVIntercept = 1.0808; // V
                kDriveRightKv = 0.2458; // V per rad/s
                kDriveRightKa = 0.0617; // V per rad/s^2

                kDriveLeftVIntercept = 1.0685; // V
                kDriveLeftKv = 0.2411; // V per rad/s
                kDriveLeftKa = 0.0541; // V per rad/s^2

                kDriveVelocityKp = 0.3;
                break;
        }

        kRightRobotLocationOnPlatform = new Pose2d(48.0 + Constants.kRobotCenterToForward + kDriveOffHabXFudgeAmount,
                -64.0 + Constants.kRobotCenterToSide, Rotation2d.fromDegrees(180));
        kMiddleRobotLocationOnPlatformReverse = new Pose2d(kRightRobotLocationOnPlatform.getTranslation().x(), 0.0, Rotation2d.fromDegrees(180));
        kMiddleRobotLocationOnPlatformForward = new Pose2d(kMiddleRobotLocationOnPlatformReverse.getTranslation(), Rotation2d.fromDegrees(0));

        kRightRobotLocationOffPlatform =
                new Pose2d(95.27523622 + Constants.kRobotCenterToForward, -64.0 + Constants.kRobotCenterToSide, Rotation2d.fromDegrees(180));
        kMiddleRobotLocationOffPlatformReverse = new Pose2d(kRightRobotLocationOffPlatform.getTranslation().x(), 0.0, Rotation2d.fromDegrees(180));
        kMiddleRobotLocationOffPlatformForward = new Pose2d(kMiddleRobotLocationOffPlatformReverse.getTranslation(), Rotation2d.fromDegrees(0));
    }
}
