package com.spartronics4915.frc2019;

import com.spartronics4915.lib.math.twodim.geometry.Pose2d;
import com.spartronics4915.lib.math.twodim.geometry.Rotation2d;
import com.spartronics4915.lib.util.Units;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants
{
    //  ********************
    //       Subsystems
    //  ********************

    //  Drive
    public static final int kLeftDriveMasterId = 3;
    public static final int kLeftDriveSlaveAId = 4;
    public static final int kRightDriveMasterId = 1;
    public static final int kRightDriveSlaveAId = 2;
    public static final double kDriveEncoderPPR = 1440.0;   // PPR (1440) = CPR (360) * 4 (because quadrature)
    public static final int kPidgeonId = 10;

    //  Control Board
    public static final int kDriveJoystickPort = 0;
    public static final int kMainButtonBoardPort = 1;
    public static final double kJoystickThreshold = 0.5;

    //  PCMs
    public static final int kCargoHatchArmPCMId = 0;
    public static final int kClimberPCMId = 1;

    //  Panel Handler
    public static final int kPanelHandlerSolenoid = 2;
    public static final double kPanelEjectTime = 2.5;   // Seconds
    public static final boolean kPanelSolenoidExtend = true;
    public static final boolean kPanelSolenoidRetract = false;

    //  Cargo Intake
    public static final int kCargoIntakeSolenoid = 0;
    public static final int kCargoIntakeSolenoidClimb = 1;
    public static final int kCargoIntakeMotorRight = 6;
    public static final int kCargoIntakeMotorLeft = 7;
    public static final int kCargoIntakeSensor = 7;
    public static final double kCargoIntakeSpeed = -0.5;
    public static final double kCargoEjectSpeed = 0.5;
    public static final double kCargoIntakeClimbSpeed = -0.8;
    public static final double kCargoIntakeOnPulseDuration = 2;
    public static final double kCargoIntakeOffPulseDuration = 1;
    public static final boolean kCargoIntakeSolenoidExtend = true;
    public static final boolean kCargoIntakeSolenoidRetract = false;

    //  Cargo Chute
    public static final int kRampMotorId = 5;
    public static final int kRampMotorSlaveId = 13;
    public static final int kRampSolenoidId = 3;
    public static final int kRampSensorId = 0;
    public static final double kEjectSpeed = 1.0;
    public static final double kRampSpeed = 0.8;
    public static final double kShootSpeed = 1.0;
    public static final double kShootTime = 4.0;
    public static final double kChuteHighExtendTime = 0.5;          // Waits for the solenoids to extend
    public static final double kChuteLowRetractTime = 0.8;
    public static final double kTransitionTime = 1.0;
    public static final double kMinBallInChuteVoltage = 0.9;        // 1.5 read
    public static final double kShootIntoBayBackupDistance = 4.0;   // Inches
    public static final boolean kRampSolenoidExtend = true;
    public static final boolean kRampSolenoidRetract = false;

    //  Climber
    public static final int kFrontLeftSolenoidId1 = 0;  // Extend
    public static final int kFrontLeftSolenoidId2 = 1;  // Retract
    public static final int kFrontRightSolenoidId1 = 2; // Extend
    public static final int kFrontRightSolenoidId2 = 3; // Retract
    public static final int kRearLeftSolenoidId1 = 4;   // Extend
    public static final int kRearLeftSolenoid2 = 5;     // Retract
    public static final int kRearRightSolenoidId1 = 6;  // Extend
    public static final int kRearRightSolenoidId2 = 7;  // Retract
    public static final int kClimberFrontIRSensorID = 2;
    public static final int kClimberRearIRSensorID = 3;
    public static final double kClimberSensorFrontMinVoltage = 2.5; // Actual amount around 3
    public static final double kClimberSensorRearMinVoltage = 1.5;  // Actual amount around 2
    public static final double kClimberFrontSolenoidDelay = 0.0;    // Seconds

    //  ********************
    //        Physical
    //  ********************

    //  ********************
    //          Auto
    //  ********************

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
            this.fieldPose = new Pose2d(Units.millimetersToInches(x), Units.millimetersToInches(y), Rotation2d.fromDegrees(rotationDegrees));
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

    public static final double kDriveOffHabXFudgeAmount = 0.0; // inches

    public static Pose2d correctPoseForRobotLength(Pose2d oldpose)
    {
        return oldpose.transformBy(Constants.kRobotCenterToForward);
    }



    //  LIDAR ----------

    //public static final IReferenceModel kSegmentReferenceModel = new SegmentReferenceModel(Segment.makeInRectangle(new Point(2, 2), new Point(0, 0)));

    // Pose of the LIDAR frame w.r.t. the robot frame
    public static final double kLidarXOffset = -11;
    public static final double kLidarYOffset = -6;
    public static final double kLidarYawAngleDegrees = -90;


    //  ********************
    //         Vision
    //  ********************

}
