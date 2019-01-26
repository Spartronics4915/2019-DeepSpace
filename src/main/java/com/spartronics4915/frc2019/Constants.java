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
    public static final double kDriveWheelTrackWidthInches = 23.75;
    public static final double kDriveWheelDiameterInches = 6;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 0.624; // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 27.22; // kg (robot's mass, guessed to be 60 lbs == 27.22 kg) TODO tune
    public static final double kRobotAngularInertia = 461; // kg m^2 (from an online calculator) TODO tune
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune

    // Uncomment for per-wheel constants
    // // Left
    // public static final double kDriveLeftVIntercept = 0.6806717258105043; // V
    // public static final double kDriveLeftKv = 0.27817287970276816; // V per rad/s
    // public static final double kDriveLeftKa = 0.01561261448817294; // V per rad/s^2
    // // Right
    // public static final double kDriveRightVIntercept = 0.921925703056741; // V
    // public static final double kDriveRightKv = 0.22982332117199153; // V per rad/s
    // public static final double kDriveRightKa = 0.020857218537664493; // V per rad/s^2

    // Left
    // public static final double kDriveLeftVIntercept = 1.59634357933506; // V
    // public static final double kDriveLeftKv = 0.199070536239607; // V per rad/s
    // public static final double kDriveLeftKa = 0.001200902662294194; // V per rad/s^2
    public static final double kDriveLeftVIntercept = 0.8029026946682132 ; // V
    public static final double kDriveLeftKv = 0.1880282238461718; // V per rad/s
    public static final double kDriveLeftKa = 0.002200902662294194; // V per rad/s^2
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

    public static final double kPathKX = 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    // PID gains for drive velocity loop (LOW GEAR)
    // Units: setpoint, error, and output are in ticks per second.
    public static final int kPositionPIDSlot = 0; // for compat with 2018
    public static final int kVelocityPIDSlot = 1; // for compat with 2018
    public static final double kDriveVelocityKp = 4.0;
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
    public static final int kLeftDriveMasterId = 3;
    public static final int kLeftDriveSlaveAId = 4;
    public static final int kRightDriveMasterId = 1;
    public static final int kRightDriveSlaveAId = 2;
    public static final double kDriveEncoderPPR = 1440.0; // PPR (1440) = CPR (360) * 4 (because quadrature)

    // Turret
    public static final int kTurretMotorId = 5; // TODO: Figure out the correct motor ID
    // We're using the CTRE Mag encoders: https://content.vexrobotics.com/vexpro/pdf/Magnetic-Encoder-User's-Guide-01282016.pdf
    public static final double kTurretPPR = 4096.0;
    public static final double kTurretUnitsPerRev = 13653; // 4096 Quadrature CPR * (10 / 3) Belt reduction

    // Control Board
    public static final int kDriveJoystickPort = 0;
    public static final double kJoystickThreshold = 0.5;

    // Arm Harvester
    public static final int kIntakeMotorLeftId = 5;
    public static final int kIntakeMotorRightId = 6;
    public static final int kIntakeSolenoidId = 1;
    public static final int kClimberSolenoidId = 2;

    // Cargo Ramp
    public static final int kCargoEjectorLeftId = 8;
    public static final int kCargoEjectorRightId = 9;
    public static final int kRampMotorId = 7;
    public static final int kFlipperSolenoidId = 3;

}
