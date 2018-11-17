package com.spartronics4915.frc2019.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.RobotState;
import com.spartronics4915.frc2019.lidar.LidarProcessor;
import com.spartronics4915.frc2019.loops.ILooper;
import com.spartronics4915.frc2019.loops.Loop;
import com.spartronics4915.frc2019.loops.Looper;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.Timer;

public class Turret extends Subsystem
{

    private static Turret mInstance = null;

    public static Turret getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Turret();
        }
        return mInstance;
    }

    public enum WantedState
    {
        DISABLED, FOLLOW_ODOMETRY, FOLLOW_LIDAR,
    }

    private enum SystemState
    {
        DISABLING, FOLLOWING,
    }

    private static final boolean kOutputInverted = false;
    private static final int kPIDSlot = 0;

    private TalonSRX mMotor;
    private LidarProcessor mLidar;
    private RobotState mOdometry;
    private WantedState mWantedState = WantedState.DISABLED;
    private SystemState mSystemState = SystemState.DISABLING;
    private boolean mUseLidar = false;
    private double mLastTurretAngle = 0;

    private Turret()
    {
        mLidar = LidarProcessor.getInstance();
        mOdometry = RobotState.getInstance();

        boolean success = true;
        try
        {
            mMotor = TalonSRXFactory.createDefaultTalon(Constants.kTurretMotorId);
            mMotor.setNeutralMode(NeutralMode.Brake);
            mMotor.config_kP(kPIDSlot, Constants.TurretPIDConstants.kP, Constants.kLongCANTimeoutMs);
            mMotor.config_kI(kPIDSlot, Constants.TurretPIDConstants.kI, Constants.kLongCANTimeoutMs);
            mMotor.config_kD(kPIDSlot, Constants.TurretPIDConstants.kD, Constants.kLongCANTimeoutMs);
            mMotor.config_kF(kPIDSlot, Constants.TurretPIDConstants.kF, Constants.kLongCANTimeoutMs);

            // Set up encoder
            mMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
            final ErrorCode sensorPresent = mMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
            if (sensorPresent != ErrorCode.OK)
            {
                logError("Could not detect encoder, error code: " + sensorPresent);
                success = false;
            }
        }
        catch (Exception e)
        {
            success = false;
        }

        logInitialized(success);
    }

    private Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Turret.this)
            {
                mWantedState = WantedState.DISABLED;
                mSystemState = SystemState.DISABLING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Turret.this)
            {
                SystemState newState = mSystemState;
                switch (mSystemState)
                {
                    case FOLLOWING:
                        Pose2d pose;
                        if (mUseLidar)
                        {
                            pose = mLidar.doICP();
                        }
                        else
                        {
                            pose = mOdometry.getFieldToVehicle(Timer.getFPGATimestamp());
                        }

                        double newAbsoluteAngle = calculateAbsoluteTurretAngle(pose, Constants.kTurretTargetFieldPosition);
                        mMotor.set(ControlMode.Position,
                                mMotor.getSelectedSensorPosition(0) + rotationsToRawUnits((newAbsoluteAngle - mLastTurretAngle) / 360));
                        mLastTurretAngle = newAbsoluteAngle;
                        break;
                    case DISABLING:
                        stop();
                    default:
                        newState = defaultStateTransfer();
                }
                mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (Turret.this)
            {
                stop();
            }
        }
    };

    private double rotationsToRawUnits(double rotations)
    {
        return rotations * Constants.kTurretUnitsPerRev;
    }

    /**
     * Calculates an absolute turret angle in degrees. The range is 0-360 degrees.
     * 
     * 0 degrees points "right" towards the target (i.e. on the negative side of the
     * target, where the target is the origin). Positive rotations run clockwise.
     * 
     * @return Absolute angle in degrees, with a range of 0-360
     */
    private double calculateAbsoluteTurretAngle(Pose2d robotPose, Translation2d targetTranslation)
    {
        robotPose.getTranslation().translateBy(Constants.kTurretRobotCenterOffset);
        Rotation2d angle = new Rotation2d(robotPose.getTranslation().x() - targetTranslation.x(),
                robotPose.getTranslation().y() - targetTranslation.y(), true)
                        .rotateBy(robotPose.getRotation().inverse());
        return angle.getDegrees() + 180.0;
    }

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case DISABLED:
                newState = SystemState.DISABLING;
                break;
            case FOLLOW_LIDAR:
                newState = SystemState.FOLLOWING;
                mUseLidar = true;
                break;
            case FOLLOW_ODOMETRY:
                newState = SystemState.FOLLOWING;
                mUseLidar = false;
                break;
            default:
                newState = SystemState.DISABLING;
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        logNotice("Wanted state to " + wantedState.toString());
        mWantedState = wantedState;
    }

    public synchronized WantedState getWantedState()
    {
        return mWantedState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    @Override
    public void stop()
    {
        mMotor.set(ControlMode.Disabled, 0.0);
    }

    @Override
    public void zeroSensors()
    {
        // Encoder only gets reset on startup
    }

    @Override
    public boolean checkSystem()
    {
        logWarning("Check system not implemented");
        return false;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutString("turretState", mSystemState.toString() + ", mUseLidar: " + mUseLidar);
    }

}
