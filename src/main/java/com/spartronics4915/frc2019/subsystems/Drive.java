package com.spartronics4915.frc2019.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.RobotState;
import com.spartronics4915.frc2019.loops.ILooper;
import com.spartronics4915.frc2019.loops.Loop;
import com.spartronics4915.frc2019.planners.DriveMotionPlanner;
import com.spartronics4915.lib.drivers.TalonSRXChecker;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Drive extends Subsystem
{

    private static final int kVelocityPIDSlot = 0;
    private static final double DRIVE_ENCODER_PPR = 4096.0; // TODO: Check me (I think this is correct for the CTRE magnetic encoder)
    private static Drive mInstance = new Drive();
    // Hardware
    private final TalonSRX mLeftMaster, mRightMaster, mLeftSlaveA, mRightSlaveA, mLeftSlaveB, mRightSlaveB; // TODO remove B slaves
    private PigeonIMU mPigeon;
    // Control states
    private DriveControlState mDriveControlState;
    // Hardware states
    private PeriodicIO mPeriodicIO;
    private boolean mIsBrakeMode;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private boolean mOverrideTrajectory = false;

    private final Loop mLoop = new Loop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Drive.this)
            {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(false);
                //                 startLogging();
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Drive.this)
            {
                switch (mDriveControlState)
                {
                    case OPEN_LOOP:
                        break;
                    case PATH_FOLLOWING:
                        updatePathFollower();
                        break;
                    default:
                        System.out.println("Unexpected drive control state: " + mDriveControlState);
                        break;
                }
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            stop();
            stopLogging();
        }
    };

    private void configureMaster(TalonSRX talon, boolean left)
    {
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK)
        {
            DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent, false);
        }
        talon.setInverted(!left);
        talon.setSensorPhase(true);
        talon.enableVoltageCompensation(true);
        talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
        talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
        talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
        talon.configNeutralDeadband(0.04, 0);
    }

    private Drive()
    {
        mPeriodicIO = new PeriodicIO();

        // Start all Talons in open loop mode.
        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
        configureMaster(mLeftMaster, true);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveAId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveBId,
                Constants.kLeftDriveMasterId);
        mLeftSlaveB.setInverted(false);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kRightDriveMasterId);
        configureMaster(mRightMaster, false);

        mRightSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveAId,
                Constants.kRightDriveMasterId);
        mRightSlaveA.setInverted(true);

        mRightSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveBId,
                Constants.kRightDriveMasterId);
        mRightSlaveB.setInverted(true);

        reloadGains();

        mPigeon = new PigeonIMU(mLeftSlaveB);
        mLeftSlaveB.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

        setOpenLoop(DriveSignal.NEUTRAL);

        // Force a CAN message across.
        mIsBrakeMode = true;
        setBrakeMode(false);

        mMotionPlanner = new DriveMotionPlanner();
    }

    public static Drive getInstance()
    {
        return mInstance;
    }

    private static double rotationsToInches(double rotations)
    {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm)
    {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches)
    {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second)
    {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s)
    {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    @Override
    public void registerEnabledLoops(ILooper in)
    {
        in.register(mLoop);
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal)
    {
        if (mDriveControlState != DriveControlState.OPEN_LOOP)
        {
            setBrakeMode(false);

            System.out.println("Switching to open loop");
            System.out.println(signal);
            mDriveControlState = DriveControlState.OPEN_LOOP;
            mLeftMaster.configNeutralDeadband(0.04, 0);
            mRightMaster.configNeutralDeadband(0.04, 0);
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
    }

    /**
     * Configures talons for velocity control
     */
    public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward)
    {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING)
        {
            // We entered a velocity control state.
            setBrakeMode(true);
            mLeftMaster.selectProfileSlot(kVelocityPIDSlot, 0);
            mRightMaster.selectProfileSlot(kVelocityPIDSlot, 0);
            mLeftMaster.configNeutralDeadband(0.0, 0);
            mRightMaster.configNeutralDeadband(0.0, 0);

            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory)
    {
        if (mMotionPlanner != null)
        {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public boolean isDoneWithTrajectory()
    {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING)
        {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    public boolean isBrakeMode()
    {
        return mIsBrakeMode;
    }

    public synchronized void setBrakeMode(boolean on)
    {
        if (mIsBrakeMode != on)
        {
            mIsBrakeMode = on;
            NeutralMode mode = on ? NeutralMode.Brake : NeutralMode.Coast;
            mRightMaster.setNeutralMode(mode);
            mRightSlaveA.setNeutralMode(mode);
            mRightSlaveB.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlaveA.setNeutralMode(mode);
            mLeftSlaveB.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading()
    {
        return mPeriodicIO.gyro_heading;
    }

    public synchronized void setHeading(Rotation2d heading)
    {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    @Override
    public synchronized void stop()
    {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutNumber("rightDistance", mPeriodicIO.right_distance);
        dashboardPutNumber("rightPositionTicks", mPeriodicIO.right_position_ticks);
        dashboardPutNumber("leftPositionTicks", mPeriodicIO.left_position_ticks);
        dashboardPutNumber("leftDistance", mPeriodicIO.left_distance);
        dashboardPutNumber("rightSpeed", getRightLinearVelocity()); // Inches per second
        dashboardPutNumber("leftSpeed", getLeftLinearVelocity()); // Inches per second

        dashboardPutNumber("xError", mPeriodicIO.error.getTranslation().x());
        dashboardPutNumber("yError", mPeriodicIO.error.getTranslation().y());
        dashboardPutNumber("thetaError", mPeriodicIO.error.getRotation().getDegrees());
        if (getHeading() != null)
        {
            dashboardPutNumber("IMU_Heading", getHeading().getDegrees());
        }
        if (mCSVWriter != null)
        {
            mCSVWriter.write();
        }
    }

    public synchronized void resetEncoders()
    {
        mLeftMaster.setSelectedSensorPosition(0, 0, 0);
        mRightMaster.setSelectedSensorPosition(0, 0, 0);
        mPeriodicIO = new PeriodicIO();
    }

    @Override
    public void zeroSensors()
    {
        setHeading(Rotation2d.identity());
        resetEncoders();
    }

    public double getLeftEncoderRotations()
    {
        return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations()
    {
        return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance()
    {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance()
    {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityNativeUnits()
    {
        return mPeriodicIO.right_velocity_ticks_per_100ms;
    }

    public double getRightLinearVelocity()
    {
        return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLeftVelocityNativeUnits()
    {
        return mPeriodicIO.left_velocity_ticks_per_100ms;
    }

    public double getLeftLinearVelocity()
    {
        return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
    }

    public double getLinearVelocity()
    {
        return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
    }

    public double getAngularVelocity()
    {
        return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
    }

    public void overrideTrajectory(boolean value)
    {
        mOverrideTrajectory = value;
    }

    private void updatePathFollower()
    {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING)
        {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory)
            {
                setVelocity(
                        new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
                                radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            }
            else
            {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        }
        else
        {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    public synchronized void reloadGains()
    {
        mLeftMaster.config_kP(kVelocityPIDSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kI(kVelocityPIDSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kD(kVelocityPIDSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_kF(kVelocityPIDSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
        mLeftMaster.config_IntegralZone(kVelocityPIDSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog()
    {
    }

    @Override
    public synchronized void readPeriodicInputs()
    {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);

        double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
        if (deltaLeftTicks > 0.0)
        {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        }
        else
        {
            mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
        }

        double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
        if (deltaRightTicks > 0.0)
        {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        }
        else
        {
            mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
        }

        if (mCSVWriter != null)
        {
            mCSVWriter.add(mPeriodicIO);
        }

        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public synchronized void writePeriodicOutputs()
    {
        if (mDriveControlState == DriveControlState.OPEN_LOOP)
        {
            mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
            mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
        }
        else
        {
            mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
            mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                    mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
        }
    }

    @Override
    public boolean checkSystem()
    {
        boolean leftSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>()
                {

                    {
                        add(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster));
                        add(new TalonSRXChecker.TalonSRXConfig("left_slave", mLeftSlaveA));
                        add(new TalonSRXChecker.TalonSRXConfig("left_slave1", mLeftSlaveB));
                    }
                }, new TalonSRXChecker.CheckerConfig()
                {

                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mLeftMaster.getSelectedSensorVelocity(0);
                    }
                });
        boolean rightSide = TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>()
                {

                    {
                        add(new TalonSRXChecker.TalonSRXConfig("right_master", mRightMaster));
                        add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlaveA));
                        add(new TalonSRXChecker.TalonSRXConfig("right_slave1", mRightSlaveB));
                    }
                }, new TalonSRXChecker.CheckerConfig()
                {

                    {
                        mCurrentFloor = 2;
                        mRPMFloor = 1500;
                        mCurrentEpsilon = 2.0;
                        mRPMEpsilon = 250;
                        mRPMSupplier = () -> mRightMaster.getSelectedSensorVelocity(0);
                    }
                });
        return leftSide && rightSide;
    }

    public synchronized void startLogging()
    {
        if (mCSVWriter == null)
        {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging()
    {
        if (mCSVWriter != null)
        {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    // The robot drivetrain's various states.
    public enum DriveControlState
    {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

    public enum ShifterState
    {
        FORCE_LOW_GEAR,
        FORCE_HIGH_GEAR,
        AUTO_SHIFT
    }

    public static class PeriodicIO
    {

        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
}
