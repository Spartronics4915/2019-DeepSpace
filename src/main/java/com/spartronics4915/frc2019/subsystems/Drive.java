package com.spartronics4915.frc2019.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.VisionUpdateManager.HeadingUpdate;
import com.spartronics4915.frc2019.paths.TrajectoryGenerator;
import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.frc2019.planners.DriveMotionPlanner;
import com.spartronics4915.lib.drivers.TalonSRXChecker;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.physics.DifferentialDrive.ChassisState;
import com.spartronics4915.lib.physics.DifferentialDrive.DriveDynamics;
import com.spartronics4915.lib.physics.DifferentialDrive.WheelState;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.timing.TimedState;
import com.spartronics4915.lib.util.DriveSignal;
import com.spartronics4915.lib.util.ReflectingCSVWriter;
import com.spartronics4915.lib.util.Units;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.nio.file.Paths;
import java.util.ArrayList;

public class Drive extends Subsystem
{

    private static Drive mInstance = new Drive();

    // Hardware
    // (By setting these to null, on failure, we end up with a null pointer for now, but if we need to handle hardware
    // absence later we will be able to look at super.isInitialized() to know when to not consume these objects)
    private TalonSRX mLeftMaster = null, mRightMaster = null, mLeftSlave = null, mRightSlave = null;
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
    private double mTargetHeading = 0; // Degrees, for closed-loop turning

    private final ILoop mLoop = new ILoop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (Drive.this)
            {
                setOpenLoop(new DriveSignal(0.05, 0.05));
                setBrakeMode(Constants.kDefaultBrakeMode);
                // startLogging();
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
                    case VELOCITY:
                        updateVelocity();
                        break;
                    case TURN:
                        break;
                    default:
                        logError("Unexpected drive control state: " + mDriveControlState);
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
        final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 100); //primary closed-loop, 100 ms timeout
        if (sensorPresent != ErrorCode.OK)
        {
            logError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent);
        }
        talon.setInverted(!left);
        talon.setSensorPhase(!Constants.kIsTestChassis);
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

        boolean success = true;
        try
        {
            // Start all Talons in open loop mode.
            mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kLeftDriveMasterId);
            configureMaster(mLeftMaster, true);

            mLeftSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kLeftDriveSlaveAId,
                    Constants.kLeftDriveMasterId);
            mLeftSlave.setInverted(false);

            mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kRightDriveMasterId);
            configureMaster(mRightMaster, false);

            mRightSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kRightDriveSlaveAId,
                    Constants.kRightDriveMasterId);
            mRightSlave.setInverted(true);

            reloadGains(mRightMaster);
            reloadGains(mLeftMaster);

            mLeftMaster.configNeutralDeadband(Constants.kDriveLeftDeadband, 0);
            mRightMaster.configNeutralDeadband(Constants.kDriveRightDeadband, 0);

            mPigeon = Constants.kIsTestChassis ? new PigeonIMU(mLeftSlave) : new PigeonIMU(Constants.kPidgeonId);
            mLeftSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

            setOpenLoop(DriveSignal.NEUTRAL);

            // Force a CAN message across.
            mIsBrakeMode = false;
            setBrakeMode(Constants.kDefaultBrakeMode);
        }
        catch (Exception e)
        {
            logException("ERROR Drive couldn't initialize hardware", e);
            success = false;
        }

        mMotionPlanner = new DriveMotionPlanner();

        logInitialized(success);
    }

    public static Drive getInstance()
    {
        return mInstance;
    }

    private static double rotationsToInches(double rotations)
    {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesToRotations(double inches)
    {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second)
    {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double inchesPerSecondToTicksPer100ms(double ips)
    {
        return ((ips * Constants.kDriveEncoderPPR) / (Constants.kDriveWheelDiameterInches * Math.PI)) / 10;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s)
    {
        return rad_s / (Math.PI * 2.0) * Constants.kDriveEncoderPPR / 10.0;
    }

    private static double ticksPer100msToInchesPerSecond(double t)
    {
        return (t / Constants.kDriveEncoderPPR) * 10 * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    @Override
    public void registerEnabledLoops(ILooper in)
    {
        in.register(mLoop);
    }

    public synchronized void setWantTurn(Rotation2d heading)
    {
        if (mDriveControlState != DriveControlState.TURN)
        {
            logDebug("Switching to robot relative turn: " + heading);
            mDriveControlState = DriveControlState.TURN;
            updateTalonsForPosition();
        }
        mTargetHeading = heading.getDegrees();

        WheelState w = mMotionPlanner.getModel().solveInverseKinematics(new ChassisState(0.0, heading.getRadians()));
        mPeriodicIO.leftDemand = (getLeftEncoderRotations() + inchesToRotations(Units.meters_to_inches(w.left / 10))) * Constants.kDriveEncoderPPR;
        mPeriodicIO.rightDemand = (getRightEncoderRotations() + inchesToRotations(Units.meters_to_inches(w.right / 10))) * Constants.kDriveEncoderPPR;
    }

    /**
     * Configure talons for open loop control
     */
    public synchronized void setOpenLoop(DriveSignal signal)
    {
        if (mDriveControlState != DriveControlState.OPEN_LOOP)
        {
            logDebug("Switching to open loop: " + signal.toString());
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        setBrakeMode(signal.getBrakeMode());

        mPeriodicIO.leftDemand = signal.getLeft();
        mPeriodicIO.rightDemand = signal.getRight();
        mPeriodicIO.leftFeedforward = 0.0;
        mPeriodicIO.rightFeedforward = 0.0;
    }

    /**
     * Configures talons for velocity control with paths
     */
    private synchronized void setPathVelocity(DriveSignal signal, DriveSignal feedforward)
    {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING)
        {
            // This branch is never entered because the control state should already be set by setTrajectory

            logDebug("Switching to path following");
            // We entered a velocity control state.
            updateTalonsForVelocity();
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        // In velocity-Control mode:
        //      demand is measured in ticksPer100ms
        //      feedforward is measured in pctVbus [0,1]
        mPeriodicIO.leftDemand = signal.getLeft();
        mPeriodicIO.rightDemand = signal.getRight();
        mPeriodicIO.leftFeedforward = feedforward.getLeft();
        mPeriodicIO.rightFeedforward = feedforward.getRight();
    }

    /**
     * Configure talons for velocity control without paths
     *
     * @param inchesPerSecVelocity Desired velocity in inches per second
     * @param feedforwardVoltage   Voltage (0-12) to apply arbitrarily to velocity
     *                             PID output
     */
    public synchronized void setVelocity(DriveSignal inchesPerSecVelocity, DriveSignal feedforwardVoltage)
    {
        if (mDriveControlState != DriveControlState.VELOCITY)
        {
            logDebug("Switching to closed loop velocity. Target: " +
                    inchesPerSecVelocity.toString() +
                    ", Arbitrary feedforward: " + feedforwardVoltage.toString());
            updateTalonsForVelocity();
            mDriveControlState = DriveControlState.VELOCITY;
        }
        mPeriodicIO.leftDemand = inchesPerSecondToTicksPer100ms(inchesPerSecVelocity.getLeft());
        mPeriodicIO.rightDemand = inchesPerSecondToTicksPer100ms(inchesPerSecVelocity.getRight());
        mPeriodicIO.leftFeedforward = feedforwardVoltage.getLeft() / 12;
        mPeriodicIO.rightFeedforward = feedforwardVoltage.getRight() / 12;
        mPeriodicIO.leftAccel = mPeriodicIO.rightAccel = 0;
    }

    public void curveTowardsVisionTarget(HeadingUpdate.TargetInfo targetInfo)
    {
        DriveDynamics w = mMotionPlanner.getModel().solveInverseDynamics(
                new ChassisState(getLinearVelocity(), getAngularVelocity()), new ChassisState(targetInfo.heightError * Constants.kDriveVisionHeightKp,
                        targetInfo.headingError.getRadians() * Constants.kDriveVisionHeadingKp));
        setVelocity(new DriveSignal(w.wheel_velocity.left, w.wheel_velocity.right),
                new DriveSignal(w.voltage.left, w.voltage.right));
    }

    private void updateTalonsForVelocity()
    {
        setBrakeMode(true);
        mLeftMaster.selectProfileSlot(Constants.kVelocityPIDSlot, 0);
        mRightMaster.selectProfileSlot(Constants.kVelocityPIDSlot, 0);
    }

    private void updateTalonsForPosition()
    {
        setBrakeMode(true);
        mLeftMaster.selectProfileSlot(Constants.kPositionPIDSlot, 0);
        mRightMaster.selectProfileSlot(Constants.kPositionPIDSlot, 0);
    }

    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory)
    {
        if (mMotionPlanner != null)
        {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
            updateTalonsForVelocity();
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

    public boolean isDoneTurning()
    {
        return mDriveControlState == DriveControlState.TURN &&
                Math.abs(mTargetHeading - mPeriodicIO.gyroHeading.getDegrees()) < Constants.kTurnDegreeTolerance &&
                Math.abs(getLeftLinearVelocity()) < Constants.kTurnVelTolerance &&
                Math.abs(getRightLinearVelocity()) < Constants.kTurnVelTolerance;
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
            mRightSlave.setNeutralMode(mode);

            mLeftMaster.setNeutralMode(mode);
            mLeftSlave.setNeutralMode(mode);
        }
    }

    public synchronized Rotation2d getHeading()
    {
        return mPeriodicIO.gyroHeading;
    }

    public synchronized double[] getAccumGyro()
    {
        return mPeriodicIO.gyroYPRAccum;
    }

    public synchronized void setHeading(Rotation2d heading)
    {
        logDebug("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
        logDebug("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyroHeading = heading;
    }

    @Override
    public synchronized void stop()
    {
        setOpenLoop(DriveSignal.NEUTRAL);
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutNumber("rightDistance", mPeriodicIO.rightDistance);
        dashboardPutNumber("rightPositionTicks", mPeriodicIO.rightPositionTicks);
        dashboardPutNumber("leftPositionTicks", mPeriodicIO.leftPositionTicks);
        dashboardPutNumber("leftDistance", mPeriodicIO.leftDistance);
        dashboardPutNumber("rightSpeed", getRightLinearVelocity()); // Inches per second
        dashboardPutNumber("leftSpeed", getLeftLinearVelocity()); // Inches per second

        dashboardPutNumber("xError", mPeriodicIO.error.getTranslation().x());
        dashboardPutNumber("yError", mPeriodicIO.error.getTranslation().y());
        dashboardPutNumber("thetaError", mPeriodicIO.error.getRotation().getDegrees());
        dashboardPutNumber("pitch", mPeriodicIO.gyroYPRAccum[2]);
        if (getHeading() != null)
        {
            dashboardPutNumber("imuHeading", getHeading().getDegrees());
        }
        if (mCSVWriter != null)
        {
            mCSVWriter.write();
        }

        dashboardPutNumber("leftDemand", mPeriodicIO.leftDemand);
        dashboardPutNumber("rightDemand", mPeriodicIO.rightDemand);
        if (mDriveControlState == DriveControlState.VELOCITY || mDriveControlState == DriveControlState.PATH_FOLLOWING)
        {
            dashboardPutNumber("leftSpeedTarget", ticksPer100msToInchesPerSecond(mPeriodicIO.leftDemand));
            dashboardPutNumber("rightSpeedTarget", ticksPer100msToInchesPerSecond(mPeriodicIO.rightDemand));
            dashboardPutNumber("leftFeedforward", mPeriodicIO.leftFeedforward);
            dashboardPutNumber("rightFeedforward", mPeriodicIO.rightFeedforward);
        }

        dashboardPutState(mDriveControlState.toString());
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
        return mPeriodicIO.leftPositionTicks / Constants.kDriveEncoderPPR;
    }

    public double getRightEncoderRotations()
    {
        return mPeriodicIO.rightPositionTicks / Constants.kDriveEncoderPPR;
    }

    public double getLeftEncoderDistance()
    {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance()
    {
        return rotationsToInches(getRightEncoderRotations());
    }

    public double getRightVelocityTicksPer100ms()
    {
        return mPeriodicIO.rightVelocityTicksPer100ms;
    }

    // in/sec
    public double getRightLinearVelocity()
    {
        return rotationsToInches(getRightVelocityTicksPer100ms() * 10.0 / Constants.kDriveEncoderPPR);
    }

    public double getRightOutputVoltage()
    {
        return mPeriodicIO.rightVoltage;
    }

    public double getLeftVelocityTicksPer100ms()
    {
        return mPeriodicIO.leftVelocityTicksPer100ms;
    }

    // in/sec
    public double getLeftLinearVelocity()
    {
        return rotationsToInches(getLeftVelocityTicksPer100ms() * 10.0 / Constants.kDriveEncoderPPR);
    }

    public double getLeftOutputVoltage()
    {
        return mPeriodicIO.leftVoltage;
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

            DriveMotionPlanner.Output output =
                    mMotionPlanner.update(now, RobotStateEstimator.getInstance().getEncoderRobotStateMap().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.pathSetpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory)
            {
                setPathVelocity(
                        new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
                                radiansPerSecondToTicksPer100ms(output.right_velocity)),
                        new DriveSignal(output.left_feedforward_voltage / 12.0,
                                output.right_feedforward_voltage / 12.0));

                // if accel is rads per sec^2, why divide by 1000 (and not 100?)
                mPeriodicIO.leftAccel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.rightAccel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            }
            else
            {
                setPathVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.leftAccel = mPeriodicIO.rightAccel = 0.0;
            }
        }
        else
        {
            DriverStation.reportError("Drive is not in path following state", false);
        }
    }

    private void updateVelocity()
    {
        if (mDriveControlState == DriveControlState.VELOCITY)
        {
        }
        else
        {
            DriverStation.reportError("Drive is not in velocity control state", false);
        }

    }

    public synchronized void reloadGains(TalonSRX talon)
    {
        talon.config_kP(Constants.kVelocityPIDSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
        talon.config_kI(Constants.kVelocityPIDSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
        talon.config_kD(Constants.kVelocityPIDSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
        talon.config_kF(Constants.kVelocityPIDSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
        talon.config_IntegralZone(Constants.kVelocityPIDSlot, Constants.kDriveVelocityIZone, Constants.kLongCANTimeoutMs);

        talon.config_kP(Constants.kPositionPIDSlot, Constants.kDrivePositionKp, Constants.kLongCANTimeoutMs);
        talon.config_kI(Constants.kPositionPIDSlot, Constants.kDrivePositionKi, Constants.kLongCANTimeoutMs);
        talon.config_kD(Constants.kPositionPIDSlot, Constants.kDrivePositionKd, Constants.kLongCANTimeoutMs);
        talon.config_kF(Constants.kPositionPIDSlot, Constants.kDrivePositionKf, Constants.kLongCANTimeoutMs);
        talon.config_IntegralZone(Constants.kPositionPIDSlot, Constants.kDrivePositionIZone, Constants.kLongCANTimeoutMs);
    }

    @Override
    public void writeToLog()
    {
    }

    @Override
    public void readPeriodicInputs()
    {
        synchronized (mPeriodicIO)
        {
            double prevLeftTicks = mPeriodicIO.leftPositionTicks;
            double prevRightTicks = mPeriodicIO.rightPositionTicks;
            mPeriodicIO.leftPositionTicks = mLeftMaster.getSelectedSensorPosition(0);
            mPeriodicIO.rightPositionTicks = mRightMaster.getSelectedSensorPosition(0);
            mPeriodicIO.leftVelocityTicksPer100ms = mLeftMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.rightVelocityTicksPer100ms = mRightMaster.getSelectedSensorVelocity(0);
            mPeriodicIO.gyroHeading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);
            mPigeon.getAccumGyro(mPeriodicIO.gyroYPRAccum);
            mPeriodicIO.leftVoltage = mLeftMaster.getMotorOutputVoltage();
            mPeriodicIO.rightVoltage = mRightMaster.getMotorOutputVoltage();
            double deltaLeftTicks = ((mPeriodicIO.leftPositionTicks - prevLeftTicks) / Constants.kDriveEncoderPPR) * Math.PI;

            if (deltaLeftTicks > 0.0) // XXX: Why do we have this if statement? (And the corresponding one for the right side)
            {
                mPeriodicIO.leftDistance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
            }
            else
            {
                mPeriodicIO.leftDistance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
            }

            double deltaRightTicks = ((mPeriodicIO.rightPositionTicks - prevRightTicks) / Constants.kDriveEncoderPPR) * Math.PI;
            if (deltaRightTicks > 0.0)
            {
                mPeriodicIO.rightDistance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
            }
            else
            {
                mPeriodicIO.rightDistance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
            }
        }

        if (mCSVWriter != null)
        {
            mCSVWriter.add(mPeriodicIO);
        }

        // System.out.println("control state: " + mDriveControlState + ", left: " + mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
    }

    @Override
    public void writePeriodicOutputs()
    {
        synchronized (mPeriodicIO)
        {
            /*
             * NB: this is where the rubber hits the road. ie: here are direct
             * controls over the talon to power the drive train. The rest of
             * this file orchestrates the periodic updateing of our inputs:
             * mDriveControlState
             * mPeriodicIO.left_demand, right_demand (pct or vel or pos)
             * mPeriodicIO.left_accel, right_accel (used with *Kd* in velocity mode)
             */
            if (mDriveControlState == DriveControlState.OPEN_LOOP)
            {
                mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.leftDemand, DemandType.ArbitraryFeedForward, 0.0);
                mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.rightDemand, DemandType.ArbitraryFeedForward, 0.0);
            }

            else if (mDriveControlState == DriveControlState.TURN)
            {
                mLeftMaster.set(ControlMode.Position, mPeriodicIO.leftDemand);
                mRightMaster.set(ControlMode.Position, mPeriodicIO.rightDemand);
            }
            else
            {
                // In ControlMode.Velocity:
                //      * demand is measured in ticksPer100ms
                //      * feedforward is measured in pctVbus [0,1]
                // Note: we employ a localized PD controller on the arbitrary
                //  feedforward term. kDriveVelocityKd combines with accel since
                //  accel is the derivative of velocity.
                // Remember that there is path-following controller that sits
                //  atop this call and that in that context the term feedfwd
                //  controller refers to an open-loop path follower.
                //  (fair warning).
                mLeftMaster.set(ControlMode.Velocity,
                        mPeriodicIO.leftDemand,
                        DemandType.ArbitraryFeedForward,
                        mPeriodicIO.leftFeedforward +
                                Constants.kDriveVelocityKd * mPeriodicIO.leftAccel / 1023.0);
                mRightMaster.set(ControlMode.Velocity,
                        mPeriodicIO.rightDemand,
                        DemandType.ArbitraryFeedForward,
                        mPeriodicIO.rightFeedforward +
                                Constants.kDriveVelocityKd * mPeriodicIO.rightAccel / 1023.0);
                //
                // Following can be used to debug the feedback term without the
                // Talon-side PID.
                //
                // mLeftMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward,
                //      mPeriodicIO.leftFeedforward/* + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0*/);
                // mRightMaster.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward,
                //      mPeriodicIO.rightFeedforward/* + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0*/);
            }
        }
    }

    @Override
    public boolean checkSystem(String variant)
    {
        if (!isInitialized())
        {
            logWarning("can't check uninitialized system");
            return false;
        }
        boolean success = true;
        DriveSignal zero = new DriveSignal(0, 0);
        DriveSignal feedfwd = zero;
        logNotice("checkSystem " + variant + " ---------------");
        if (variant.equals("velocityControl"))
        {
            Timer timer = new Timer();
            logNotice("  enter velocity control mode");
            logNotice("  straight: 10ips, 4 sec");
            this.setVelocity(new DriveSignal(10, 10), feedfwd);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }
            this.setVelocity(zero, zero);
            this.outputTelemetry();
            this.stop();

            Timer.delay(2);
            logNotice("  straight: -10ips, 4 sec");
            this.setVelocity(new DriveSignal(-10, -10), feedfwd);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }
            this.setVelocity(zero, zero);
            this.outputTelemetry();
            this.stop();

            Timer.delay(2);
            logNotice("  curve left: 4 sec");
            this.setVelocity(new DriveSignal(3, 5), feedfwd);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }
            this.setVelocity(zero, zero);
            this.outputTelemetry();
            this.stop();

            Timer.delay(2);
            logNotice("  curve right: 4 sec");
            this.setVelocity(new DriveSignal(5, 3), feedfwd);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(4))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }
            this.setVelocity(zero, zero);
            this.outputTelemetry();
            this.stop();

            Timer.delay(2);
            logNotice("  mixed speeds straight: 10, 4, 20, 4");
            this.setVelocity(new DriveSignal(10, 10), feedfwd);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }

            this.setVelocity(new DriveSignal(4, 4), feedfwd);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }

            timer.reset();
            timer.start();
            this.setVelocity(new DriveSignal(20, 20), feedfwd);
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }

            this.setVelocity(new DriveSignal(4, 4), feedfwd);
            timer.reset();
            timer.start();
            while (!timer.hasPeriodPassed(2))
            {
                Timer.delay(.05);
                this.outputTelemetry();
            }
            this.stop();
            this.setVelocity(zero, zero);
            this.outputTelemetry();
        }
        else
        {
            boolean leftSide = TalonSRXChecker.CheckTalons(this,
                    new ArrayList<TalonSRXChecker.TalonSRXConfig>()
                    {

                        {
                            add(new TalonSRXChecker.TalonSRXConfig("left_master", mLeftMaster));
                            add(new TalonSRXChecker.TalonSRXConfig("left_slave", mLeftSlave));
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
                            add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlave));
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
            success = leftSide && rightSide;
        }
        return success;
    }

    public synchronized void startLogging()
    {
        if (mCSVWriter == null)
        {
            mCSVWriter = new ReflectingCSVWriter<>(Paths.get(System.getProperty("user.home"), "DRIVE-LOGS.csv").toString(), PeriodicIO.class);
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
        PATH_FOLLOWING, // velocity PID control from a path
        VELOCITY, // constant velocity from not a path
        TURN, // turning to a field-relative heading
    }

    public static class PeriodicIO
    {

        // INPUTS
        public int leftPositionTicks;
        public int rightPositionTicks;
        public double leftDistance;
        public double rightDistance;
        public int leftVelocityTicksPer100ms;
        public int rightVelocityTicksPer100ms;
        public double leftVoltage;
        public double rightVoltage;
        public Rotation2d gyroHeading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        public double[] gyroYPRAccum = new double[3];

        // OUTPUTS
        public double leftDemand;
        public double rightDemand;
        public double leftAccel;
        public double rightAccel;
        public double leftFeedforward;
        public double rightFeedforward;
        public TimedState<Pose2dWithCurvature> pathSetpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }
}
