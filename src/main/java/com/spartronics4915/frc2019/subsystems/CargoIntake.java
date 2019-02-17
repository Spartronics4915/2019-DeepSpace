package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.drivers.A41IRSensor;
import com.spartronics4915.lib.drivers.IRSensor;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.util.CANProbe;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CargoIntake extends Subsystem
{

    private static CargoIntake mInstance = null;

    public static CargoIntake getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new CargoIntake();
        }
        return mInstance;
    }

    public enum WantedState
    {
        HOLD, ARM_DOWN, INTAKE, EJECT, CLIMB
    }

    private enum SystemState
    {
        HOLDING, ARM_DOWNING, INTAKING, EJECTING, CLIMBING
    }

    private WantedState mWantedState = WantedState.HOLD;
    private SystemState mSystemState = SystemState.HOLDING;

    private static final boolean kSolenoidExtend = true;
    private static final boolean kSolenoidRetract = false;
    private static final double kIntakeSpeed = -0.5;
    private static final double kEjectSpeed = 0.5;
    private static final double kIntakeClimbSpeed = -0.5;

    private Solenoid mSolenoid = null;
    private Solenoid mSolenoidClimb = null;
    private TalonSRX mMotorRight = null;
    private TalonSRX mMotorLeft = null;
    private IRSensor mSensor = null;

    private boolean mStateChanged;

    private CargoIntake()
    {
        boolean success = true; // IR sensor anolog port 7 to detect cargo going into chute
        try
        {
            if (!CANProbe.getInstance().validatePCMId(Constants.kCargoHatchArmPCMId))
                throw new RuntimeException("CargoIntake PCM isn't on the CAN bus!");

            mMotorRight = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeMotorRight);
            mMotorLeft = TalonSRXFactory.createDefaultTalon(Constants.kCargoIntakeMotorLeft);
            mSolenoid = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kCargoIntakeSolenoid);
            mSolenoidClimb = new Solenoid(Constants.kCargoHatchArmPCMId, Constants.kCargoIntakeSolenoidClimb);
            mSensor = new A41IRSensor(Constants.kCargoIntakeSensor);
        }
        catch (Exception e)
        {
            success = false;
            logException("Couldn't instantiate hardware", e);
        }

        logInitialized(success);
    }

    private final ILoop mLoop = new ILoop()
    {

        @Override
        public void onStart(double timestamp)
        {
            synchronized (CargoIntake.this)
            {
                mWantedState = WantedState.HOLD;
                mSystemState = SystemState.HOLDING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (CargoIntake.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case HOLDING://DS5
                        if (mStateChanged)
                        {
                            mMotorRight.set(ControlMode.PercentOutput, 0.0);
                            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
                            setSolenoidsToUp();
                        }
                        break;
                    case ARM_DOWNING:
                        if (mStateChanged)
                        {
                            setSolenoidsToDown();
                            mMotorRight.set(ControlMode.PercentOutput, 0.0);
                            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
                        }
                        break;
                    case INTAKING://BB2
                        if (mStateChanged)
                        {
                            setSolenoidsToDown();
                            mMotorRight.set(ControlMode.PercentOutput, kIntakeSpeed);
                            mMotorLeft.set(ControlMode.PercentOutput, kIntakeSpeed);
                        }
                        break;
                    case EJECTING://BB3
                        if (mStateChanged)
                        {
                            setSolenoidsToDown();
                            mMotorRight.set(ControlMode.PercentOutput, kEjectSpeed);
                            mMotorLeft.set(ControlMode.PercentOutput, kEjectSpeed);
                        }
                        break;
                    case CLIMBING:
                        if (mStateChanged)
                        {
                            mMotorRight.set(ControlMode.PercentOutput, kIntakeClimbSpeed);
                            mMotorLeft.set(ControlMode.PercentOutput, kIntakeClimbSpeed);
                            mSolenoid.set(kSolenoidExtend);
                            mSolenoidClimb.set(kSolenoidExtend);
                        }
                        break;
                    default:
                        logError("Unhandled system state!");
                }
                if (newState != mSystemState)
                {
                    mStateChanged = true;
                    logNotice("System state to " + newState);
                }
                else
                    mStateChanged = false;
                mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (CargoIntake.this)
            {
                stop();
            }
        }
    };

    private void setSolenoidsToUp()
    {
        mSolenoid.set(kSolenoidRetract);
        mSolenoidClimb.set(kSolenoidRetract);
    }

    private void setSolenoidsToDown()
    {
        mSolenoid.set(kSolenoidExtend);
        mSolenoidClimb.set(kSolenoidRetract);
    }

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case HOLD:
                newState = SystemState.HOLDING;
                break;
            case ARM_DOWN:
                newState = SystemState.ARM_DOWNING;
                break;
            case INTAKE:
                newState = SystemState.INTAKING;
                break;
            case EJECT:
                newState = SystemState.EJECTING;
                break;
            case CLIMB:
                newState = SystemState.CLIMBING;
                break;
            default:
                newState = SystemState.HOLDING;
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
    }

    public synchronized boolean atTarget()
    {
        switch (mWantedState)
        {
            case HOLD:
                return mSystemState == SystemState.HOLDING;
            case ARM_DOWN:
                return mSystemState == SystemState.ARM_DOWNING;
            case INTAKE:
                return mSystemState == SystemState.INTAKING && mSensor.isTargetInDistanceRange(Constants.kCargoIntakeSensorMinDistance, Constants.kCargoIntakeSensorMaxDistance);
            case EJECT:
                return mSystemState == SystemState.EJECTING && mSensor.isTargetInDistanceRange(Constants.kCargoIntakeSensorMinDistance, Constants.kCargoIntakeSensorMaxDistance);
            case CLIMB:
                return mSystemState == SystemState.CLIMBING;
            default:
                logError("atTarget for unknown wanted state " + mWantedState);
                return false;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        enabledLooper.register(mLoop);
    }

    @Override
    public boolean checkSystem(String variant)
    {
        logNotice("Starting CargoIntake Solenoid Check");
        try
        {
            logNotice("Extending solenoids for 2 seconds");
            mSolenoid.set(kSolenoidExtend);
            mSolenoidClimb.set(kSolenoidExtend);
            Timer.delay(2);
            logNotice("Retracting solenoids for 2 seconds");
            setSolenoidsToUp();
            Timer.delay(2);
            logNotice("CargoIntake Solenoid Check End");
            logNotice("Running motors at 50% for 2 seconds");
            mMotorRight.set(ControlMode.PercentOutput, kIntakeSpeed);
            mMotorLeft.set(ControlMode.PercentOutput, kIntakeSpeed);
            Timer.delay(2);
            logNotice("Running motors at 0% for 2 seconds");
            mMotorRight.set(ControlMode.PercentOutput, 0.0);
            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
            Timer.delay(2);
            logNotice("Running motors at -50% for 2 seconds");
            mMotorRight.set(ControlMode.PercentOutput, kEjectSpeed);
            mMotorLeft.set(ControlMode.PercentOutput, kEjectSpeed);
            Timer.delay(2);
            mMotorRight.set(ControlMode.PercentOutput, 0.0);
            mMotorLeft.set(ControlMode.PercentOutput, 0.0);
            logNotice("CargoIntake Motor Check End");
        }
        catch (Exception e)
        {
            logException("Trouble instantiating hardware ", e);
            return false;
        }
        return true;
    }

    @Override
    public void outputTelemetry()
    {
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
        dashboardPutBoolean("mSolenoid Extended", mSolenoid.get());
        dashboardPutBoolean("mSolenoidClimb Extended", mSolenoidClimb.get());
        dashboardPutNumber("mMotor1 Speed", mMotorRight.getMotorOutputPercent());
        dashboardPutNumber("mMotor2 Speed", mMotorLeft.getMotorOutputPercent());
        dashboardPutNumber("Distance from cargo", mSensor.getDistance());
        dashboardPutBoolean("Is cargo obtained", mSensor.isTargetInDistanceRange(Constants.kCargoIntakeSensorMinDistance, Constants.kCargoIntakeSensorMaxDistance));
    }

    @Override
    public void stop() // TODO: This may interfere with the CargoChute
    {
        mSolenoid.set(kSolenoidRetract);
        mSolenoidClimb.set(kSolenoidRetract);
        mMotorRight.set(ControlMode.PercentOutput, 0.0);
        mMotorLeft.set(ControlMode.PercentOutput, 0.0);
    }
}