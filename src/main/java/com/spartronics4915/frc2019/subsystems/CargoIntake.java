package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.drivers.TalonSRXFactory;
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
    private static final double kIntakeSpeed = 0.5;
    private static final double kEjectSpeed = -0.5;
    private static final double kIntakeClimbSpeed = -0.5;


    private Solenoid mSolenoid = null;
    private Solenoid mSolenoidClimb = null;

    private TalonSRX mMotor1 = null;
    private TalonSRX mMotor2 = null;

    private boolean mStateChanged;

    private CargoIntake()
    {
        boolean success = true;
        try
        {
            mSolenoid = new Solenoid(1);
            mSolenoidClimb = new Solenoid(2);
            mMotor1 = TalonSRXFactory.createDefaultTalon(5);
            mMotor2 = TalonSRXFactory.createDefaultTalon(6);
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
                    case HOLDING:
                        if (mStateChanged)
                        {
                            mMotor1.set(ControlMode.PercentOutput, 0);
                            mMotor2.set(ControlMode.PercentOutput, 0);
                            mSolenoid.set(kSolenoidRetract);
                            mSolenoidClimb.set(kSolenoidRetract);
                        }
                        break;
                    case ARM_DOWNING:
                        if (mStateChanged)
                        {
                            mSolenoid.set(kSolenoidExtend);
                            mSolenoidClimb.set(kSolenoidRetract);
                        }
                        setWantedState(WantedState.INTAKE);
                        break;
                    case INTAKING://transition to ARM_UPING using proximity sensor
                        if (mStateChanged)
                        {
                            mMotor1.set(ControlMode.PercentOutput, kIntakeSpeed);
                            mMotor2.set(ControlMode.PercentOutput, kIntakeSpeed);
                        }
                        break;
                    case EJECTING://transition to holding using proximity sensor
                        if (mStateChanged)
                        {
                            mMotor1.set(ControlMode.PercentOutput, kEjectSpeed);
                            mMotor2.set(ControlMode.PercentOutput, kEjectSpeed);
                        }
                        break;
                    case CLIMBING:
                        if (mStateChanged)
                        {
                            mMotor1.set(ControlMode.PercentOutput, kIntakeClimbSpeed);
                            mMotor2.set(ControlMode.PercentOutput, kIntakeClimbSpeed);
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
                return mSystemState == SystemState.INTAKING;
            case EJECT:
                return mSystemState == SystemState.EJECTING;
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
            mSolenoid.set(kSolenoidRetract);
            mSolenoidClimb.set(kSolenoidRetract);
            Timer.delay(2);
            logNotice("CargoIntake Solenoid Check End");
            logNotice("Running motors at 50% for 2 seconds");
            mMotor1.set(ControlMode.PercentOutput, kIntakeSpeed);
            mMotor2.set(ControlMode.PercentOutput, kIntakeSpeed);
            Timer.delay(2);
            logNotice("Running motors at 0% for 2 seconds");
            mMotor1.set(ControlMode.PercentOutput, 0);
            mMotor2.set(ControlMode.PercentOutput, 0);
            Timer.delay(2);
            logNotice("Running motors at -50% for 2 seconds");
            mMotor1.set(ControlMode.PercentOutput, kEjectSpeed);
            mMotor2.set(ControlMode.PercentOutput, kEjectSpeed);
            Timer.delay(2);
            mMotor1.set(ControlMode.PercentOutput, 0);
            mMotor2.set(ControlMode.PercentOutput, 0);
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
        dashboardPutNumber("mMotor1 Speed", mMotor1.getMotorOutputPercent());
        dashboardPutNumber("mMotor2 Speed", mMotor2.getMotorOutputPercent());
    }

    @Override
    public void stop()
    {
        mSolenoid.set(kSolenoidRetract);
        mSolenoidClimb.set(kSolenoidRetract);
        mMotor1.set(ControlMode.PercentOutput, 0);
        mMotor2.set(ControlMode.PercentOutput, 0);
    }
}
