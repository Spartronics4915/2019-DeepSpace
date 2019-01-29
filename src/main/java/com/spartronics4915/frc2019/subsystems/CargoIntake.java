package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;

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
        HOLD, ARM_DOWN, ARM_UP, INTAKE, EJECT, CLIMB
    }

    private enum SystemState
    {
        HOLDING, ARM_DOWNING, ARM_UPING, INTAKING, EJECTING, CLIMBING
    }

    private WantedState mWantedState = WantedState.HOLD;
    private SystemState mSystemState = SystemState.HOLDING;

    private static final boolean kSolenoidExtend = true;
    private static final boolean kSolenoidRetract = false;

    private Solenoid mSolenoid = null;
    private Solenoid mSolenoidClimb = null;

    private TalonSRX mMotor1 = null;
    private TalonSRX mMotor2 = null;

    private CargoIntake()
    {
        boolean success = true;
        try
        {
            mSolenoid = new Solenoid(1);
            mSolenoidClimb = new Solenoid(2);
            mMotor1 = new TalonSRX(5);
            mMotor2 = new TalonSRX(6);
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
                        if (newState != mSystemState){
                            mSolenoid.set(kSolenoidRetract);
                            mSolenoidClimb.set(kSolenoidRetract);
                            mMotor1.set(ControlMode.PercentOutput, 0);
                            mMotor2.set(ControlMode.PercentOutput, 0);
                        }
                        break;
                    case ARM_DOWNING:
                        if (newState != mSystemState){
                            mSolenoid.set(kSolenoidExtend);
                            mSolenoidClimb.set(kSolenoidRetract);
                        }
                        break;
                    case ARM_UPING:
                        if (newState != mSystemState){
                            mSolenoid.set(kSolenoidRetract);
                            mSolenoidClimb.set(kSolenoidRetract);
                        }
                        break;
                    case INTAKING:
                        if (newState != mSystemState){
                            mMotor1.set(ControlMode.PercentOutput, 0.5);
                            mMotor2.set(ControlMode.PercentOutput, 0.5);
                        }
                        break;
                    case EJECTING:
                        if (newState != mSystemState){
                            mMotor1.set(ControlMode.PercentOutput, -0.5);
                            mMotor2.set(ControlMode.PercentOutput, -0.5);
                        }
                        break;
                    case CLIMBING:
                        if (newState != mSystemState)
                        {
                            mSolenoid.set(kSolenoidExtend);
                            mSolenoidClimb.set(kSolenoidExtend);
                        }
                        break;
                    default:
                        logError("Unhandled system state!");
                }
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
            case ARM_UP:
                newState = SystemState.ARM_UPING;
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
            case ARM_UP:
                return mSystemState == SystemState.ARM_UPING;
            case INTAKE:
                return mSystemState == SystemState.INTAKING;
            case EJECT:
                return mSystemState == SystemState.EJECTING;
            case CLIMB:
                return mSystemState == SystemState.CLIMBING;
            default:
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
        return false;
    }

    @Override
    public void outputTelemetry()
    {
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
    }
}
