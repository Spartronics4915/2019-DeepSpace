package com.spartronics4915.frc2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.drivers.TalonSRXFactory;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

public class Climber extends Subsystem
{

    private static Climber mInstance = null;

    public static Climber getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public enum WantedState
    {
        DISABLED, 
        CLIMB_TO_TWO,
        CLIMB_TO_THREE,
        BACKRISE,
    }

    private enum SystemState
    {
        DISABLED, 
        CLIMBING_TO_TWO,
        CLIMBING_TO_THREE,
        BACKRAISING,
    }

    private WantedState mWantedState = WantedState.DISABLED;
    private SystemState mSystemState = SystemState.DISABLED;
    private TalonSRX mMotor = null;

    private Climber()
    
    {
        boolean success = true;
        try
        {
            mMotor = TalonSRXFactory.createDefaultTalon(Constants.kTurretMotorId);
            mMotor.setNeutralMode(NeutralMode.Brake);       
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
            synchronized (Climber.this)
            {
                mWantedState = WantedState.DISABLED;
                mSystemState = SystemState.DISABLED;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (Climber.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case CLIMBING_TO_TWO:
                        //Solenoids will get the robot to the point where it can climb to Level 2
                        mMotor.set(ControlMode.PercentOutput, 0.4);
                        break;
                    case DISABLED:
                        //Climber is disabled
                        mMotor.set(ControlMode.PercentOutput, 0.0);
                        stop();
                        break;
                    case CLIMBING_TO_THREE:
                        //Solenoids will rasie the robot to the angle required to get to Level 3
                        mMotor.set(ControlMode.PercentOutput, 0.8);
                        break;
                    case BACKRAISING:
                        mMotor.set(ControlMode.PercentOutput, -0.7);
                        //In case the robot requires just the back to be raised, this System State is here
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
            synchronized (Climber.this)
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
            case DISABLED:
                newState = SystemState.DISABLED;
                break;
            case CLIMB_TO_TWO:
                newState = SystemState.CLIMBING_TO_TWO;
                break;
            case CLIMB_TO_THREE:
                newState = SystemState.CLIMBING_TO_THREE;
                break;
            case BACKRISE:
                newState = SystemState.BACKRAISING;
                break;
            default:
                newState = SystemState.DISABLED;
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
        return true;
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
        dashboardPutState(mSystemState.toString());
        dashboardPutWantedState(mWantedState.toString());
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
    }
}
