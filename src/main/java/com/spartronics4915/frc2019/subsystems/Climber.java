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
        CLIMB,
        RETRACT_FRONT_STRUTS,
        RETRACT_REAR_STRUTS,
    }

    private enum SystemState
    {
        DISABLING, 
        CLIMBING,
        RETRACTING_FRONT_STRUTS,
        RETRACTING_REAR_STRUTS,
    }

    private WantedState mWantedState = WantedState.DISABLED;
    private SystemState mSystemState = SystemState.DISABLING;
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
                mSystemState = SystemState.DISABLING;
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
                    case DISABLING:
                        //Climber is disabled
                        mMotor.set(ControlMode.PercentOutput, 0.0);
                        stop();
                        break;
                    case CLIMBING:
                        //Solenoids will rasie the robot to the angle required to get to Level 3
                        mMotor.set(ControlMode.PercentOutput, 0.8);
                        break;
                    case RETRACTING_FRONT_STRUTS:
                        //Robot retracts the front 2 struts for when we reach the level we need to climb to
                        mMotor.set(ControlMode.PercentOutput, -0.7);
                        break;
                    case RETRACTING_REAR_STRUTS:
                        //Robot retracts the rear 2 struts when it is fully supported by the platform of level 3
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
                newState = SystemState.DISABLING;
                break;
            case CLIMB:
                newState = SystemState.CLIMBING;
                break;
            case RETRACT_FRONT_STRUTS:
                newState = SystemState.RETRACTING_FRONT_STRUTS;
                break;
            default:
                newState = SystemState.DISABLING;
                logNotice("Robot is in an Unhandled Wanted State!");
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
