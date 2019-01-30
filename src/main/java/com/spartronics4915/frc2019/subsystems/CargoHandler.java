package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CargoHandler extends Subsystem
{

    private static CargoHandler mInstance = null;

    public static CargoHandler getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new CargoHandler();
        }
        return mInstance;
    }

    public enum WantedState // Each WantedState will correspond to a button
    {
        MANUAL_RAMP, // Runs while held down; Ignores sensors
        EJECT_OUT, // Runs while held down; Button shared with CargoIntake
        SHOOT_BAY, // Checks if pneumatic down, then shoots
        SHOOT_ROCKET, // Checks if pneumatic up, then shoots
    }

    private enum SystemState
    {
        HOLDING, // Ramp runs until cargo reaches top
        EJECTING, // Ramp runs in reverse
        SHOOTING, // Timed or sensor??
        IDLING, // No cargo
    }

    private WantedState mWantedState = WantedState.MANUAL_RAMP;
    private SystemState mSystemState = SystemState.IDLING;

    private CargoHandler()
    {
        boolean success = true;
        try
        {
            // Instantiate your hardware here
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
            synchronized (CargoHandler.this)
            {
                mWantedState = WantedState.MANUAL_RAMP;
                mSystemState = SystemState.IDLING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (CargoHandler.this)
            {
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case HOLDING:
                        break;
                    case EJECTING:
                        break;
                    case SHOOTING:
                        break;
                    case IDLING:
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
            synchronized (CargoHandler.this)
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
            case MANUAL_RAMP:
                newState = SystemState.HOLDING;
                break;
            case EJECT_OUT:
                newState = SystemState.EJECTING;
                break;
            case SHOOT_BAY:
                newState = SystemState.SHOOTING;
                break;
            case SHOOT_ROCKET:
                newState = SystemState.SHOOTING;
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
    }

    @Override
    public void stop()
    {
        // Stop your hardware here
        mWantedState = WantedState.MANUAL_RAMP;
        mSystemState = SystemState.IDLING;
    }
}
