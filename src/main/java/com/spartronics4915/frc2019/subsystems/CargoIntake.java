package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

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
        TODO
    }

    private enum SystemState
    {
        TODOING
    }

    private WantedState mWantedState = WantedState.TODO;
    private SystemState mSystemState = SystemState.TODOING;

    private CargoIntake()
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
            synchronized (CargoIntake.this)
            {
                mWantedState = WantedState.TODO;
                mSystemState = SystemState.TODOING;
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
                    case TODOING:
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
            case TODO:
                newState = SystemState.TODOING;
                break;
            default:
                newState = SystemState.TODOING;
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
    }
}
