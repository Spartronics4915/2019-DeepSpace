package com.spartronics4915.frc2019.subsystems;

import javax.lang.model.util.ElementScanner6;

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
        DEACTIVATED, FRONT1CLIMB, BACK1CLIMB, FRONT2CLIMB, BACK2CLIMB,
    }

    private enum SystemState
    {
        DEACTIVATING, FRONT1CLIMBING, BACK1CLIMBING, FRONT2CLIMBING, BACK2CLIMBING,
    }

    private WantedState mWantedState = WantedState.DEACTIVATED;
    private SystemState mSystemState = SystemState.DEACTIVATING;

    private Climber()
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
            synchronized (Climber.this)
            {
                mWantedState = WantedState.DEACTIVATED;
                mSystemState = SystemState.DEACTIVATING;
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
                    case FRONT1CLIMBING:
                        break;
                    case BACK1CLIMBING:
                        break;
                    case FRONT2CLIMBING:
                        break;
                    case BACK2CLIMBING:
                        break;
                    case DEACTIVATING:
                        stop();
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
            case DEACTIVATED:
                if(mWantedState == WantedState.FRONT1CLIMB)
                    newState = SystemState.FRONT1CLIMBING;
                if(mWantedState == WantedState.FRONT2CLIMB)
                    newState = SystemState.FRONT1CLIMBING;
                else
                    newState = SystemState.DEACTIVATING;
                break;
            case FRONT1CLIMB:
                if(mWantedState == WantedState.BACK1CLIMB)
                    newState = SystemState.BACK1CLIMBING;
                else
                    newState = SystemState.FRONT1CLIMBING;
                break;
            case BACK1CLIMB:
                if(mWantedState == WantedState.DEACTIVATED)
                    newState = SystemState.DEACTIVATING;
                else
                    newState = SystemState.BACK1CLIMBING;
                break;
            case FRONT2CLIMB:
                if(mWantedState == WantedState.BACK2CLIMB)
                    newState = SystemState.BACK2CLIMBING;
                else
                    newState = SystemState.FRONT2CLIMBING;
                break;
            case BACK2CLIMB:
                if(mWantedState == WantedState.DEACTIVATED)
                    newState = SystemState.DEACTIVATING;
                else
                    newState = SystemState.BACK1CLIMBING;
                break;
            default:
                newState = SystemState.DEACTIVATING;
                break;
        }
        return newState;
    }

    public synchronized void setWantedState(WantedState wantedState)
    {
        mWantedState = wantedState;
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
