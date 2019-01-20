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

    public enum WantedState
    {
        CLOSED, INTAKE,
    }

    private enum SystemState
    {
        CLOSING, INTAKING,
    }

    private WantedState mWantedState = WantedState.CLOSED;
    private SystemState mSystemState = SystemState.CLOSING;

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
                mWantedState = WantedState.CLOSED;
                mSystemState = SystemState.CLOSING;
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
                    case INTAKING:
                        break;
                    case CLOSING:
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
            case CLOSED:
                newState = SystemState.CLOSING;
                break;
            case INTAKE:
                newState = SystemState.INTAKING;
                break;
            default:
                newState = SystemState.CLOSING;
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
