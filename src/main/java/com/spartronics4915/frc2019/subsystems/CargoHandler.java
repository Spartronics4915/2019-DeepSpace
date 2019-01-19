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
        INTAKE, RELEASE_ROCKET, RELEASE_SHIP, HOLD,
    }

    private enum SystemState
    {
        INTAKING, ASCENDING, DESCENDING, HOLDING,

    }

    private WantedState mWantedState = WantedState.HOLD;
    private SystemState mSystemState = SystemState.HOLDING;

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
                mWantedState = WantedState.HOLD;
                mSystemState = SystemState.HOLDING;
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
                    case ASCENDING:
                        break;
                    case DESCENDING:
                        break;
                    case HOLDING:
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
            case HOLD:
                newState = SystemState.HOLDING;
                break;
            case INTAKE:
                newState = SystemState.INTAKING;
                break;
            case RELEASE_ROCKET:
                newState = SystemState.ASCENDING;
                break;
            case RELEASE_SHIP:
                newState = SystemState.DESCENDING;
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
