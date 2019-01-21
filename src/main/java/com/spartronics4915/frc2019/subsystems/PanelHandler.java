package com.spartronics4915.frc2019.subsystems;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.Solenoid;

public class PanelHandler extends Subsystem
{ //Current Idea: 1 Pneumatic to control arm, integrate cargo intake to flick off the panel, limit switch to sense pneumatic is on/off
 //Back-up Idea(Velcro): 2 Pneumatics, 1 to push intake forward, 1 to eject panel, 
    private static PanelHandler mInstance = null;

    public static PanelHandler getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new PanelHandler();
        }
        return mInstance;
    }

    public enum WantedState
    {
        RETRACT, AQUIRE,
    }

    private enum SystemState
    {
        RETRACTING, AQUIRING,
    }

    private WantedState mWantedState = WantedState.RETRACT;
    private SystemState mSystemState = SystemState.RETRACTING;

    //private TalonSRX mMotor = null;

    private static final boolean kSolenoidExtend = true;
    private static final boolean kSolenoidRetract = false;

    private Solenoid mSolenoid = null;

    private PanelHandler()
    {
        boolean success = true;
        try
        {
            //mMotor = new TalonSRX(1);
            mSolenoid = new Solenoid(1);
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
            synchronized (PanelHandler.this)
            {
                mWantedState = WantedState.RETRACT;
                mSystemState = SystemState.RETRACTING;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (PanelHandler.this)
            {
                outputTelemetry();
                SystemState newState = defaultStateTransfer();
                switch (mSystemState)
                {
                    case AQUIRING:
                        //newState = handleAquire();
                        break;
                    case RETRACTING:
                        //newState = handleRetract();
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
            synchronized (PanelHandler.this)
            {
                stop();
            }
        }
    };

    private SystemState defaultStateTransfer()
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {//ask what "switch (mWantedState)" is
            case RETRACT:
                mSolenoid.set(kSolenoidRetract);
                if(mWantedState == WantedState.AQUIRE)
                    newState = SystemState.AQUIRING;
                else
                    newState = SystemState.RETRACTING;
                break;
            case AQUIRE:
                mSolenoid.set(kSolenoidExtend);
                if(mWantedState == WantedState.RETRACT)
                    newState = SystemState.RETRACTING;
                else
                    newState = SystemState.AQUIRING;
                break;
            default:
                newState = SystemState.RETRACTING;
                break;
        }
        return newState;
    }

    /*private SystemState handleAquire()
    {
        if (mWantedState == WantedState.RETRACT)
        {
            return defaultStateTransfer();
        }
        return SystemState.AQUIRING;
    }
    private SystemState handleRetract()
    {
        if (mWantedState == WantedState.AQUIRE)
        {
            return defaultStateTransfer();
        }
        return SystemState.RETRACTING;
    }*/

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
        mSolenoid.set(kSolenoidRetract);
    }
}
