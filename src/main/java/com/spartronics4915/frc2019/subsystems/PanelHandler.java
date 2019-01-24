package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class PanelHandler extends Subsystem
{ 
//2 solenoid pincers to secure panels in, 2 to eject panels, also panels held on by velcro
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
        RETRACT, EJECT, OPEN, CLOSE
    }

    private enum SystemState
    {
        RETRACTING, EJECTING, OPENING, CLOSING
    }

    private WantedState mWantedState = WantedState.RETRACT;
    private SystemState mSystemState = SystemState.RETRACTING;

    private static final DoubleSolenoid.Value kSolenoidExtend = DoubleSolenoid.Value.kForward;
    private static final DoubleSolenoid.Value kSolenoidRetract = DoubleSolenoid.Value.kReverse;

    private DoubleSolenoid mSolenoid1 = null;
    private DoubleSolenoid mSolenoid2 = null;
    private DoubleSolenoid mSolenoid3 = null;
    private DoubleSolenoid mSolenoid4 = null;
    //Add Timer

    private PanelHandler()
    {
        boolean success = true;
        try
        {
            //mMotor = new TalonSRX(1);
            mSolenoid1 = new DoubleSolenoid(1, 2);
            mSolenoid2 = new DoubleSolenoid(3, 4);
            mSolenoid3 = new DoubleSolenoid(5, 6);
            mSolenoid4 = new DoubleSolenoid(7, 8);
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
                    case RETRACTING:
                        if (newState != mSystemState)
                        {
                            mSolenoid1.set(kSolenoidRetract);
                            mSolenoid2.set(kSolenoidRetract);
                        }
                        break;
                    case EJECTING:
                        if (newState != mSystemState)
                        {
                            mSolenoid1.set(kSolenoidExtend);
                            mSolenoid2.set(kSolenoidExtend);
                        }
                        break;
                    case OPENING:
                        if (newState != mSystemState)
                        {
                            mSolenoid3.set(kSolenoidRetract);
                            mSolenoid4.set(kSolenoidRetract);
                        }
                        break;
                    case CLOSING:
                        if (newState != mSystemState)
                        {
                            mSolenoid3.set(kSolenoidExtend);
                            mSolenoid4.set(kSolenoidExtend);
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
            synchronized (PanelHandler.this)
            {
                stop();
            }
        }
    };

    private SystemState defaultStateTransfer() //Open -timer-> Eject -timer-> Retract
    {
        SystemState newState = mSystemState;
        switch (mWantedState)
        {
            case RETRACT:
                newState = SystemState.RETRACTING;
                break;
            case EJECT:
                if (mSystemState != SystemState.CLOSING)
                    newState = SystemState.EJECTING;
                break;
            case OPEN:
                newState = SystemState.OPENING;
                break;
            case CLOSE:
                newState = SystemState.CLOSING;
                break;
            default:
                newState = SystemState.RETRACTING;
                break;
        }
        return newState;
    }

    public void automate()
    {
        setWantedState(WantedState.OPEN);
        //Wait
        setWantedState(WantedState.EJECT);
        //Wait
        setWantedState(WantedState.RETRACT);
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
        //Ensure solenoids are functioning, and State Transitions

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
        mSolenoid1.set(kSolenoidRetract);
        mSolenoid2.set(kSolenoidRetract);
        mSolenoid3.set(kSolenoidRetract);
        mSolenoid4.set(kSolenoidRetract);
    }
}
