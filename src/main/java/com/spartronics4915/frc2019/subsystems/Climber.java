package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

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
    private DoubleSolenoid mFrontClimberSolenoid1 = null;
    private DoubleSolenoid mFrontClimberSolenoid2 = null;
    private DoubleSolenoid mRearClimberSolenoid1 = null;
    private DoubleSolenoid mRearClimberSolenoid2 = null;

    private Climber()
    
    {
        boolean success = true;
        try
        {
            
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

        public boolean mStateChanged = true;
        
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
                        //Climber is disabled (Will be like this until the last 30 seconds of the match)
                        //Make sure tanks are at acceptable levels for climbing (Check before intiating CLIMBING)
                       if (mStateChanged == true)
                       {
                        mFrontClimberSolenoid1.set(Value.kOff);
                        mFrontClimberSolenoid2.set(Value.kOff);
                        mRearClimberSolenoid1.set(Value.kOff);
                        mRearClimberSolenoid2.set(Value.kOff);
                       }
                        break;

                    case CLIMBING:
                        //Struts will extend from their dormant position to allow the robot to reach the height required to get to L3
                        //Must be done when robot is flushed with L3 (Done with distance sensors and a backup encoder reading)
                       if (mStateChanged == true)
                       {
                        mFrontClimberSolenoid1.set(Value.kForward);
                        mFrontClimberSolenoid2.set(Value.kForward);
                        mRearClimberSolenoid1.set(Value.kForward);
                        mRearClimberSolenoid2.set(Value.kForward);
                       }
                        break;
                    
                    case RETRACTING_FRONT_STRUTS:
                        //Solenoids from the front struts will retract when they become flushed with L3
                        //Done with distance sensors and backup driver vision
                        if (mStateChanged == true)
                        {
                        mFrontClimberSolenoid1.set(Value.kReverse);
                        mFrontClimberSolenoid1.set(Value.kReverse);
                        }
                        break;

                    case RETRACTING_REAR_STRUTS:
                        //Solenoids from the rear struts will retract when the robot can support its own weight on L3
                        //Done primarily with driver vision, but distance sensor might be used
                        if (mStateChanged)
                        {
                        mRearClimberSolenoid1.set(Value.kReverse);
                        mRearClimberSolenoid2.set(Value.kReverse);
                        }
                        break;

                    default:
                        logError("Unhandled system state!");
                }
                mSystemState = newState;
                if (newState != mSystemState)
                {
                    mStateChanged = false;
                }
                else
                {
                    mStateChanged = true;
                }
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

            case RETRACT_REAR_STRUTS:
                
                newState = SystemState.RETRACTING_REAR_STRUTS;
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
        logNotice("Lifitng for 5 Seconds");
        mFrontClimberSolenoid1.set(Value.kForward);
        mFrontClimberSolenoid2.set(Value.kForward);
        mRearClimberSolenoid1.set(Value.kForward);
        mRearClimberSolenoid2.set(Value.kForward);
        

        return true;
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