package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.drivers.IRSensor;
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
        DISABLE, CLIMB, RETRACT_FRONT_STRUTS, RETRACT_REAR_STRUTS,
    }

    private enum SystemState
    {
        DISABLING, CLIMBING, RETRACTING_FRONT_STRUTS, RETRACTING_REAR_STRUTS,
    }

    private WantedState mWantedState = WantedState.DISABLE;
    private SystemState mSystemState = SystemState.DISABLING;
    private DoubleSolenoid mFrontLeftClimberSolenoid = null; //Port
    private DoubleSolenoid mFrontRightClimberSolenoid = null; //Starboard
    private DoubleSolenoid mRearLeftClimberSolenoid = null; //Port
    private DoubleSolenoid mRearRightClimberSolenoid = null; //Starboard
    public IRSensor mFrontRightIRSensor = null; //Starboard
    public IRSensor mFrontLeftIRSensor = null; //Port

    private Climber()

    {
        boolean success = true;
        try
        {
            mFrontLeftClimberSolenoid = new DoubleSolenoid(Constants.kClimberPWMId, Constants.kFrontPortSolenoidId1, Constants.kFrontPortSolenoidId2);
            mFrontRightClimberSolenoid = new DoubleSolenoid(Constants.kClimberPWMId, Constants.kFrontStarboardSolenoidId1, Constants.kFrontStarboardSolenoidId2);
            mRearLeftClimberSolenoid = new DoubleSolenoid(Constants.kClimberPWMId, Constants.kRearPortSolenoidId1, Constants.kRearPortSolenoidId2);
            mRearRightClimberSolenoid = new DoubleSolenoid(Constants.kClimberPWMId, Constants.kRearStarboardSolenoidId1, Constants.kRearStarboardSolenoidId2);
            mFrontRightIRSensor = new IRSensor(Constants.kFrontPortIRSensorId);
            mFrontLeftIRSensor = new IRSensor(Constants.kFrontStarboardIRSensorId);
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
                mWantedState = WantedState.DISABLE;
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
                        // Climber is disabled (Will be like this until the last 30 seconds of the
                        // match)
                        // Make sure tanks are at acceptable levels for climbing (Check before intiating
                        // CLIMBING)
                        if (mStateChanged == true)
                        {
                            mFrontLeftClimberSolenoid.set(Value.kOff);
                            mFrontRightClimberSolenoid.set(Value.kOff);
                            mRearLeftClimberSolenoid.set(Value.kOff);
                            mRearRightClimberSolenoid.set(Value.kOff);
                        }
                        break;

                    case CLIMBING:
                        // Struts will extend from their dormant position to allow the robot to reach
                        // the height required to get to L3
                        // Must be done when robot is flushed with L3 (Done with distance sensors and a
                        // backup encoder reading)
                        if (mStateChanged == true)
                        {
                            mFrontLeftClimberSolenoid.set(Value.kForward);
                            mFrontRightClimberSolenoid.set(Value.kForward);
                            mRearLeftClimberSolenoid.set(Value.kForward);
                            mRearRightClimberSolenoid.set(Value.kForward);
                        }
                        break;

                    case RETRACTING_FRONT_STRUTS:
                        // Solenoids from the front struts will retract when they become flushed with L3
                        // Done with distance sensors and backup driver vision
                        if (mStateChanged == true)
                        {
                            mFrontLeftClimberSolenoid.set(Value.kReverse);
                            mFrontLeftClimberSolenoid.set(Value.kReverse);
                        }
                        break;

                    case RETRACTING_REAR_STRUTS:
                        // Solenoids from the rear struts will retract when the robot can support its
                        // own weight on L3
                        // Done primarily with driver vision, but distance sensor might be used
                        if (mStateChanged)
                        {
                            mRearLeftClimberSolenoid.set(Value.kReverse);
                            mRearRightClimberSolenoid.set(Value.kReverse);
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
            case DISABLE:

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
        mFrontLeftClimberSolenoid.set(Value.kForward);
        mFrontRightClimberSolenoid.set(Value.kForward);
        mRearLeftClimberSolenoid.set(Value.kForward);
        mRearRightClimberSolenoid.set(Value.kForward);

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
