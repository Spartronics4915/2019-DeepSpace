package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.SerialPort;


public class LED extends Subsystem
{

    private SerialPort mBling;

    private static LED mInstance = null;

    public static LED getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new LED();
        }
        return mInstance;
    }

    // private enum SubsytemLEDState
    // {
    //     CLIMBING, 
    // }

    private final byte[] kForwards = "2".getBytes();
    private final byte[] kBackwards = "3".getBytes();
    private final byte[] kOff = "4".getBytes();
    private final byte[] kAutonomous = "5".getBytes();

    private enum LEDState
    {
        DISABLING, UPDATING,
    }

    public enum DriveLEDState
    {
        FORWARDS, BACKWARDS, AUTONOMOUS, DISABLED
    }

    private LEDState mLEDState = LEDState.DISABLING;
    private DriveLEDState mDriveState = DriveLEDState.DISABLED;

    public LED()
    {
        boolean success = true;
        try
        {
            mBling = new SerialPort(9600, SerialPort.Port.kUSB);
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
            synchronized (LED.this)
            {
                mDriveState = DriveLEDState.DISABLED;
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            synchronized (LED.this)
            {
                // SystemState newState = defaultStateTransfer();
                // switch (mSystemState)
                // {
                //     case INTAKING:
                //         break;
                //     case CLOSING:
                //         stop();
                //         break;
                //     default:
                //         logError("Unhandled system state!");
                // }
                // mSystemState = newState;
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            synchronized (LED.this)
            {
                stop();
            }
        }
    };

    public synchronized void setDriveState(DriveLEDState driveState)
    {
        mDriveState = driveState;
        mLEDState = LEDState.UPDATING;
    }

    public synchronized void setDriveState() 
    {
        if (mDriveState == DriveLEDState.FORWARDS)
        {
            mDriveState = DriveLEDState.BACKWARDS;
            mLEDState = LEDState.UPDATING;
        } else if (mDriveState == DriveLEDState.BACKWARDS)
        {
            mDriveState = DriveLEDState.FORWARDS;
            mLEDState = LEDState.UPDATING;
        }
    }

    public synchronized void updateBling() 
    {
        if (mLEDState == LEDState.UPDATING)
        {
            switch (mDriveState)
            {
                case FORWARDS:
                    mBling.write(kForwards, kForwards.length);
                    break;
                case BACKWARDS:
                    mBling.write(kBackwards, kBackwards.length);
                    break;
                case AUTONOMOUS:
                    mBling.write(kAutonomous, kAutonomous.length);
                    break;
                default:
                    mBling.write(kOff, kOff.length);
                    break;
            }
            mLEDState = LEDState.DISABLING;
        }
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
