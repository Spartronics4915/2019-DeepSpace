package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.SerialPort;


public class LED extends Subsystem
{

    private SerialPort mSerialPort;

    private static LED mInstance = null;

    public static LED getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new LED();
        }
        return mInstance;
    }

    private enum LEDState
    {
        DISABLING, UPDATING, OFF,
    }

    public enum DriveLEDState
    {
        OFF("0".getBytes()),
        FORWARDS("1".getBytes()),
        BACKWARDS("2".getBytes()),
        AUTONOMOUS("3".getBytes()),
        DISABLED("4".getBytes()); // Add the remainder of your states

        public final byte[] serialSignal;

        private DriveLEDState(byte[] serialSignal)
        {
            this.serialSignal = serialSignal;
        }
    }

    private LEDState mLEDState = LEDState.DISABLING;
    private DriveLEDState mDriveState = DriveLEDState.DISABLED;

    public LED()
    {
        boolean success = true;
        try
        {
            mSerialPort = new SerialPort(9600, SerialPort.Port.kUSB);
        }
        catch (Exception e)
        {
            success = false;
            logException("Couldn't instantiate hardware", e);
        }

        logInitialized(success);
    }

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
            mSerialPort.write(mDriveState.serialSignal, mDriveState.serialSignal.length);
            mLEDState = LEDState.DISABLING;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {
        
    }

    @Override
    public boolean checkSystem(String variant)
    {
        return true;
    }

    @Override
    public void outputTelemetry()
    {

    }

    @Override
    public void stop()
    {
        mDriveState = DriveLEDState.OFF;
        mSerialPort.write(mDriveState.serialSignal, mDriveState.serialSignal.length);
        mLEDState = LEDState.OFF;
    }
}
