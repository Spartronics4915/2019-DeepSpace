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
        ON, OFF,
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

    private LEDState mLEDState = LEDState.ON;
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
        if (mLEDState == LEDState.ON)
        {
            mSerialPort.write(driveState.serialSignal, driveState.serialSignal.length);
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
