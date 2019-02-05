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
        dashboardPutNumber("Byte yeet", mDriveState.serialSignal.length);
        dashboardPutNumber("Bytes sent", mSerialPort.write(mDriveState.serialSignal, mDriveState.serialSignal.length));
        mSerialPort.flush();
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
        dashboardPutState(mDriveState.toString());
    }

    @Override
    public void stop()
    {
        mDriveState = DriveLEDState.OFF;
        mSerialPort.write(mDriveState.serialSignal, mDriveState.serialSignal.length);
        mSerialPort.close();
    }
}
