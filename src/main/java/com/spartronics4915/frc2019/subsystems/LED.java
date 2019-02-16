package com.spartronics4915.frc2019.subsystems;

import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Arrays;

import com.spartronics4915.lib.util.ILooper;

import edu.wpi.first.wpilibj.SerialPort;


public class LED extends Subsystem
{

    // private SerialPort mSerialPort;
    private FileOutputStream mSerialPort;

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
            // mSerialPort = new SerialPort(9600, SerialPort.Port.kUSB);
            //mSerialPort = new FileOutputStream("/dev/ttyACM0");
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
        // try
        // {
        //     mSerialPort.write(driveState.serialSignal);
        // }
        // catch(IOException e)
        // {
        //     logException("We didn't think this would work anyways, but we did hope",  e);
        // }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper)
    {}

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
    {}
}
