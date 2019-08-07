package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

public class LED extends SpartronicsSubsystem
{
    private static LED mInstance = null;

    public static LED getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED()
    {
        boolean success = true;
        try
        {
            //  Instantiate your hardware here
            success = true;
        }
        catch (Exception e)
        {
            success = false;
            logException("Couldn't instantiate hardware", e);
        }

        logInitialized(success);
    }

    @Override
    public void initDefaultCommand()
    {
    }

    public boolean checkSystem(String variant)
    {
        return false;
    }

    @Override
    public void outputTelemetry()
    {

    }
}
