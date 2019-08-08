package com.spartronics4915.frc2019.subsystems;

import com.spartronics4915.lib.subsystems.SpartronicsSubsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive extends SpartronicsSubsystem
{
    private static Drive sInstance = null;

    private DifferentialDrive mDifferentialDrive;

    public static Drive getInstance()
    {
        if (sInstance == null)
        {
            sInstance = new Drive();
        }
        return sInstance;
    }

    private Drive()
    {
        //  TODO: do some things
    }

    @Override
    public void outputTelemetry()
    {

    }

    @Override
    protected void initDefaultCommand()
    {

    }
}
