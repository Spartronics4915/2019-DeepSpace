package com.spartronics4915.frc2019;

import com.spartronics4915.frc2019.subsystems.*;
import com.spartronics4915.frc2019.subsystems.PanelHandler;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot
{
    private Drive mDrive;
    private CargoChute mCargoChute;
    private CargoIntake mCargoIntake;
    private Climber mClimber;
    private PanelHandler mPanelHandler;
    private LED mLED;
    private OI mOI;

    @Override
    public void robotInit()
    {
        mDrive = Drive.getInstance();
        mCargoChute = CargoChute.getInstance();
        mCargoIntake = CargoIntake.getInstance();
        mClimber = Climber.getInstance();
        mPanelHandler = PanelHandler.getInstance();
        mLED = LED.getInstance();
        mOI = new OI();
    }

    @Override
    public void robotPeriodic()
    {
    }

    @Override
    public void autonomousInit()
    {
    }

    @Override
    public void autonomousPeriodic()
    {
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void testInit()
    {
    }

    @Override
    public void testPeriodic()
    {
    }

    @Override
    public void disabledInit()
    {
    }

    @Override
    public void disabledPeriodic()
    {
    }

    /*
    private void outputAllToSmartDashboard()
    {
    }
    */
}
