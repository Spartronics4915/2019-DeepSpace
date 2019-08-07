package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.subsystems.PanelHandler;

import edu.wpi.first.wpilibj.command.Command;

public class PanelRetract extends Command
{
    private PanelHandler mPanelHandler;

    public PanelRetract()
    {
        mPanelHandler = PanelHandler.getInstance();
        //  Use requires() here to declare subsystem dependencies
        requires(mPanelHandler);
    }

    //  Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        setInterruptible(false);

        mPanelHandler.retract();
    }

    //  Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        //  Intentionally left blank
    }

    //  Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        //  Instantly finished, because solenoids + it's not that important
        return true;
    }

    //  Called once after isFinished returns true
    @Override
    protected void end()
    {
    }

    //  Called when another command which requires one or more of the same
    //  subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
    }
}
