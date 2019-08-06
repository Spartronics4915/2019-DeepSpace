package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class ClimbRetractFrontPneumatics extends Command
{
    public ClimbRetractFrontPneumatics()
    {
        //  Use requires() here to declare subsystem dependencies
        requires(Robot.mClimber);
    }

    //  Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
    }

    //  Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        Robot.mClimber.retractFrontPneumatics();
    }

    //  Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        //  XXX: Only needs to run execute() once?
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
