package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.subsystems.Climber;

import edu.wpi.first.wpilibj.command.Command;

public class ClimbExtendAllPneumatics extends Command
{
    private Climber mClimber;

    public ClimbExtendAllPneumatics()
    {
        mClimber = Climber.getInstance();
        //  Use requires() here to declare subsystem dependencies
        requires(mClimber);
    }

    //  Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        setInterruptible(false);
        setTimeout(1.5);

        mClimber.extendFrontPneumatics();
        mClimber.extendBackPneumatics();

    }

    //  Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        //  Activating pneumatics is instantenous
    }

    //  Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        if (isTimedOut())
            return true;

        return false;
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
