package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.CargoChute;

import edu.wpi.first.wpilibj.command.Command;

public class ManualChuteLower extends Command
{
    private CargoChute mCargoChute;

    public ManualChuteLower()
    {
        mCargoChute = CargoChute.getInstance();
        //  Use requires() here to declare subsystem dependencies
        requires(mCargoChute);
    }

    //  Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        setInterruptible(false);
        setTimeout(Constants.kChuteLowRetractTime);

        //  XXX: Does this cancel the command?
        if (mCargoChute.isChuteDown())
            return;
        mCargoChute.lower();
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
