package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.subsystems.CargoChute;

import edu.wpi.first.wpilibj.command.Command;

public class CargoShoot extends Command
{
    private CargoChute mCargoChute;

    public CargoShoot()
    {
        mCargoChute = CargoChute.getInstance();
        //  Use requires() here to declare subsystem dependencies
        requires(mCargoChute);
    }

    //  Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        setInterruptible(true);
        setTimeout(Constants.kShootTime);
    }

    //  Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        mCargoChute.shoot();
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
        mCargoChute.stop();
    }

    //  Called when another command which requires one or more of the same
    //  subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
        mCargoChute.stop();
    }
}