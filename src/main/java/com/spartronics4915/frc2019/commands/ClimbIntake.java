//  ClimbIntake or IntakeClimb?
//  Should it be descriptive of the subsystem or the command group?

package com.spartronics4915.frc2019.commands;

import com.spartronics4915.frc2019.subsystems.CargoIntake;

import edu.wpi.first.wpilibj.command.Command;

public class ClimbIntake extends Command
{
    private CargoIntake mCargoIntake;

    public ClimbIntake()
    {
        mCargoIntake = CargoIntake.getInstance();
        //  Use requires() here to declare subsystem dependencies
        requires(mCargoIntake);
    }

    //  Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        //  Stop motors if they (somehow) are running
        mCargoIntake.stop();

        //  Putting the arm down should only be called once
        mCargoIntake.armClimb();
    }

    //  Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        mCargoIntake.intake();
    }

    //  Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished()
    {
        //  XXX: Should stop once the back pneumatic legs are up
        return false;
    }

    //  Called once after isFinished returns true
    @Override
    protected void end()
    {
        mCargoIntake.stop();
    }

    //  Called when another command which requires one or more of the same
    //  subsystems is scheduled to run
    @Override
    protected void interrupted()
    {
    }
}
