//  Most of this is commented out - we never had an autonomous climb.

package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class Climb extends CommandGroup
{
    public Climb()
    {
        addSequential(new ClimbExtendAllPneumatics());
        addSequential(new ClimbIntake());
        //  A potential issue here is that the intake needs to keep running after it's time for the back legs to come up.
        addSequential(new ClimbRetractBackPneumatics());
        addSequential(new ClimbRetractFrontPneumatics());
    }
}
