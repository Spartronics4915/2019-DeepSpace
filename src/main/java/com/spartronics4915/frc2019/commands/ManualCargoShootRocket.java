package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ManualCargoShootRocket extends CommandGroup
{
    public ManualCargoShootRocket()
{
        addSequential(new ManualChuteLower());
        addSequential(new ManualCargoShoot());
    }
}
