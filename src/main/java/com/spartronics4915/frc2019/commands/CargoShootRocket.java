package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoShootRocket extends CommandGroup
{
    public CargoShootRocket()
    {
        addSequential(new ChuteLower());
        addSequential(new CargoShoot());
    }
}
