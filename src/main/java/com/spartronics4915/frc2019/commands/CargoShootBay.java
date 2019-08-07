package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoShootBay extends CommandGroup
{
    public CargoShootBay()
    {
        addSequential(new ChuteRaise());
        addSequential(new CargoShoot());
    }
}
