package com.spartronics4915.frc2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class InsideFramePerimeter extends CommandGroup
{
    public InsideFramePerimeter()
    {
        addParallel(new CargoManualHold());
        addParallel(new ChuteRaise());
        addParallel(new PanelRetract());
    }
}
