package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        System.out.println("Doing nothing");
    }
}
