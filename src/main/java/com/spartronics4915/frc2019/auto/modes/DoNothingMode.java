package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.lib.util.Logger;

public class DoNothingMode extends AutoModeBase
{

    @Override
    protected void routine() throws AutoModeEndedException
    {
        Logger.debug("Doing nothing");
    }
}
