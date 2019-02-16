package com.spartronics4915.frc2019.auto.modes;

import com.spartronics4915.frc2019.auto.AutoModeBase;
import com.spartronics4915.frc2019.auto.AutoModeEndedException;
import com.spartronics4915.frc2019.auto.actions.DetermineWheelbaseDiameter;

public class FindEffectiveWheelbaseDiameter extends AutoModeBase
{

@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new DetermineWheelbaseDiameter());
    }
    
}