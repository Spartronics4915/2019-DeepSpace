package com.spartronics4915.frc2019.auto;

import com.spartronics4915.frc2019.auto.actions.Action;
import com.spartronics4915.lib.util.Logger;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This
 * is implemented in auto modes (which are
 * routines that do actions).
 */
public abstract class AutoModeBase
{

    protected double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run()
    {
        mActive = true;

        try
        {
            routine();
        }
        catch (AutoModeEndedException e)
        {
            mActive = false;
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }

        done();
    }

    public void done()
    {
        mActive = false;
        Logger.info("Auto mode done");
    }

    public void stop()
    {
        mActive = false;
    }

    public boolean isActive()
    {
        return mActive;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException
    {
        if (!isActive())
        {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException
    {
        isActiveWithThrow();
        action.start();

        while (isActiveWithThrow() && !action.isFinished())
        {
            action.update();
            long waitTime = (long) (mUpdateRate * 1000.0);

            try
            {
                Thread.sleep(waitTime);
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }

        action.done();
    }
}
