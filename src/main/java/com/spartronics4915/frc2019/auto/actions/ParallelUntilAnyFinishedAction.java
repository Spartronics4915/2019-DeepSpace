package com.spartronics4915.frc2019.auto.actions;

import java.util.List;

public class ParallelUntilAnyFinishedAction extends ParallelAction
{
    public ParallelUntilAnyFinishedAction(List<Action> actions)
    {
        super(actions);
    }

    @Override
    public boolean isFinished()
    {
        for (Action action : mActions)
        {
            if (action.isFinished())
                return true;
        }
        return false;
    }
}