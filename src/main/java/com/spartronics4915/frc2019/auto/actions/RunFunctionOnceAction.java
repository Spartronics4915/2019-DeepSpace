package com.spartronics4915.frc2019.auto.actions;

import java.util.function.Function;

public class RunFunctionOnceAction extends RunOnceAction
{

    private final Runnable mFunction;

    public RunFunctionOnceAction(Runnable func)
    {
        mFunction = func;
    }

    @Override
    public void runOnce()
    {
        mFunction.run();
    }

}
