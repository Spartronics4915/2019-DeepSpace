package com.spartronics4915.frc2019.auto.actions;

import java.util.function.Function;
import java.util.function.Supplier;

public class RunFunctionOnceUntilAction implements Action
{

    private final Runnable mOnceFunc;
    private final Supplier<Boolean> mIsFinishedFunc;
    private boolean mHasStarted = false;

    public RunFunctionOnceUntilAction(Runnable onceFunc, Supplier<Boolean> isFinishedFunc)
    {
        mOnceFunc = onceFunc;
        mIsFinishedFunc = isFinishedFunc;
    }

    @Override
    public boolean isFinished()
    {
        return mIsFinishedFunc.get() && mHasStarted;
    }

    @Override
    public void update()
    {

    }

    @Override
    public void done()
    {

    }

    @Override
    public void start()
    {
        mOnceFunc.run();
        mHasStarted = true;
    }

}
