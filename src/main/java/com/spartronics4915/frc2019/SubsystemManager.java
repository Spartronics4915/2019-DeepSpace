package com.spartronics4915.frc2019;

import com.spartronics4915.lib.util.ILooper;
import com.spartronics4915.lib.util.ILoop;
import com.spartronics4915.frc2019.subsystems.Subsystem;

import java.util.ArrayList;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper
{

    private final List<Subsystem> mAllSubsystems;
    private List<ILoop> mLoops = new ArrayList<>();
    private int mTelemetrySelector = 0;

    public SubsystemManager(List<Subsystem> allSubsystems)
    {
        mAllSubsystems = allSubsystems;
    }

    public void outputToTelemetry(boolean roundRobin)
    {
        // To reduce looptime spent in telemetry, we optionally round-robin 
        // telemetry requests to each subystem.  
        // For 10 sub subsystems with 20ms looptimes, we get updates from
        // each subsystem every 200ms.
        int nSys = 0;
        for(int i=0;i<mAllSubsystems.size();i++)
        {
            Subsystem s = mAllSubsystems.get(i);
            if (s.isInitialized())
            {
                if(roundRobin)
                {
                    if(nSys++ == this.mTelemetrySelector)
                        s.outputTelemetry();
                }
                else
                    s.outputTelemetry();

            }
        }
        this.mTelemetrySelector++;
        if(this.mTelemetrySelector == nSys)
            this.mTelemetrySelector = 0;
    }

    public void writeToLog()
    {
        mAllSubsystems.forEach((s) ->
        {
            if (s.isInitialized())
                s.writeToLog();
        });
    }

    public void stop()
    {
        mAllSubsystems.forEach((s) ->
        {
            if (s.isInitialized())
                s.stop();
        });
    }

    private class EnabledLoop implements ILoop
    {

        @Override
        public void onStart(double timestamp)
        {
            for (ILoop l : mLoops)
            {
                l.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp)
        {
            for (Subsystem s : mAllSubsystems)
            {
                s.readPeriodicInputs();
            }
            for (ILoop l : mLoops)
            {
                l.onLoop(timestamp);
            }
            for (Subsystem s : mAllSubsystems)
            {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp)
        {
            for (ILoop l : mLoops)
            {
                l.onStop(timestamp);
            }
        }
    }

    private class DisabledLoop implements ILoop
    {

        @Override
        public void onStart(double timestamp)
        {

        }

        @Override
        public void onLoop(double timestamp)
        {
            for (Subsystem s : mAllSubsystems)
            {
                s.readPeriodicInputs();
            }
            for (Subsystem s : mAllSubsystems)
            {
                s.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp)
        {

        }
    }

    public void registerEnabledLoops(ILooper enabledLooper)
    {
        mAllSubsystems.forEach((s) ->
        {
            if (s.isInitialized())
                s.registerEnabledLoops(this);
        });
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(ILooper disabledLooper)
    {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(ILoop loop)
    {
        mLoops.add(loop);
    }
}
