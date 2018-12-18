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

    public SubsystemManager(List<Subsystem> allSubsystems)
    {
        mAllSubsystems = allSubsystems;
    }

    public void outputToTelemetry()
    {
        mAllSubsystems.forEach((s) -> s.outputTelemetry());
    }

    public void writeToLog()
    {
        mAllSubsystems.forEach((s) -> s.writeToLog());
    }

    public void stop()
    {
        mAllSubsystems.forEach((s) -> s.stop());
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
        mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
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
