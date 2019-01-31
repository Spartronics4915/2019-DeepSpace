package com.spartronics4915.lib.drivers;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput.AnalogTriggerType;
import edu.wpi.first.wpilibj.Counter;

/**
 * Driver for an analog Sharp IR sensor (or any distance sensor where output
 * voltage is a function of range, really).
 */
public class IRSensor
{

    private final AnalogInput mAnalogInput;
    private final AnalogTrigger mAnalogTrigger;
    private final Counter mCounter;
    private final double mA;
    private final double mB;
    private final double mC;

    public IRSensor(int port, double min_trigger_distance_centimeters, double max_trigger_disntance_centimeters, int a, int b, int c)
    {
        mA = a;
        mB = b;
        mC = c;

        mAnalogInput = new AnalogInput(port);
        mAnalogInput.setAverageBits(6);
        mAnalogTrigger = new AnalogTrigger(mAnalogInput);
        mAnalogTrigger.setAveraged(true);
        mAnalogTrigger.setFiltered(false);
        mAnalogTrigger.setLimitsVoltage(min_trigger_distance_centimeters, max_trigger_disntance_centimeters);
        mCounter = new Counter(mAnalogTrigger.createOutput(AnalogTriggerType.kState));
    }

    public int getCount()
    {
        return mCounter.get();
    }

    public double getVoltage()
    {
        return mAnalogInput.getAverageVoltage();
    }

    public boolean isTriggered()
    {
        return mAnalogTrigger.getTriggerState();
    }

    public void resetCount()
    {
        mCounter.reset();
    }

    public double getVoltageForDistance(double distance)
    {
        return (Math.log((distance - mA) / mB) / Math.log(Math.E)) / mC;
    }
}