package com.spartronics4915.lib.drivers;

import edu.wpi.first.wpilibj.AnalogInput;

public class IRSensor
{

    AnalogInput mAnalogInput;
    final double kTriggerVoltage = 1.1;

    public IRSensor(int port)
    {
        mAnalogInput = new AnalogInput(port);
        mAnalogInput.setAverageBits(6);
    }

    public double getVoltage()
    {
        double v = mAnalogInput.getAverageVoltage();
        if (v < .001)
            v = .001;
        return v;
    }

    public double getDistance() // inches
    {
        // formula for sharp a41 detector, each model has a different formula
        //      v = 1 / (L + .42)  (1/cm)
        // 
        //double cm = 1.0 / getVoltage() - .42; // warning blows up when v == 0
        double volt = getVoltage();
        double cm = 59.06 - (94.11 * volt) + (57.60 * Math.pow(volt, 2.0)) - (11.65 * Math.pow(volt, 3.0));
        return cm / 2.54;
    }

    public boolean isTargetInVoltageRange(double min, double max)
    {
        double v = getVoltage();
        if (v > min && v < max)
            return true;
        else
            return false;
    }

    /**
     * @param minDist in inches
     * @param maxDist in inches
     * @return is within the distance
     */
    public boolean isTargetInDistanceRange(double minDist, double maxDist)
    {
        double d = getDistance();
        if (d > minDist && d < maxDist)
            return true;
        else
            return false;
    }
    
    public boolean isTargetAcquired()
    {
        double voltage = getVoltage();
        if (voltage > kTriggerVoltage)
            return true;
        else
            return false;
    }
}