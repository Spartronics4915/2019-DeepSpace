package com.spartronics4915.lib.drivers;

public class A21IRSensor extends IRSensor 
{
    public A21IRSensor(int port)
    {
        super(port);
    }

    @Override
    public double getDistance()
    {
        double volt = getVoltage();
        double cm = 59.06 - (94.11 * volt) + (57.60 * Math.pow(volt, 2.0)) - (11.65 * Math.pow(volt, 3.0));
        return cm / 2.54;
    }

}