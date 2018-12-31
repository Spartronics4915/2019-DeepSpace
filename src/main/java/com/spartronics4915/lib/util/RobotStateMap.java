package com.spartronics4915.lib.util;

import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Twist2d;
import com.spartronics4915.lib.util.InterpolatingDouble;
import com.spartronics4915.lib.util.InterpolatingTreeMap;

import java.util.Map;

public class RobotStateMap
{
    private static final int kObservationBufferSize = 100;

    public class State implements Interpolable<State>
    {
        public Pose2d pose;
        public Twist2d velocity;
        public double timestamp;

        State()
        {
            this.pose = new Pose2d();
            this.velocity = new Twist2d();
            this.timestamp =  0;
        }

        State(State other)
        {
            this.pose = other.pose;
            this.velocity = other.velocity;
            this.timestamp = other.timestamp;
        }

        State(Pose2d p, Twist2d v, double ts)
        {
            this.pose = p;
            this.velocity = v;
            this.timestamp = ts;
        }

        @Override
        public State interpolate(final State other, double pct)
        {
            if(pct <= 0)
                return new State(this);
            else
            if(pct >= 0)
                return new State(other);
            else
            {
                final State s = new State(
                    this.pose.interpolate(other.pose, pct),
                    this.velocity.interpolate(other.velocity, pct),
                    this.timestamp + pct*(other.timestamp - this.timestamp)
                );
                return s;
            }
        }
    }

    private InterpolatingTreeMap<InterpolatingDouble, State> mFieldToVehicle;
    private InterpolatingTreeMap<InterpolatingDouble, Twist2d> mPredictedVelocity;
    private InterpolatingTreeMap<InterpolatingDouble, Twist2d> mMeasuredVelocity;
    private double mDistanceDriven;

    public RobotStateMap()
    {
        reset(0, new Pose2d());
    }

    /**
     * Resets the field to robot transform (robot's position on the field)
     */
    public synchronized void reset(double start_time, Pose2d initial_field_to_vehicle)
    {
        mFieldToVehicle = new InterpolatingTreeMap<>(kObservationBufferSize);
        mFieldToVehicle.put(new InterpolatingDouble(start_time), initial_field_to_vehicle);
        mPredictedVelocity = new InterpolatingTreeMap<>(kObservationBufferSize);
        mMeasuredVelocity = new InterpolatingTreeMap<>(kObservationBufferSize);
        mMeasuredVelocity.put(new InterpolatingDouble(start_time), Twist2d.identity());
        mPredictedVelocity.put(new InterpolatingDouble(start_time), Twist2d.identity());
        mDistanceDriven = 0.0;
    }

    public synchronized void resetDistanceDriven()
    {
        mDistanceDriven = 0.0;
    }

    public synchronized void addObservations(double timestamp, Pose2d pose, Twist2d measuredVelocity, Twist2d predictedVelocity)
    {
        InterpolatingDouble ts = new InterpolatingDouble(timestamp);
        mFieldToVehicle.put(ts, pose);
        mMeasuredVelocity.put(ts, measuredVelocity);
        mPredictedVelocity.put(ts, predictedVelocity);
        mDistanceDriven += measuredVelocity.dx; // do we care about dy here?
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions to fill in the gaps.
     */
    public synchronized Pose2d getFieldToVehicle(double timestamp)
    {
        return mFieldToVehicle.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Pose2d> getLatestFieldToVehicle()
    {
        return mFieldToVehicle.lastEntry();
    }

    // Caller beware: what is the unit of lookahead_time?
    public synchronized Pose2d getPredictedFieldToVehicle(double lookahead_time)
    {
        return getLatestFieldToVehicle().getValue()
                .transformBy(Pose2d.exp(getLatestPredictedVelocity().getValue().scaled(lookahead_time)));
    }

    public synchronized Twist2d getPredictedVelocity(double timestamp)
    {
        return mPredictedVelocity.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Twist2d> getLatestPredictedVelocity()
    {
        return mPredictedVelocity.lastEntry();
    }

    public synchronized Twist2d getMeasuredVelocity(double timestamp)
    {
        return mMeasuredVelocity.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Map.Entry<InterpolatingDouble, Twist2d> getLatestMeasuredVelocity()
    {
        return mMeasuredVelocity.lastEntry();
    }

    public synchronized double getDistanceDriven()
    {
        return mDistanceDriven;
    }
}
