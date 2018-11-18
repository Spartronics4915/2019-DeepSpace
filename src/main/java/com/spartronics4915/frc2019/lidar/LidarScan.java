package com.spartronics4915.frc2019.lidar;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.lib.lidar.icp.Point;

import java.util.ArrayList;

/**
 * Holds a single 360 degree scan from the lidar
 */
class LidarScan
{

    private ArrayList<Point> points = new ArrayList<>(Constants.kLidarScanSize);
    private double timestamp = 0;

    public String toJsonString()
    {
        String json = "{\"timestamp\": " + timestamp + ", \"scan\": [";
        for (Point point : points)
        {
            json += "{\"x\":" + point.x + ", \"y\":" + point.y + "},";
        }
        json = json.substring(0, json.length() - 1);
        json += "]}";
        return json;
    }

    public String toString()
    {
        String s = "";
        for (Point point : points)
        {
            s += "x: " + point.x + ", y: " + point.y + "\n";
        }
        return s;
    }

    public ArrayList<Point> getPoints()
    {
        return points;
    }

    public double getTimestamp()
    {
        return timestamp;
    }

    public void addPoint(Point point, double time)
    {
        if (timestamp == 0)
        {
            timestamp = time;
        }
        points.add(point);
    }
}
