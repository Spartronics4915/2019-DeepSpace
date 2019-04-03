package com.spartronics4915.path.controller;

import com.spartronics4915.frc2019.planners.DriveMotionPlanner;
import com.spartronics4915.lib.geometry.Pose2d;
import com.spartronics4915.lib.geometry.Pose2dWithCurvature;
import com.spartronics4915.lib.geometry.Rotation2d;
import com.spartronics4915.lib.geometry.Translation2d;
import com.spartronics4915.lib.spline.QuinticHermiteSpline;
import com.spartronics4915.lib.spline.Spline;
import com.spartronics4915.lib.spline.SplineGenerator;
import com.spartronics4915.lib.trajectory.TimedView;
import com.spartronics4915.lib.trajectory.TrajectoryIterator;
import com.spartronics4915.lib.trajectory.timing.TimedState;

import org.springframework.web.bind.annotation.*;

import java.net.URLDecoder;
import java.util.ArrayList;

@RestController
@RequestMapping("api")
public class APIController
{

    private DriveMotionPlanner mMotionPlanner;

    public APIController()
    {
        mMotionPlanner = new DriveMotionPlanner();
        mMotionPlanner.setFollowerType(DriveMotionPlanner.FollowerType.FEEDFORWARD_ONLY);
    }

    @RequestMapping(value = "/calculate_splines", method = RequestMethod.POST)
    public @ResponseBody
            String calcSplines(@RequestBody String message)
    {
        message = message.substring(0, message.length() - 1);
        try
        {
            message = URLDecoder.decode(message, "UTF-8");
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        String[] messages = message.split(" ");
        if (messages.length < 2)
            return "no";

        System.out.println(messages[1]);

        ArrayList<Pose2d> points = new ArrayList<>();
        for (String pointString : messages[0].split(";"))
        {
            String[] pointData = pointString.split(",");

            int x = pointData[0].equals("NaN") ? 0 : Integer.parseInt(pointData[0]);
            int y = pointData[1].equals("NaN") ? 0 : Integer.parseInt(pointData[1]);
            int heading = pointData[2].equals("NaN") ? 0 : Integer.parseInt(pointData[2]);

            points.add(new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(heading)));
        }

        String json = "{\"points\":[";

        if (points.size() < 2)
        {
            return "no";
        }
        else
        {
            TrajectoryIterator<TimedState<Pose2dWithCurvature>> traj = new TrajectoryIterator<>(
                    new TimedView<>(
                            (mMotionPlanner.generateTrajectory(messages[1].equals("true"), points, null /* no constraints */, 120, 120, 10))));
            mMotionPlanner.setTrajectory(traj);

            double t = 0.0;
            Pose2dWithCurvature pose = mMotionPlanner.setpoint().state();
            while (!mMotionPlanner.isDone())
            {
                mMotionPlanner.update(t, pose.getPose());
                pose = mMotionPlanner.mSetpoint.state();
                t += 0.01;

                json += poseToJSON(pose) + ",";
            }
        }

        return json.substring(0, json.length() - 1) + "]}";
    }

    private String poseToJSON(Pose2dWithCurvature pose)
    {
        double x = pose.getTranslation().x();
        double y = pose.getTranslation().y();
        double rotation = pose.getRotation().getRadians();
        double curvature = pose.getCurvature();

        return "{\"x\":" + x + ", \"y\":" + y + ", \"rotation\":" + rotation + ", \"curvature\":" + curvature + "}";
    }
}
