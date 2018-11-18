package com.spartronics4915.frc2019.lidar;

import com.spartronics4915.frc2019.Constants;
import com.spartronics4915.frc2019.RobotState;
import com.spartronics4915.lib.lidar.icp.ICP;
import com.spartronics4915.lib.lidar.icp.Point;
import com.spartronics4915.lib.lidar.icp.Transform;
import com.spartronics4915.frc2019.loops.Loop;
import com.spartronics4915.lib.geometry.Translation2d;
import com.spartronics4915.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.zip.GZIPOutputStream;

/**
 * Receives LIDAR points from the {@link LidarServer}, stores a set number of
 * scans/revolutions, and provides methods for processing the data.
 * <p>
 * All interfacing with the LIDAR should be done through this class.
 *
 * @see Constants.kLidarNumScansToStore
 * @see doICP()
 * @see getTowerPosition()
 */
public class LidarProcessor implements Loop
{

    private static LidarProcessor mInstance = null;

    public static LidarProcessor getInstance()
    {
        if (mInstance == null)
        {
            mInstance = new LidarProcessor();
        }
        return mInstance;
    }

    private final String kPointCloudDashboardKey = "Lidar/points";

    private RobotState mRobotState = RobotState.getInstance();
    private LidarServer mLidarServer = LidarServer.getInstance();

    private LinkedList<LidarScan> mScans = new LinkedList<>();
    private double prev_timestamp;

    private ICP mICP = new ICP(100); // Timeout of 100 ms

    private DataOutputStream dataLogFile;

    private final ReadWriteLock lock = new ReentrantReadWriteLock();

    private static FileOutputStream newLogFile() throws IOException
    {
        // delete old files if we're over the limit
        File logDir = new File(Constants.kLidarLogDir);
        File[] logFiles = logDir.listFiles();
        if (logFiles == null)
            throw new IOException("List files in " + Constants.kLidarLogDir);
        Arrays.sort(logFiles, (f1, f2) ->
        {
            return Long.compare(f1.lastModified(), f2.lastModified());
        });
        for (int i = 0; i < logFiles.length - Constants.kNumLidarLogsToKeep + 1; i++)
        {
            logFiles[i].delete();
        }

        // create the new file and return
        String dateStr = new SimpleDateFormat("MM-dd-HH_mm_ss").format(new Date());
        File newFile = new File(logDir, "lidarLog-" + dateStr + ".dat");
        newFile.createNewFile();
        return new FileOutputStream(newFile, false);
    }

    private LidarProcessor()
    {
        mScans.add(new LidarScan());
        try
        {
            dataLogFile = new DataOutputStream(new GZIPOutputStream(newLogFile()));
        }
        catch (IOException e)
        {
            System.err.println("Failed to open lidar log file:");
            e.printStackTrace();
        }
    }

    private void logPoint(double angle, double dist, double x, double y)
    {
        try
        {
            dataLogFile.writeInt((int) (angle * 100));
            dataLogFile.writeInt((int) (dist * 256));
            dataLogFile.writeInt((int) (x * 256));
            dataLogFile.writeInt((int) (y * 256));
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public void addPoint(LidarPoint point, boolean newScan)
    {
        SmartDashboard.putNumber("Lidar/angle", point.angle);

        Translation2d cartesian = point.toCartesian();
        logPoint(point.angle, point.distance, cartesian.x(), cartesian.y());

        lock.writeLock().lock();
        try
        {
            if (newScan)
            { // crosses the 360-0 threshold. start a new scan
                prev_timestamp = Timer.getFPGATimestamp();

                SmartDashboard.putString(kPointCloudDashboardKey, "new");
                // long start = System.nanoTime();
                // Translation2d towerPos = getTowerPosition();
                // long end = System.nanoTime();
                // SmartDashboard.putNumber("towerPos_ms", (end-start)/1000000);
                // SmartDashboard.putNumber("towerPosX", towerPos.x());
                // SmartDashboard.putNumber("towerPosY", towerPos.y());

                mScans.add(new LidarScan());
                if (mScans.size() > Constants.kLidarNumScansToStore)
                {
                    mScans.removeFirst();
                }
            }

            if (!excludePoint(cartesian.x(), cartesian.y()))
            {
                getCurrentScan().addPoint(new Point(cartesian), point.timestamp);

                // The point cloud output is relative to the robot's position, so it probably
                // won't look to good if you move the robot around.
                SmartDashboard.putString(kPointCloudDashboardKey, cartesian.x() + " " + cartesian.y());
            }
        }
        finally
        {
            lock.writeLock().unlock();
        }
    }

    private static final double FIELD_WIDTH = 27 * 12, FIELD_HEIGHT = 54 * 12;
    private static final double RECT_RX = FIELD_WIDTH / 5, RECT_RY = FIELD_HEIGHT / 2;
    private static final double FIELD_CX = FIELD_WIDTH / 2, FIELD_CY = FIELD_HEIGHT / 2;
    private static final double RECT_X_MIN = FIELD_CX - RECT_RX, RECT_X_MAX = FIELD_CX + RECT_RX,
            RECT_Y_MIN = FIELD_CY - RECT_RY, RECT_Y_MAX = FIELD_CY + RECT_RY;

    private static boolean excludePoint(double x, double y)
    {
        return x < RECT_X_MIN || x > RECT_X_MAX || y < RECT_Y_MIN || y > RECT_Y_MAX;
    }

    private LidarScan getCurrentScan()
    {
        return mScans.getLast();
    }

    private ArrayList<Point> getAllPoints()
    {
        ArrayList<Point> list = new ArrayList<>();
        for (LidarScan scan : mScans)
        {
            list.addAll(scan.getPoints());
        }
        return list;
    }

    private Point getAveragePoint()
    {
        double sumX = 0, sumY = 0;
        int n = 0;
        for (Point p : getAllPoints())
        {
            sumX += p.x;
            sumY += p.y;
            n++;
        }
        return new Point(sumX / n, sumY / n);
    }

    private static final double BUCKET_SIZE = 3.0; // inches

    /**
     * Cantor pairing function (to bucket & hash two doubles)
     */
    private int getBucket(double x, double y)
    {
        int ix = (int) (x / BUCKET_SIZE);
        int iy = (int) (y / BUCKET_SIZE);
        int a = ix >= 0 ? 2 * ix : -2 * ix - 1;
        int b = iy >= 0 ? 2 * iy : -2 * iy - 1;
        int sum = a + b;
        return sum * (sum + 1) / 2 + a;
    }

    /**
     * Returns a list of points that have been thinned roughly uniformly.
     */
    private ArrayList<Point> getCulledPoints()
    {
        ArrayList<Point> list = new ArrayList<>();
        HashSet<Integer> buckets = new HashSet<>();
        for (Point p : getAllPoints())
        {
            if (buckets.add(getBucket(p.x, p.y)))
                list.add(p);
        }
        return list;
    }

    public Pose2d doICP()
    {
        lock.readLock().lock();
        try
        {
            Pose2d guess = mRobotState.getFieldToLidar(getCurrentScan().getTimestamp());
            Pose2d finalPose = mICP.doICP(getCulledPoints(), new Transform(guess).inverse(), ReferenceModel.TOWER).inverse().toPose2d();
            SmartDashboard.putString("Lidar/pose", finalPose.getTranslation().x() + " " + finalPose.getTranslation().y()
                    + " " + finalPose.getRotation().getDegrees());
            // TODO: Maybe put the processing into its own looper and save past poses (like
            // RobotState)
            return finalPose;
        }
        finally
        {
            lock.readLock().unlock();
        }
    }

    public void setPrevTimestamp(double time)
    {
        lock.writeLock().lock();
        try
        {
            prev_timestamp = time;
        }
        finally
        {
            lock.writeLock().unlock();
        }
    }

    public double getPrevTimestamp()
    {
        lock.readLock().lock();
        try
        {
            return prev_timestamp;
        }
        finally
        {
            lock.readLock().unlock();
        }
    }

    @Override
    public void onStart(double timestamp)
    {
        setPrevTimestamp(Double.NEGATIVE_INFINITY);
    }

    @Override
    public void onLoop(double timestamp)
    {
        if (timestamp - getPrevTimestamp() > Constants.kLidarRestartTime)
        {
            if (!mLidarServer.isEnding() && !mLidarServer.isRunning())
            {
                if (mLidarServer.start())
                {
                    setPrevTimestamp(timestamp);
                    SmartDashboard.putString("Lidar/status", "Server started");
                }
                else
                {
                    SmartDashboard.putString("Lidar/status", "Server couldn't start; sensor " +
                            (mLidarServer.isLidarConnected() ? "is" : "is not") + " connected");
                }
            }
        }
    }

    @Override
    public void onStop(double timestamp)
    {
        mLidarServer.stop();
        SmartDashboard.putString("Lidar/status", "Server stopped");
    }
}
