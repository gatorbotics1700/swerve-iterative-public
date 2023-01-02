import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

import javax.xml.namespace.QName;//check what this is

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Trajectory {//they call trajectoryGenerator method on mp goals template example (in other words, revisit this)
    public void generateTrajectory(){//return value?
        Pose2d start  = new Pose2d(0, 0, new Rotation2d());//Rotation2d() not currently working (pre new keyword)
        Pose2d interiorWaypoints = new Pose2d(1, 0, new Rotation2d());
        Pose2d end = new Pose2d(2, 0, new Rotation2d());

        TrajectoryConfig config = new TrajectoryConfig(0.5,0.5);
        config.setReversed(true);

        // what we should do from trajectorygenerator class: public static Trajectory generateTrajectory​(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, TrajectoryConfig config);
    }
}
