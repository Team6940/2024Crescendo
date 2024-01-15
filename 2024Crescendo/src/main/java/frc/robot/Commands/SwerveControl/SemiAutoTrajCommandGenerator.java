package frc.robot.Commands.SwerveControl;
// package frc.robot.commands.SwerveControl;


// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.util.List;

// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.PathPlannerTrajectory;
// import com.pathplanner.lib.path.PathPoint;

// import java.util.ArrayList;
// import frc.robot.Constants;
// import frc.robot.Auto.FollowPathCommandGenerator;
// import frc.robot.Constants.SemiAutoConstants;

// public class SemiAutoTrajCommandGenerator{
    
//     /**
//      * Drive From StartPose to the EndPose
//      * @param _StartPose
//      * @param _EndPose
//      */
//     public static Command SemiAutoPPSControl(Pose2d _StartPose, Pose2d _EndPose)
//     {
//     //     List<PathPoint> m_PathPoints= new ArrayList<>();
//     //     PathPlannerTrajectory m_Trajectory;
//     //     m_PathPoints.add(new PathPoint(_StartPose.getTranslation(), new Rotation2d(_EndPose.getX()-_StartPose.getX(), _EndPose.getY()-_StartPose.getX())));
//     //     m_PathPoints.add(new PathPoint(_EndPose.getTranslation(), new Rotation2d(_EndPose.getX()-_StartPose.getX(), _EndPose.getY()-_StartPose.getX())));
        
//     //     m_Trajectory=PathPlanner.generatePath(new PathConstraints(SemiAutoConstants.kSemiAutoVelocityConstrants.maxVelocity, SemiAutoConstants.kSemiAutoVelocityConstrants.maxAcceleration),m_PathPoints); 
//     //     return FollowPathCommandGenerator.followTrajectoryCommand(m_Trajectory, false);
//     // }
//     // /**
//     //  * Bypass all the Waypoints
//     //  * @param _WayPoints
//     //  */
//     // public static Command SemiAutoPPSControl(Pose2d[] _WayPoints)
//     // {
//     //     List<PathPoint> m_PathPoints=new ArrayList<>();
//     // PathPlannerTrajectory m_Trajectory;
//     //     for(int i=0;i<_WayPoints.length-1;++i)
//     //     {
//     //         m_PathPoints.add(new PathPoint(_WayPoints[i].getTranslation(),new Rotation2d(_WayPoints[i+1].getX()-_WayPoints[i].getX(), _WayPoints[i+1].getY()-_WayPoints[i].getY())));
//     //     }
//     //     m_PathPoints.add(new PathPoint(_WayPoints[_WayPoints.length-1].getTranslation(),new Rotation2d(_WayPoints[_WayPoints.length-1].getX()-_WayPoints[_WayPoints.length-2].getX(), _WayPoints[_WayPoints.length-1].getY()-_WayPoints[_WayPoints.length-2].getY())));
//     //     m_Trajectory=PathPlanner.generatePath(new PathConstraints(SemiAutoConstants.kSemiAutoVelocityConstrants.maxVelocity, SemiAutoConstants.kSemiAutoVelocityConstrants.maxAcceleration),m_PathPoints); 
//     //     return FollowPathCommandGenerator.followTrajectoryCommand(m_Trajectory, false);
    
//     // }
    

// }
