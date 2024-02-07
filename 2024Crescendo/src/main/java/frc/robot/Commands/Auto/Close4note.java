package frc.robot.Commands.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CommandConstants;
import frc.robot.RobotContainer;

public class Close4note extends SequentialCommandGroup{
    public Close4note(){
        addRequirements(RobotContainer.m_swerve);
        addCommands(
            new InstantCommand(()->RobotContainer.m_swerve.ResetOdometry(PathPlannerPath.fromPathFile("close4notes-1").getPreviewStartingHolonomicPose())),
            RobotContainer.m_swerve.followPathCommand("close4notes-1"),
        RobotContainer.m_swerve.followPathCommand("close4notes-2"),
        RobotContainer.m_swerve.followPathCommand("close4notes-3"));
    }

}
