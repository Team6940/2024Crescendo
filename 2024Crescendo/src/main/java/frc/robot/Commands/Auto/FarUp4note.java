package frc.robot.Commands.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class FarUp4note extends SequentialCommandGroup{
    public FarUp4note(){
        addRequirements(RobotContainer.m_swerve);
        addCommands(
            new InstantCommand(()->RobotContainer.m_swerve.ResetOdometry(PathPlannerPath.fromPathFile("close4notes-1").getPreviewStartingHolonomicPose())),
            RobotContainer.m_swerve.followPathCommand("farup4notes-1"),
        RobotContainer.m_swerve.followPathCommand("farup4notes-2"),
        RobotContainer.m_swerve.followPathCommand("farup4notes-3"),
        RobotContainer.m_swerve.followPathCommand("farup4notes-4"),
        RobotContainer.m_swerve.followPathCommand("farup4notes-5"),
        RobotContainer.m_swerve.followPathCommand("farup4notes-6")
        );
    }

}