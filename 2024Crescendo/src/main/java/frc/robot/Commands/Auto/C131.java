package frc.robot.Commands.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.NoteIntake.NoteIntake;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.Commands.Shoot.NewShoot;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Subsystems.ImprovedXboxController.Button;

public class C131 extends SequentialCommandGroup{
    public C131(){
        addRequirements(RobotContainer.m_swerve);
        
        addCommands(
            new InstantCommand(()->
            {
                if(DriverStation.getAlliance().get()==Alliance.Red)
                {
                 Pose2d _Pose=PathPlannerPath.fromChoreoTrajectory("C131-1").flipPath().getPreviewStartingHolonomicPose();
                
                 RobotContainer.m_swerve.ResetOdometry(new Pose2d(_Pose.getX(), _Pose.getY(), _Pose.getRotation().rotateBy(new Rotation2d(Math.PI))));
                }
                else 
                {
                    RobotContainer.m_swerve.ResetOdometry(PathPlannerPath.fromChoreoTrajectory("C131-1").getPreviewStartingHolonomicPose());
                }
        }),
        new NewShoot(ShootCommandConstants.SpeakerSet[1], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.2),
            
            RobotContainer.m_swerve.followPathCommand("C131-1",true).raceWith(new NoteIntake(Button.kAutoButton.value)),
            new AutoShoot().withTimeout(1.2),
        RobotContainer.m_swerve.followPathCommand("C131-2",true).raceWith(new NoteIntake(Button.kAutoButton.value)),
            new AutoShoot().withTimeout(1.2),
        RobotContainer.m_swerve.followPathCommand("C131-3",true).raceWith(new NoteIntake(Button.kAutoButton.value)),
            new AutoShoot().withTimeout(1.2),
            RobotContainer.m_swerve.followPathCommand("C131-4",true).raceWith(new NoteIntake(Button.kAutoButton.value)),
            new AutoShoot().withTimeout(1.2)
        );
    }

}
