package frc.robot.Commands.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Commands.NoteIntake.NoteIntake;
import frc.robot.Commands.Shoot.NewShoot;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Subsystems.ImprovedXboxController.Button;

public class Close4note extends SequentialCommandGroup{
    public Close4note(){
        addRequirements(RobotContainer.m_swerve);
        
        addCommands(
            new InstantCommand(()->
            {
                if(DriverStation.getAlliance().get()==Alliance.Red)
                RobotContainer.m_swerve.ResetOdometry(PathPlannerPath.fromPathFile("close4notes-1").flipPath().getPreviewStartingHolonomicPose());
                else 
                {
                    RobotContainer.m_swerve.ResetOdometry(PathPlannerPath.fromPathFile("close4notes-1").getPreviewStartingHolonomicPose());
                }
        }),
            new NewShoot(ShootCommandConstants.SpeakerSet[0], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.4),
            
            RobotContainer.m_swerve.followPathCommand("close4notes-1").raceWith(new NoteIntake(Button.kAutoButton.value)),
            new NewShoot(ShootCommandConstants.SpeakerSet[0], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.4),
        RobotContainer.m_swerve.followPathCommand("close4notes-2").raceWith(new NoteIntake(Button.kAutoButton.value)),
            new NewShoot(ShootCommandConstants.SpeakerSet[0], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.4),
        RobotContainer.m_swerve.followPathCommand("close4notes-3").raceWith(new NoteIntake(Button.kAutoButton.value)),
            new NewShoot(ShootCommandConstants.SpeakerSet[0], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.4)
        );
    }

}
