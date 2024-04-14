package frc.robot.Commands.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.RobotContainer;
import frc.robot.Commands.NoteIntake.NoteIntake;
import frc.robot.Commands.Shoot.NewShoot;
import frc.robot.Subsystems.ImprovedXboxController.Button;

public class C13 extends SequentialCommandGroup{
    public C13(){
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
            
            RobotContainer.m_swerve.followPathCommand("C13-1").raceWith(new NoteIntake(Button.kAutoButton.value)),
            new NewShoot(ShootCommandConstants.SpeakerSet[0], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.4),
        RobotContainer.m_swerve.followPathCommand("C13-2").raceWith(new NoteIntake(Button.kAutoButton.value)),
            new NewShoot(ShootCommandConstants.SpeakerSet[0], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.4),
        RobotContainer.m_swerve.followPathCommand("C13-3").raceWith(new NoteIntake(Button.kAutoButton.value)),
            new NewShoot(ShootCommandConstants.SpeakerSet[0], Button.kAutoButton.value, Button.kAutoButton.value).withTimeout(1.4)
        );
    }
}
