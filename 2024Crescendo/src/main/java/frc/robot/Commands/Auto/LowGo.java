package frc.robot.Commands.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.NoteIntake.NoteIntake;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.Commands.Shoot.NewShoot;
import frc.robot.Subsystems.ImprovedXboxController.Button;

public class LowGo extends SequentialCommandGroup{
    public LowGo(){
        addRequirements(RobotContainer.m_swerve);
        
        addCommands(
            new InstantCommand(()->
            {
                RobotContainer.m_Arm.SetArmPosition(Constants.ShootCommandConstants.DefaultSet.ArmAngle);
                if(DriverStation.getAlliance().get()==Alliance.Red)
                {
                 Pose2d _Pose=PathPlannerPath.fromPathFile("LowGo").flipPath().getPreviewStartingHolonomicPose();
                
                 RobotContainer.m_swerve.ResetOdometry(_Pose);
                }else 
                {
                    RobotContainer.m_swerve.ResetOdometry(PathPlannerPath.fromPathFile("LowGo").getStartingDifferentialPose());
                }
        }),
            
        new NewShoot(ShootCommandConstants.PassSet.ArmAngle,ShootCommandConstants.PassSet.ShooterRPS ,Button.kAutoButton.value , Button.kAutoButton.value).withTimeout(3),    
            new WaitCommand(7.),
            RobotContainer.m_swerve.followPathCommand("LowGo",false).raceWith(new NoteIntake(Button.kAutoButton.value))
            
                
            );
    }
}