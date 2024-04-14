package frc.robot.Commands.Auto;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.NoteIntake.NoteIntake;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.Commands.Shoot.NewShoot;
import frc.robot.Subsystems.ImprovedXboxController.Button;

public class C102 extends SequentialCommandGroup{
    public C102(){
        addRequirements(RobotContainer.m_swerve);
        
        addCommands(
            new InstantCommand(()->
            {
                RobotContainer.m_Arm.SetArmPosition(Constants.ShootCommandConstants.DefaultSet.ArmAngle);
                if(DriverStation.getAlliance().get()==Alliance.Red)
                {
                 Pose2d _Pose=PathPlannerPath.fromChoreoTrajectory("C102-1").flipPath().getPreviewStartingHolonomicPose();
                
                 RobotContainer.m_swerve.ResetOdometry(new Pose2d(_Pose.getX(), _Pose.getY(), _Pose.getRotation().rotateBy(new Rotation2d(Math.PI))));
                }else 
                {
                    RobotContainer.m_swerve.ResetOdometry(PathPlannerPath.fromChoreoTrajectory("C102-1").getPreviewStartingHolonomicPose());
                }
        }),
            
            RobotContainer.m_swerve.followPathCommand("C102-1",true).raceWith(new NoteIntake(Button.kAutoButton.value)),
            new AutoShoot().withTimeout(1.3),        
            RobotContainer.m_swerve.followPathCommand("C102-2",true).raceWith(new NoteIntake(Button.kAutoButton.value)),
            new AutoShoot().withTimeout(1.3),
        RobotContainer.m_swerve.followPathCommand("C102-3",true).raceWith(new NoteIntake(Button.kAutoButton.value)),
            new AutoShoot()       
            );
    }
}