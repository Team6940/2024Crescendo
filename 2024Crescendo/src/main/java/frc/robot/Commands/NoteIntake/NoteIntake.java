package frc.robot.Commands.NoteIntake;

import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbCommandConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.ImprovedXboxController.Button;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;

public class NoteIntake extends Command{
    int m_ButtonID = Button.kLeftBumper.value;
    public NoteIntake(int _ButtonID){
        m_ButtonID=_ButtonID;
    }
    @Override
    public void initialize(){
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
    }
    @Override
    public void execute(){
        RobotContainer.m_Arm.SetArmPosition(IntakeConstants.ArmAngle);
        RobotContainer.m_Intake.NoteIn();
    }
    @Override
    public void end(boolean interrupted){
        RobotContainer.m_Arm.SetArmPosition(ShootCommandConstants.DefaultSet.ArmAngle);
        RobotContainer.m_Intake.SetIntakeOutput(0);
    }
    @Override
    public boolean isFinished(){
        if(!RobotContainer.m_driverController.getButton(m_ButtonID)) return true;
        return false;
    }
}