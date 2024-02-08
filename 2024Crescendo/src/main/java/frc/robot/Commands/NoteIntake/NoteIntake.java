package frc.robot.Commands.NoteIntake;

import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbCommandConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;

public class NoteIntake extends Command{
    @Override
    public void initialize(){
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
    }
    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){

    }
    @Override
    public boolean isFinished(){
        return false;
    }
}