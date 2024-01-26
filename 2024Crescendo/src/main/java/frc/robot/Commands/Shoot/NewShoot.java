package frc.robot.Commands.Shoot;

import frc.robot.RobotContainer;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.CommandConstants.ShootingMode;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.StadiaController.Button;

public class NewShoot extends Command{
    boolean m_Auto;
    double m_TargetAngle;
    double m_TargetRPS;
    Pose2d m_TargetPose2d;
    int m_AimButtonID;
    int m_ShootButtonID;
    public enum ShooterState{
        Aiming, Shooting;
    }
    ShooterState m_State;
    public NewShoot(double _TargetAngle, double _TargetRPS, int _AimButtonID, int _ShootButtonID, Pose2d _TargetPose){
        m_Auto=true;
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_swerve);
        m_TargetAngle = _TargetAngle;
        m_TargetRPS = _TargetRPS;
        m_AimButtonID = _AimButtonID;
        m_ShootButtonID = _ShootButtonID;
        m_TargetPose2d = _TargetPose;
    }
    public NewShoot(double _TargetAngle, double _TargetRPS, Pose2d _TargetPose, int _AimButtonID, int _ShootButtonID){
        m_Auto=false;
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_Shooter);
        m_TargetAngle = _TargetAngle;
        m_TargetRPS = _TargetRPS;
        m_AimButtonID = _AimButtonID;
        m_ShootButtonID = _ShootButtonID;
        m_TargetPose2d = new Pose2d(null, null, null);
    }
    @Override
    public void initialize()
    {
        m_State=ShooterState.Aiming;
    }
    @Override
    public void execute()
    {
        if(m_State==ShooterState.Aiming){
            Aim();
        }
        if(m_State==ShooterState.Shooting){
            Shoot();
        }
    }
    void Aim(){
        if(m_Auto){
            //TODO 到达指定Pose2D;
        }
        RobotContainer.m_Arm.SetArmPosition(m_TargetAngle);
        RobotContainer.m_Shooter.SetRPS(m_TargetRPS);
        
        if( RobotContainer.m_Arm.IsAtTargetPosition()
            &&RobotContainer.m_Shooter.IsAtTargetRPS()
            &&(m_Auto?RobotContainer.m_swerve.IsAtPosition(m_TargetPose2d):true)
        ){
            m_State = ShooterState.Shooting;
        }
    }
    void Shoot(){
        if(RobotContainer.m_driverController.getRawButton(m_ShootButtonID)){
            RobotContainer.m_Intake.NoteOut();
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        //TODO 这两行实在是太丑了；
        RobotContainer.m_Arm.SetArmPosition(CommandConstants.kShootingSets[CommandConstants.ShootingMode.Default.ordinal()].getX());
        RobotContainer.m_Shooter.SetRPS(CommandConstants.kShootingSets[CommandConstants.ShootingMode.Default.ordinal()].getY());
    }
    @Override
    public boolean isFinished() 
    {
        if(!RobotContainer.m_driverController.getRawButton(m_AimButtonID)) return true;
        return false;
    }
}
