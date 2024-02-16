package frc.robot.Commands.Shoot;

import frc.robot.RobotContainer;
import frc.robot.Library.NumberLimiter;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team1706.MathUtils;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SemiAutoConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Shooter;
import frc.robot.Library.LimelightHelper.LimelightHelpers;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.StadiaController.Button;

public class NewShoot extends Command{
    boolean m_LineUp;
    double m_ArmTargetAngle;
    double m_ShooterTargetRPS;
    int m_AimButtonID;
    int m_ShootButtonID;
    int m_TargetAprilTagID = 0;
    double m_TargetDirection = 0.;
    public enum ShooterState{
        Aiming, Shooting;
    }
    ShooterState m_State;
    public NewShoot(double _ArmTargetAngle, double _ShooterTargetRPS, int _AimButtonID, int _ShootButtonID){
        m_LineUp=false;
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_Shooter);
        m_ArmTargetAngle = _ArmTargetAngle;
        m_ShooterTargetRPS = _ShooterTargetRPS;
        m_AimButtonID = _AimButtonID;
        m_ShootButtonID = _ShootButtonID;
    }
    public NewShoot(ShootCommandConstants.ShootingSet _set, int _AimButtonID, int _ShootButtonID){
        this(_set.ArmAngle, _set.ShooterRPS, _AimButtonID, _ShootButtonID);
    }
    public NewShoot(double _ArmTargetAngle, double _ShooterTargetRPS, int _AimButtonID, int _ShootButtonID, int _AprilTagID, double _TargetDirection){
        m_LineUp=true;
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_Shooter);
        m_ArmTargetAngle = _ArmTargetAngle;
        m_ShooterTargetRPS = _ShooterTargetRPS;
        m_AimButtonID = _AimButtonID;
        m_ShootButtonID = _ShootButtonID;
        m_TargetAprilTagID = _AprilTagID;
        m_TargetDirection = _TargetDirection;
    }
    @Override
    public void initialize()
    {
        m_State=ShooterState.Aiming;
        if(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0) == m_TargetAprilTagID){
            m_LineUp = false;
        }
        if(m_LineUp){
            addRequirements(RobotContainer.m_swerve);
        }
    }
    @Override
    public void execute()
    {
        if(m_State==ShooterState.Aiming){
            Aim();
        }
        else if(m_State==ShooterState.Shooting){
            Shoot();
        }
        if(m_LineUp){
            RobotContainer.m_swerve.Drive(getTranslation2d(), getOmega(), false, false);
        }
    }
    private Translation2d getTranslation2d(){
        double translationX = -inputTransform(getX());
        double translationY = -inputTransform(RobotContainer.m_driverController.getLeftX());
        return new Translation2d(translationX, translationY);
    }
    private double inputTransform(double input) {
        return MathUtils.signedSquare(MathUtils.applyDeadband(input));
    }
    private double getOmega(){
        if(RobotContainer.m_swerve.GetGyroRotation2d().getDegrees()-m_TargetDirection >= ShootCommandConstants.kShootDirectionTolerance){
            return -ShootCommandConstants.kShootFixOmega;
        }
        else if(RobotContainer.m_swerve.GetGyroRotation2d().getDegrees()-m_TargetDirection <= -ShootCommandConstants.kShootDirectionTolerance){
            return ShootCommandConstants.kShootFixOmega;
        }
        return 0.;
    }
    private double getX(){
        if(LimelightHelpers.getTX("limelight")>=ShootCommandConstants.kShootLineUpTolerance) {
            return -ShootCommandConstants.kShootLineUpFixVelocity;
        }
        if(LimelightHelpers.getTX("limelight")>=ShootCommandConstants.kShootLineUpTolerance){
            return ShootCommandConstants.kShootLineUpFixVelocity;
        }
        return 0.;
    }
    void Aim(){
        RobotContainer.m_Arm.SetArmPosition(m_ArmTargetAngle);
        RobotContainer.m_Shooter.SetRPS(m_ShooterTargetRPS);
        SmartDashboard.putBoolean("IsAtPosition", RobotContainer.m_Arm.IsAtTargetPosition());
        SmartDashboard.putBoolean("IsAtRPS", RobotContainer.m_Shooter.IsAtTargetRPS());
        if(RobotContainer.m_Arm.IsAtTargetPosition()
            &&RobotContainer.m_Shooter.IsAtTargetRPS()
        ){ 
            m_State = ShooterState.Shooting;
        }
    }
    void Shoot(){
        SmartDashboard.putBoolean("Shooting", true);
        if(RobotContainer.m_driverController.getButton(m_ShootButtonID)){
            RobotContainer.m_Intake.NoteOut();
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.m_Arm.SetArmPosition(ShootCommandConstants.DefaultSet.ArmAngle);
        RobotContainer.m_Shooter.SetPct(ShootCommandConstants.DefaultSet.ShooterRPS);
        RobotContainer.m_Intake.SetIntakeOutput(0);
        if(m_ShootButtonID==0)
        {
            
        RobotContainer.m_Arm.SetArmPosition(IntakeConstants.ArmAngle);
        RobotContainer.m_Shooter.SetPct(0);
        RobotContainer.m_Intake.NoteIn();
        }
    }
    @Override
    public boolean isFinished() 
    {
        if(!RobotContainer.m_driverController.getButton(m_AimButtonID)) return true;
        return false;
    }
}
