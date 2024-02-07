package frc.robot.Commands.Shoot;

import frc.robot.RobotContainer;
import frc.robot.Library.NumberLimiter;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.SemiAutoConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.CommandConstants.ShootingMode;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.StadiaController.Button;

public class NewShoot extends Command{
    boolean m_Auto;
    double m_ArmTargetAngle;
    double m_ShooterTargetRPS;
    int m_AimButtonID;
    int m_ShootButtonID;
    class Movement{
        Pose2d m_TargetPose2d;
        PIDController m_VelocityPIDController;
        PIDController m_OmegaPIDController;
        Rotation2d m_VelocityDirection;
        double m_BeginTime;
        SlewRateLimiter m_SemiAutoOmegSlewRateLimiter=new SlewRateLimiter(DriveConstants.kRotationSlew);
        SlewRateLimiter m_SemiAutoVelocitySlewRateLimiter=new SlewRateLimiter(DriveConstants.kTranslationSlew);
    }
    Movement m_Movement;
    public enum ShooterState{
        Aiming, Shooting;
    }
    ShooterState m_State;
    public NewShoot(double _ArmTargetAngle, double _ShooterTargetRPS, int _AimButtonID, int _ShootButtonID, Pose2d _TargetPose){
        m_Auto=true;
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_swerve);
        m_ArmTargetAngle = _ArmTargetAngle;
        m_ShooterTargetRPS = _ShooterTargetRPS;
        m_AimButtonID = _AimButtonID;
        m_ShootButtonID = _ShootButtonID;
        m_Movement.m_TargetPose2d = _TargetPose;
    }
    public NewShoot(double _ArmTargetAngle, double _ShooterTargetRPS, Pose2d _TargetPose, int _AimButtonID, int _ShootButtonID){
        m_Auto=false;
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_Shooter);
        m_ArmTargetAngle = _ArmTargetAngle;
        m_ShooterTargetRPS = _ShooterTargetRPS;
        m_AimButtonID = _AimButtonID;
        m_ShootButtonID = _ShootButtonID;
        m_Movement.m_TargetPose2d = new Pose2d(null, null, null);
    }
    @Override
    public void initialize()
    {
        m_State=ShooterState.Aiming;
        if(m_Auto){
            //m_Movement.m_BeginTime = Timer.getFPGATimestamp();
            Pose2d _CurrentPose2d = RobotContainer.m_swerve.getPose();
            m_Movement.m_VelocityDirection = new Rotation2d(
                m_Movement.m_TargetPose2d.getX()-_CurrentPose2d.getX(),
                m_Movement.m_TargetPose2d.getY()-_CurrentPose2d.getY()
            );
            m_Movement.m_VelocityPIDController = new PIDController(SemiAutoConstants.kSemiAutoVelocityP, SemiAutoConstants.kSemiAutoVelocityI, SemiAutoConstants.kSemiAutoVelocityD);   //TODO 没准常数得换
            m_Movement.m_OmegaPIDController = new PIDController(SemiAutoConstants.kSemiAutoOmegaP,SemiAutoConstants.kSemiAutoOmegaI, SemiAutoConstants.kSemiAutoOmegaD);    //TODO
            m_Movement.m_OmegaPIDController.setSetpoint(0);
            m_Movement.m_OmegaPIDController.setTolerance(Math.PI/20);
            m_Movement.m_VelocityPIDController.setSetpoint(0);
            m_Movement.m_VelocityPIDController.setTolerance(0.05);
            m_Movement.m_OmegaPIDController.enableContinuousInput(-Math.PI, Math.PI);
        }
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
            Pose2d _NowPose2d=RobotContainer.m_swerve.getPose();
            m_Movement.m_VelocityDirection=new Rotation2d(m_Movement.m_TargetPose2d.getX()-_NowPose2d.getX(),m_Movement.m_TargetPose2d.getY()-_NowPose2d.getY());
            //double _DeltaTime=Timer.getFPGATimestamp()-m_Movement.m_BeginTime;
            double _Distance=Math.sqrt((_NowPose2d.getX()-m_Movement.m_TargetPose2d.getX())*(_NowPose2d.getX()-m_Movement.m_TargetPose2d.getX())+
            (_NowPose2d.getY()-m_Movement.m_TargetPose2d.getY())*(_NowPose2d.getY()-m_Movement.m_TargetPose2d.getY()));//计算两个点之间的距离
            double _Velocity=-m_Movement.m_VelocityPIDController.calculate(_Distance);

            double _Omega=m_Movement.m_SemiAutoOmegSlewRateLimiter.calculate(m_Movement.m_OmegaPIDController.calculate(-m_Movement.m_TargetPose2d.minus(_NowPose2d).getRotation().getRadians()));
            _Omega=NumberLimiter.Limit(-SemiAutoConstants.SemiAutoOmegaMax,+SemiAutoConstants.SemiAutoOmegaMax,_Omega);
            _Velocity=NumberLimiter.Limit(-SemiAutoConstants.kSemiAutoVelocityConstrants.maxVelocity,+SemiAutoConstants.kSemiAutoVelocityConstrants.maxVelocity,_Velocity);
            _Velocity=m_Movement.m_SemiAutoVelocitySlewRateLimiter.calculate(_Velocity); 
            RobotContainer.m_swerve.Drive(new Translation2d(_Velocity,m_Movement.m_VelocityDirection), _Omega, true, false);
        }
        RobotContainer.m_Arm.SetArmPosition(m_ArmTargetAngle);
        RobotContainer.m_Shooter.SetRPS(m_ShooterTargetRPS);
        
        if(RobotContainer.m_Arm.IsAtTargetPosition()
            &&RobotContainer.m_Shooter.IsAtTargetRPS()
            &&(m_Auto?RobotContainer.m_swerve.IsAtPosition(m_Movement.m_TargetPose2d):true)
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
        RobotContainer.m_Arm.SetArmPosition(CommandConstants.DefaultSet.ArmAngle);
        RobotContainer.m_Shooter.SetRPS(CommandConstants.DefaultSet.ShooterRPS);
    }
    @Override
    public boolean isFinished() 
    {
        if(!RobotContainer.m_driverController.getRawButton(m_AimButtonID)) return true;
        return false;
    }
}
