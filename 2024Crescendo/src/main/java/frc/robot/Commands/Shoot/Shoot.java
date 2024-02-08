package frc.robot.Commands.Shoot;

import frc.robot.RobotContainer;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.ShootCommandConstants.ShootingMode;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;

public class Shoot extends Command{

    public enum ShooterState{
        Aiming, Shooting;
    }
    ShooterState m_State;
    int m_TargetDistance;
    ShootCommandConstants.ShootingMode m_Mode;
    double m_ShootRPS;
    double m_ShootAngle;
    public Shoot(double _Angle, double _RPS){
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        m_ShootAngle = _Angle;
        m_ShootRPS = _RPS;
    }
    public Shoot(ShootCommandConstants.ShootingMode _Mode){
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        if(_Mode==ShootCommandConstants.ShootingMode.Auto)
        {
            addRequirements(RobotContainer.m_swerve);
            double _TargetDistance=RobotContainer.m_swerve.GetRobotToTargetTranslation(DriverStation.getAlliance(),GoalConstants.FieldElement.Speaker).getDistance(new Translation2d());
            m_ShootAngle=ShootConstants.kHoodTable.getOutput(_TargetDistance);
            m_ShootRPS=ShootConstants.kRPMTable.getOutput(_TargetDistance);
        
        }
        else
        {
            m_ShootAngle = ShootCommandConstants.kShootingSets[_Mode.ordinal()].getX();
            m_ShootRPS = ShootCommandConstants.kShootingSets[_Mode.ordinal()].getY();
            m_Mode=_Mode;
        }
    }
    @Override
    public void initialize()
    {
        m_State = ShooterState.Aiming;
    }
    @Override
    public void execute()
    {
        if(m_State == ShooterState.Aiming)
            Aim();
        if(m_State == ShooterState.Shooting)
            Shooting();
    }
    void Aim(){
        if(m_Mode==ShootingMode.Auto)
        {
            RobotContainer.m_Arm.SetArmPosition(m_ShootAngle);
            RobotContainer.m_Shooter.SetRPS(m_ShootRPS);
            double _SwerveToSpeakerDegree=RobotContainer.m_swerve.GetRobotToTargetRotation(DriverStation.getAlliance(), GoalConstants.FieldElement.Speaker).minus(RobotContainer.m_swerve.getPose().getRotation()).getDegrees();
            if(_SwerveToSpeakerDegree>ShootConstants.kShootDirectionTolerance)
            {
                RobotContainer.m_swerve.Drive(new Translation2d(), -ShootConstants.kShootFixOmega, true, true);
            }
            else if (_SwerveToSpeakerDegree<-ShootConstants.kShootDirectionTolerance)
            {
                RobotContainer.m_swerve.Drive(new Translation2d(), ShootConstants.kShootFixOmega, true, true);
            
            }
            else{
                RobotContainer.m_swerve.Drive(new Translation2d(), 0, true, true);
                if(RobotContainer.m_Arm.IsAtTargetPosition()&&RobotContainer.m_Shooter.IsAtTargetRPS())
                    m_State = ShooterState.Shooting;
            
            }
        }
        else{
            RobotContainer.m_Arm.SetArmPosition(m_ShootAngle);
            RobotContainer.m_Shooter.SetRPS(m_ShootRPS);
            if(RobotContainer.m_Arm.IsAtTargetPosition()&&RobotContainer.m_Shooter.IsAtTargetRPS())
                m_State = ShooterState.Shooting;
        }
        
    }
    void Shooting(){
        if(m_Mode == ShootCommandConstants.ShootingMode.AMP)
        {
            if(RobotContainer.m_driverController.getAButton()){     //TODO 哪个键？
                RobotContainer.m_Intake.NoteOut();
            }
        }
        else{
            RobotContainer.m_Intake.NoteOut();
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.m_Arm.SetArmPosition(ShootCommandConstants.kShootingSets[ShootCommandConstants.ShootingMode.Default.ordinal()].getX());
        RobotContainer.m_Shooter.SetRPS(ShootCommandConstants.kShootingSets[ShootCommandConstants.ShootingMode.Default.ordinal()].getY());
    }
    @Override
    public boolean isFinished() 
    {
        // if(!RobotContainer.m_Intake.HasNote()) return true;
        if(!RobotContainer.m_driverController.getLeftBumper()&&m_Mode==ShootCommandConstants.ShootingMode.AMP) return true;    //TODO 哪个键？
        if(!RobotContainer.m_driverController.getRightBumper()&&m_Mode==ShootCommandConstants.ShootingMode.Auto) return true;    //TODO 哪个键？
        if(!RobotContainer.m_driverController.getAButton()&&m_Mode==ShootCommandConstants.ShootingMode.SpeakerPos1) return true;    //TODO 哪个键？
        
        return false;
    }
}
