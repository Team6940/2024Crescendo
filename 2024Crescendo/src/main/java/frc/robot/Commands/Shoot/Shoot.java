package frc.robot.Commands.Shoot;

import frc.robot.RobotContainer;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Shoot extends Command{

    public enum ShooterState{
        Aiming, Shooting;
    }
    ShooterState m_State;
    int m_TargetDistance;
    CommandConstants.ShootingMode m_Mode;
    double m_ShootRPS;
    double m_ShootAngle;
    public Shoot(double _Angle, double _RPS){
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        m_ShootAngle = _Angle;
        m_ShootRPS = _RPS;
    }
    public Shoot(CommandConstants.ShootingMode _Mode){
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Intake);
        if(_Mode==CommandConstants.ShootingMode.Auto)
        {
            addRequirements(RobotContainer.m_swerve);
            double _TargetDistance=RobotContainer.m_swerve.GetRobotToSpeakerTranslation().getDistance(new Translation2d());
            m_ShootAngle=ShooterConstants.kHoodTable.getOutput(_TargetDistance);
            m_ShootRPS=ShooterConstants.kRPMTable.getOutput(_TargetDistance);
        
        }
        else
        {
            
            m_ShootAngle = CommandConstants.kShootingSets[_Mode.ordinal()].getX();
            m_ShootRPS = CommandConstants.kShootingSets[_Mode.ordinal()].getY();
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
        RobotContainer.m_Arm.SetArmPosition(m_ShootAngle);
        RobotContainer.m_Shooter.SetRPS(m_ShootRPS);
        if(RobotContainer.m_Arm.IsAtTargetPosition()&&RobotContainer.m_Shooter.IsAtTargetRPS())
            m_State = ShooterState.Shooting;
    }
    void Shooting(){
        if(m_Mode == CommandConstants.ShootingMode.AMP)
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
        RobotContainer.m_Arm.SetArmPosition(CommandConstants.kShootingSets[CommandConstants.ShootingMode.Default.ordinal()].getX());
        RobotContainer.m_Shooter.SetRPS(CommandConstants.kShootingSets[CommandConstants.ShootingMode.Default.ordinal()].getY());
    }
    @Override
    public boolean isFinished() 
    {
        // if(!RobotContainer.m_Intake.HasNote()) return true;
        if(!RobotContainer.m_driverController.getAButton()) return true;    //TODO 哪个键？
        return false;
    }
}
