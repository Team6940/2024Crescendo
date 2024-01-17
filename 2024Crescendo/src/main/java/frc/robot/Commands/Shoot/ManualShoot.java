package frc.robot.Commands.Shoot;

import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualShoot extends Command{

    public enum ShooterState{
        Aiming, Accelerating, Shooting;
    }
    ShooterState m_State;
    double m_TargetDistance;
    double m_ShootRPS;
    double m_ShootAngle;
    public ManualShoot(){
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Arm);
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
        if(m_State == ShooterState.Accelerating)
            Accelerate();
        if(m_State == ShooterState.Shooting)
            Shoot();
    }
    void Aim(){
        
    }
    void Accelerate(){

    }
    void Shoot(){

    }
    @Override
    public void end(boolean interrupted)
    {
        
    }
    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
