package frc.robot.Commands.Shoot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AMPCommandConstants;
import frc.robot.Constants.AutoShootCommandConstants;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Commands.Shoot.NewShoot.ShooterState;
import frc.robot.Library.LimelightHelper.LimelightHelpers;

public class AutoShoot extends Command{

    
    double m_ArmTargetAngle=0.;
    double m_SHooterTargetRPS=0.;
;
    enum AutoShootState {
        Aim,Accelerate,Shoot;
    }
    AutoShootState m_State;
    public AutoShoot()
    {
        
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_swerve);
    }
    @Override
    public void initialize()
    {
        m_State=AutoShootState.Aim;
    }
    void Aim()
    {
        
        double _Omega=0.; 
        RobotContainer.m_Shooter.SetRPS(AutoShootCommandConstants.AutoShootRPS);
        if(LimelightHelpers.getTV("limelight"))
        {
            if(LimelightHelpers.getTX("limelight")>AutoShootCommandConstants.NewShootAngleTolerance)
            _Omega=-AutoShootCommandConstants.NewShootFixingOmega;
            else if(LimelightHelpers.getTX("limelight")<-AutoShootCommandConstants.NewShootAngleTolerance)
            _Omega=AutoShootCommandConstants.NewShootFixingOmega;
            else
            {
                RobotContainer.m_swerve.Drive(new Translation2d(), _Omega, false, false);
        
                m_ArmTargetAngle=AutoShootCommandConstants.m_CloseShootLinearInterPolationTable.getOutput(LimelightHelpers.getTY("limelight"));
                m_SHooterTargetRPS=AutoShootCommandConstants.m_RPSInterPolationTable.getOutput(LimelightHelpers.getTY("limelight"));
                
                SmartDashboard.putNumber("ArmGgle", m_ArmTargetAngle);
                SmartDashboard.putNumber("LimelightTY", LimelightHelpers.getTY("limelight"));
                m_State=AutoShootState.Accelerate;
            }
        }
        RobotContainer.m_swerve.Drive(new Translation2d(), _Omega, false, false);
        
    }
    void Accelerate()
    {
        RobotContainer.m_Arm.SetArmPosition(m_ArmTargetAngle);
        RobotContainer.m_Shooter.SetRPS(m_SHooterTargetRPS);
        if(RobotContainer.m_Arm.IsAtTargetPosition()&&RobotContainer.m_Shooter.IsAtTargetRPS())
        {
            m_State=AutoShootState.Shoot;
        }
        
    }
    void Shoot()
    {
        RobotContainer.m_Intake.NoteOut();
    }
    @Override
    public void execute()
    {
        if(m_State==AutoShootState.Aim)
        {
            Aim();
        }
        if(m_State==AutoShootState.Accelerate)
        {
            Accelerate();
        }
        if(m_State==AutoShootState.Shoot)
        {
            Shoot();
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.m_Arm.SetArmPosition(ShootCommandConstants.DefaultSet.ArmAngle);
        RobotContainer.m_Shooter.SetPct(0.);
        RobotContainer.m_Intake.SetIntakeOutput(0.);
    }
    @Override
    public boolean isFinished() 
    {
        if(!RobotContainer.m_driverController.getButton(AutoShootCommandConstants.AutoShootButton)&&!DriverStation.isAutonomous())
        {
            return true;
        }
        return false;
    }
}
