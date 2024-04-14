package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbCommandConstants;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;

public class ArmCoast extends Command{
      public ArmCoast(){
        addRequirements(RobotContainer.m_Arm);
    }

    @Override
    public void initialize()
    {
        
    }
    @Override
    public void execute()
    {
        double Velocity=Math.sqrt(RobotContainer.m_swerve.getChassisSpeeds().vxMetersPerSecond*RobotContainer.m_swerve.getChassisSpeeds().vxMetersPerSecond
        +RobotContainer.m_swerve.getChassisSpeeds().vyMetersPerSecond*RobotContainer.m_swerve.getChassisSpeeds().vyMetersPerSecond);
        double VelocityPercent=Velocity/SwerveConstants.kMaxSpeed;
        if(VelocityPercent>0.3)
        {
            RobotContainer.m_Arm.SetArmPosition((1-VelocityPercent)*ShootCommandConstants.DefaultSet.ArmAngle);
        }
        else{
            RobotContainer.m_Arm.SetArmPosition(ShootCommandConstants.DefaultSet.ArmAngle);
            
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        // RobotContainer.m_Arm.SetArmPosition(ShootCommandConstants.DefaultSet.ArmAngle);
    }
    @Override
    public boolean isFinished() 
    {
        // if(RobotContainer.m_driverController.getAButtonReleased() )return true;//TODO 哪个按键？
        return false;
    }
    
}
