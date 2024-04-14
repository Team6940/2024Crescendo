package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimbCommandConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO 挂好了给手柄反馈

public class Climb extends Command{

    boolean m_IsDown=false;
    public Climb(){
        addRequirements(RobotContainer.m_Arm);
        
    }

    @Override
    public void initialize()
    {
        RobotContainer.m_Arm.SetArmPosition(ClimbCommandConstants.kClimbOpenDegree);
    }
    @Override
    public void execute()
    {
        if(RobotContainer.m_driverController.getBButtonReleased())
            
            RobotContainer.m_Arm.SetArmPosition(ClimbCommandConstants.kClimbDegree);
        

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
