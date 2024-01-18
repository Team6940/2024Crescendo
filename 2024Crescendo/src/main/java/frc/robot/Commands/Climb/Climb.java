package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO 挂好了给手柄反馈

public class Climb extends Command{

    public Climb(){
        addRequirements(RobotContainer.m_Arm);
    }

    @Override
    public void initialize()
    {
        RobotContainer.m_Arm.SetArmPosition(CommandConstants.kClimbOpenDegree);
    }
    @Override
    public void execute()
    {

    }
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.m_Arm.SetArmPosition(CommandConstants.kClimbDegree);
    }
    @Override
    public boolean isFinished() 
    {
        if(RobotContainer.m_driverController.getAButtonReleased() )return true;//TODO 哪个按键？
        return false;
    }
}
