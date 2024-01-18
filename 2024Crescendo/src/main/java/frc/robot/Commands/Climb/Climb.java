package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Shooter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO 挂好了给手柄反馈

public class Climb extends Command{
    public Climb(){
        addRequirements(RobotContainer.m_Arm);
    }
    public enum ClimbState{
        START,UP,DOWN
    }
    public ClimbState m_State;

    @Override
    public void initialize()
    {
        m_State=ClimbState.START;
    }
    @Override
    public void execute()
    {
        if(m_State==ClimbState.START){
            RobotContainer.m_Arm.SetArmPosition(CommandConstants.kClimbOpenDegree);
            if(RobotContainer.m_Arm.IsAtTargetPosition()){
                m_State=ClimbState.UP;
            }
        }

        if(m_State==ClimbState.UP){
            RobotContainer.m_Arm.SetArmPosition(CommandConstants.kClimbDegree);
            if(RobotContainer.m_Arm.IsAtTargetPosition() && !RobotContainer.m_driverController.getAButton()){//TODO 哪个键？
                m_State=ClimbState.DOWN;
            }
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.m_Arm.SetArmPosition(CommandConstants.kClimbOpenDegree);
    }
    @Override
    public boolean isFinished() 
    {
        if(RobotContainer.m_driverController.getAButton() && m_State==ClimbState.DOWN)return true;
        return false;
    }
}
