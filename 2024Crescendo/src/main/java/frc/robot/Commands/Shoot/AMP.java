package frc.robot.Commands.Shoot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.AMPCommandConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Library.LimelightHelper.LimelightHelpers;

public class AMP extends Command{

    public enum AMPState{
        Adjust, Shoot
    };
    AMPState m_State;
    public AMP()
    {
        addRequirements(RobotContainer.m_Arm);
        addRequirements(RobotContainer.m_Shooter);
        addRequirements(RobotContainer.m_Intake);
        addRequirements(RobotContainer.m_swerve);
    }
    @Override
    public void initialize()
    {
        m_State=AMPState.Adjust;
        RobotContainer.m_Arm.SetArmPosition(AMPCommandConstants.AMPArmDegree);
        RobotContainer.m_Shooter.SetRPS(AMPCommandConstants.AMPArmDegree);
    }
    @Override
    public void execute()
    {
        if(m_State==AMPState.Adjust)
        {
            Adjust();
        }
        else
        {
            Shoot();
        }
    }
    double GetAMPAngle(){
        if(DriverStation.getAlliance().get()==DriverStation.Alliance.Blue)
        {
            return -90.;
        }
        else
            return 90.;
    }
    void Adjust()
    {
        double _Omega=0.;
        double _XDrive=-RobotContainer.m_driverController.getLeftY()*SwerveConstants.kMaxSpeed;
        double _YDrive=0.;
        if(LimelightHelpers.getTV("limelight"))
        {
            if(LimelightHelpers.getTX("limelight")>AMPCommandConstants.AMPdxTolerance)
            _YDrive=AMPCommandConstants.AMDFixingX;
            if(LimelightHelpers.getTX("limelight")<-AMPCommandConstants.AMPdxTolerance)
            _YDrive=-AMPCommandConstants.AMDFixingX;

        }
        if(GetAMPAngle()-RobotContainer.m_swerve.GetHeading_Deg()>AMPCommandConstants.AMPAngleTolerance)
        {
            _Omega=AMPCommandConstants.AMPFixingOmega;
        }
        if(RobotContainer.m_swerve.GetHeading_Deg()-GetAMPAngle()<-AMPCommandConstants.AMPAngleTolerance)
        {
            _Omega=-AMPCommandConstants.AMPFixingOmega;
        }
        RobotContainer.m_swerve.Drive(new Translation2d(_XDrive, _YDrive), _Omega, false, false);
        if(_Omega==0.&&_XDrive==0.&&RobotContainer.m_Shooter.IsAtTargetRPS()&&RobotContainer.m_Arm.IsAtTargetPosition()&&RobotContainer.m_driverController.getButton(AMPCommandConstants.AMPSHootingButton))
        {
            m_State=AMPState.Shoot;
        }
    }
    void Shoot()
    {
        
        RobotContainer.m_Intake.NoteOut();
    }
    @Override
    public void end(boolean interrupted)
    {
        RobotContainer.m_Intake.SetIntakeOutput(0.);
        RobotContainer.m_Shooter.SetPct(0.);
        RobotContainer.m_Arm.SetArmPosition(ShootCommandConstants.DefaultSet.ArmAngle);

    }
    @Override
    public boolean isFinished() 
    {
        if(!RobotContainer.m_driverController.getButton(AMPCommandConstants.AMPAimingButton))
        {
            return true;
        }
        return false;
    }
}
