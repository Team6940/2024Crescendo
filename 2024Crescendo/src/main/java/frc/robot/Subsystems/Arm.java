package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShootConstants;
import java.lang.Math;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
public class Arm extends SubsystemBase{
    public static Arm m_Instance;
    private static TalonFX m_ArmMotor;
    private double m_TargetPosition;    //parametre is degrees;
    private TalonFXConfiguration m_TalonFXConfiguration=new TalonFXConfiguration();
    private MotionMagicDutyCycle m_MotionMagicDutyCycle=new MotionMagicDutyCycle(0, false, 0.3, 0, true, false, false);
    
    Arm()
    {
        m_ArmMotor= new TalonFX(ArmConstants.ArmMotorPort);
        m_TalonFXConfiguration.MotorOutput.NeutralMode=NeutralModeValue.Brake;
        m_TalonFXConfiguration.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
        m_TalonFXConfiguration.MotorOutput.PeakForwardDutyCycle=1;
        m_TalonFXConfiguration.MotorOutput.PeakReverseDutyCycle=-1;
    
        m_TalonFXConfiguration.Slot0.kP=ArmConstants.m_ArmP;
        m_TalonFXConfiguration.Slot0.kI=ArmConstants.m_ArmI;
        m_TalonFXConfiguration.Slot0.kD=ArmConstants.m_ArmD;
        m_TalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity=ArmConstants.m_ArmVelocity;
        m_TalonFXConfiguration.MotionMagic.MotionMagicAcceleration=ArmConstants.m_ArmAcceleration;
        m_ArmMotor.getConfigurator().apply(m_TalonFXConfiguration);
    }
    /**
     * Set the arm to the target Position
     * @param _Degree degree
     */
    public void SetArmPosition(double _Degree)
    {
        m_TargetPosition=_Degree;
        m_ArmMotor.setControl(m_MotionMagicDutyCycle.withPosition(_Degree/360.));
    }
    /**
     * Get the Arm's Postion In degree
     * @return the Arm's Position Compare to the Initial Degree
     */
    public double GetArmPosition()
    {
         return m_ArmMotor.getPosition().getValue()*360.;
    }
    /*
     * Get the Arm's Position in degrees
     * @return the Arm's Target Position Comapred to the Initial Position
     */
    public double GetTargetPosition(){
        return m_TargetPosition;
    }
    public boolean IsAtTargetPosition(){
        return Math.abs(GetArmPosition()-GetTargetPosition())<ArmConstants.ArmDegreeTolerance;
    }
    public static Arm GetInstance()
    {
        return m_Instance==null?m_Instance=new Arm():m_Instance;
    }
}
