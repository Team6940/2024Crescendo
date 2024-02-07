package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
    private static TalonFX m_ArmMotorLeft;
    private static TalonFX m_ArmMotorRight;
    private double m_TargetPosition;    //parametre is degrees;
    private TalonFXConfiguration m_LeftTalonFXConfiguration=new TalonFXConfiguration();
    private TalonFXConfiguration m_RghtTalonFXConfiguration=new TalonFXConfiguration();
    private MotionMagicDutyCycle m_MotionMagicDutyCycle=new MotionMagicDutyCycle(0, false, 0., 0, true, false, false);
    
    Arm()
    {
        m_ArmMotorLeft= new TalonFX(ArmConstants.ArmMotorLeftPort);
        m_ArmMotorRight=new TalonFX(ArmConstants.ArmMotorRightPort);
        m_LeftTalonFXConfiguration.MotorOutput.NeutralMode=NeutralModeValue.Brake;
        m_LeftTalonFXConfiguration.MotorOutput.Inverted=InvertedValue.Clockwise_Positive;
        m_LeftTalonFXConfiguration.MotorOutput.PeakForwardDutyCycle=1;
        m_LeftTalonFXConfiguration.MotorOutput.PeakReverseDutyCycle=-1;
        m_LeftTalonFXConfiguration.Feedback.SensorToMechanismRatio=250;
    
        m_LeftTalonFXConfiguration.Slot0.kP=ArmConstants.m_ArmP;
        m_LeftTalonFXConfiguration.Slot0.kI=ArmConstants.m_ArmI;
        m_LeftTalonFXConfiguration.Slot0.kD=ArmConstants.m_ArmD;
        m_LeftTalonFXConfiguration.Slot0.kS=ArmConstants.m_ArmkS;
        m_LeftTalonFXConfiguration.Slot0.GravityType=GravityTypeValue.Arm_Cosine;
        m_LeftTalonFXConfiguration.MotionMagic.MotionMagicCruiseVelocity=ArmConstants.m_ArmVelocity;
        m_LeftTalonFXConfiguration.MotionMagic.MotionMagicAcceleration=ArmConstants.m_ArmAcceleration;
        
        
        m_RghtTalonFXConfiguration=m_LeftTalonFXConfiguration;
        m_ArmMotorLeft.getConfigurator().apply(m_LeftTalonFXConfiguration);
        
        m_RghtTalonFXConfiguration.MotorOutput.Inverted=InvertedValue.CounterClockwise_Positive;
        m_ArmMotorRight.getConfigurator().apply(m_RghtTalonFXConfiguration);
    }
    /**
     * Set the arm to the target Position
     * @param _Degree degree
     */
    public void SetArmPosition(double _Degree)
    {
        m_TargetPosition=_Degree;
        m_ArmMotorLeft.setControl(m_MotionMagicDutyCycle.withPosition(m_TargetPosition/360.));
        m_ArmMotorRight.setControl(m_MotionMagicDutyCycle.withPosition(m_TargetPosition/360.));
    }
    public void SetArmOutput(double _PctOut)
    {
        m_ArmMotorLeft.set(_PctOut);
        m_ArmMotorRight.set(_PctOut);
    }
    /**
     * Get the Arm's Postion In degree
     * @return the Arm's Position Compare to the Initial Degree
     */
    public double GetArmPosition()
    {
         return m_ArmMotorLeft.getPosition().getValue()*360.;
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
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("ArmPosition", GetArmPosition());
    }
}  
