package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public static Shooter m_Instance;
    TalonFX m_ShooterLeft;
    TalonFX m_ShooterRght;
    MotorOutputConfigs m_LeftMotorOutputConfigs=new MotorOutputConfigs();
    MotorOutputConfigs m_RghtMorotOutputConfigs=new MotorOutputConfigs();
    Slot0Configs m_Slot0Configs=new Slot0Configs();
    VelocityDutyCycle m_VelocityDutyCycle =new VelocityDutyCycle(0, 0, false, Constants.ShooterConstants.kShootF, 0, false, true, true);
    VoltageConfigs m_VoltageConfigs=new VoltageConfigs();
    
    Shooter()
    {
        m_LeftMotorOutputConfigs.NeutralMode=NeutralModeValue.Coast;
        m_LeftMotorOutputConfigs.Inverted=InvertedValue.Clockwise_Positive;
        
    }
    /**
     * Set the Rotation speed of the shooter, Positive stands for get the Note out
     * @param _RPS
     */
    void SetRPS(double _RPS)
    {
        if(!m_Enabled) return;
        m_TargetSpeed = _RPS;
        if(_RPS == 0){
            m_ShooterLeft.stopMotor();
        }
        else{
            m_ShooterLeft.setControl(m_request.withVelocity(_RPS));
        }
    }
    /**
     * Get the Rotation speed of the shooter, Positive stands for get the Note out
     */
    double GetRPS()
    {
        if(m_Enabled){
            return m_ShooterLeft.getVelocity().getValue();
        }
        return 0.;
    }
    /**
     * Get the Target Rotation speed of the shooter, Positive stands for get the Note out
     */
    double GetTargetRPS()
    {
        if(m_Enabled){
            return m_TargetSpeed;
        }
        return 0.;
    }
    /**
     * Is shooter at TargetRPM
     * @return true means the shooter is at Target RPM
     */
    boolean IsAtTargetRPM()
    {

        return true;
    }
    public Shooter GetInstance()
    {
        return m_Instance==null?m_Instance=new Shooter():m_Instance;
    }
    //TODO Dashboard Related;
}
