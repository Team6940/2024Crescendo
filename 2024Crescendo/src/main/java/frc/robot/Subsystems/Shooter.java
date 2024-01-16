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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Shooter extends SubsystemBase {
    public static Shooter m_Instance;
    private static TalonFX m_ShooterLeft;   //LeftShooter
    private static TalonFX m_ShooterRght;   //RightShooter
    //private Slot0Configs m_Shooter_Slot0Configs = new Slot0Configs();
    //private MotorOutputConfigs m_Shooter_MotorOutputConfigs = new MotorOutputConfigs();
    //private VoltageConfigs m_ShooterVoltageConfigs = new VoltageConfigs();
    private TalonFXConfiguration m_Shooter_Configuration = new TalonFXConfiguration();
    final VelocityDutyCycle m_request = new VelocityDutyCycle(0, 0, false, 0, 0, true, true, true);
    // private boolean m_Enabled = true;
    private double m_TargetSpeed;   //Parametre is RPS;
    Shooter()
    {
        
    }
    /**
     * Set the Rotation speed of the shooter, Positive stands for get the Note out
     * @param _RPS
     */
    void SetRPS(double _RPS)
    {
        // if(!m_Enabled) return;
        m_TargetSpeed = _RPS;
        // if(_RPS == 0){
        //     m_ShooterLeft.stopMotor();
        // }
        // else{
            m_ShooterLeft.setControl(m_request.withVelocity(_RPS));
        // }
    }
    /**
     * Get the Rotation speed of the shooter, Positive stands for get the Note out
     */
    double GetRPS()
    {
        // if(m_Enabled){
            return m_ShooterLeft.getVelocity().getValue();
        // }
        // return 0.;
    }
    /**
     * Get the Target Rotation speed of the shooter, Positive stands for get the Note out
     */
    double GetTargetRPS()
    {
        // if(m_Enabled){
            return m_TargetSpeed;
        // }
        // return 0.;
    }
    /**
     * Is shooter at TargetRPM
     * @return true means the shooter is at Target RPM
     */
    boolean IsAtTargetRPM()
    {
        // if(m_Enabled){
            return Math.abs(GetRPS()-GetTargetRPS())<ShooterConstants.kShooterTorlerance;
        // }
        // return false;
    }
    //TODO Dashboard Related;
}
