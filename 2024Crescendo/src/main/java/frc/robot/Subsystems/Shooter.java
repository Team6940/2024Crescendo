package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Library.team1678.math.Conversions;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


public class Shooter extends SubsystemBase {
    public static Shooter m_Instance;
    private static TalonFX m_ShooterLeft;   //LeftShooter
    private static TalonFX m_ShooterRght;   //RightShooter
    //private Slot0Configs m_Shooter_Slot0Configs = new Slot0Configs();
    //private MotorOutputConfigs m_Shooter_MotorOutputConfigs = new MotorOutputConfigs();
    //private VoltageConfigs m_ShooterVoltageConfigs = new VoltageConfigs();
    private TalonFXConfiguration m_Shooter_Configuration = new TalonFXConfiguration();
    final DutyCycleOut m_Shooter_DutyCycleOut = new DutyCycleOut(0);
    final VelocityDutyCycle m_request = new VelocityDutyCycle(0, 0, false, 0, 0, true, true, true);
    // private boolean m_Enabled = true;
    private double m_TargetSpeed;   //Parametre is RPS;
    Shooter()
    {
        ShooterConfig();
    }
    public static Shooter GetInstance()
    {
        return m_Instance==null?m_Instance=new Shooter():m_Instance;
    }
    private void ShooterConfig() 
    {
        m_ShooterLeft = new TalonFX(ShooterConstants.SHOOTER_L_MASTER_ID);
        m_ShooterLeft.setInverted(true);
        m_ShooterLeft.setNeutralMode(NeutralModeValue.Coast);
        m_Shooter_Configuration.Slot0.kP = ShooterConstants.SHOOTER_KP;
        m_Shooter_Configuration.Slot0.kI = ShooterConstants.SHOOTER_KI;
        m_Shooter_Configuration.Slot0.kD = ShooterConstants.SHOOTER_KD;
        m_Shooter_Configuration.MotorOutput.PeakForwardDutyCycle = 1.;
        m_Shooter_Configuration.MotorOutput.PeakReverseDutyCycle = -1.;
        //TODO Configs of Sensors;
        //TODO Configs of VoltageCompSaturation
        //TODO Configs of Velocity Measurements
        m_ShooterLeft.getConfigurator().apply(m_Shooter_Configuration);
        //Right Motor，同上
        m_ShooterRght.setInverted(false);   //TODO
        m_ShooterRght.getConfigurator().apply(m_Shooter_Configuration);
        m_ShooterRght.setControl(new Follower(m_ShooterLeft.getDeviceID(), false));

    }
    /* 
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
