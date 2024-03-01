package frc.robot.subsystem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Swerve {
    public static Swerve m_Instance;
    private VictorSPX VictorSPX_left1=new VictorSPX(1);
    private VictorSPX VictorSPX_left2=new VictorSPX(2);
    private VictorSPX VictorSPX_left3=new VictorSPX(3);
    private VictorSPX VictorSPX_left4=new VictorSPX(4);

    public Swerve() {
        SwerveConfig();
    }
    public static Swerve GetInstance()
    {
        return m_Instance==null? m_Instance=new Swerve():m_Instance;
    }
    private void SwerveConfig(){
        VictorSPX_left1.setInverted(false);
        VictorSPX_left1.setNeutralMode(NeutralMode.Coast);
        VictorSPX_left1.config_kP(0, 0.3);
        VictorSPX_left1.config_kI(0, 0);
        VictorSPX_left1.config_kD(0, 0.13);
        VictorSPX_left1.config_kF(0, 0.07);
        VictorSPX_left1.configPeakOutputForward(1.0);
        VictorSPX_left1.configPeakOutputReverse(-1.0);
        VictorSPX_left1.enableVoltageCompensation(true);

        VictorSPX_left2.setNeutralMode(NeutralMode.Coast);
        VictorSPX_left2.config_kP(0, 0.3);
        VictorSPX_left2.config_kI(0, 0);
        VictorSPX_left2.config_kD(0, 0.13);
        VictorSPX_left2.config_kF(0, 0.07);
        VictorSPX_left2.configPeakOutputForward(1.0);
        VictorSPX_left2.configPeakOutputReverse(-1.0);
        VictorSPX_left2.enableVoltageCompensation(true);
        VictorSPX_left2.setInverted(false);
        VictorSPX_left2.follow(VictorSPX_left1);

        VictorSPX_left3.setNeutralMode(NeutralMode.Coast);
        VictorSPX_left3.config_kP(0, 0.3);
        VictorSPX_left3.config_kI(0, 0);
        VictorSPX_left3.config_kD(0, 0.13);
        VictorSPX_left3.config_kF(0, 0.07);
        VictorSPX_left3.configPeakOutputForward(1.0);
        VictorSPX_left3.configPeakOutputReverse(-1.0);
        VictorSPX_left3.enableVoltageCompensation(true);
        VictorSPX_left3.setInverted(true);

        VictorSPX_left4.setNeutralMode(NeutralMode.Coast);
        VictorSPX_left4.config_kP(0, 0.3);
        VictorSPX_left4.config_kI(0, 0);
        VictorSPX_left4.config_kD(0, 0.13);
        VictorSPX_left4.config_kF(0, 0.07);
        VictorSPX_left4.configPeakOutputForward(1.0);
        VictorSPX_left4.configPeakOutputReverse(-1.0);
        VictorSPX_left4.enableVoltageCompensation(true);
        VictorSPX_left4.setInverted(true);
        VictorSPX_left4.follow(VictorSPX_left3);
    }

    public void drive(double _LeftSpeed,double _RightSpeed){
       VictorSPX_left1.set(VictorSPXControlMode.PercentOutput,_LeftSpeed);
       VictorSPX_left2.set(VictorSPXControlMode.PercentOutput,_LeftSpeed);
       VictorSPX_left3.set(VictorSPXControlMode.PercentOutput,_RightSpeed);
       VictorSPX_left4.set(VictorSPXControlMode.PercentOutput,_RightSpeed);
    }

}
