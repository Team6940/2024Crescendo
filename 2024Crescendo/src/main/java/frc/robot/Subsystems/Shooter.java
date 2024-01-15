package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    public static Shooter m_Instance;
    /**
     * Set the Rotation speed of the shooter, Positive stands for get the Note out
     * @param _RPS
     */
    void SetRPS(double _RPS)
    {

    }
    /**
     * Get the Rotation speed of the shooter, Positive stands for get the Note out
     */
    double GetRPS()
    {
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
    Shooter()
    {

    }
    public Shooter GetInstance()
    {
        return m_Instance==null?m_Instance=new Shooter():m_Instance;
    }
    
}
