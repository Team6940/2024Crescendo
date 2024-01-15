package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    public static Arm m_Instance;
    /**
     * Set the arm to the target Position
     * @param _Degree degree
     */
    void SetArmPosition(double _Degree)
    {

    }
    /**
     * Get the Arm's Postion In degree
     * @return the Arm's Position Compare to the Initial Degree
     */
    double GetArmPosition()
    {
        return 0.;
    }
    /**
     * Reset Arm's Position in degree
     * @param _Degree the Position Arm is now on
     */
    void ResetArmPostion(double _Degree)
    {

    }
    Arm()
    {

    }
    public Arm GetInstance()
    {
        return m_Instance==null?m_Instance=new Arm():m_Instance;
    }
}
