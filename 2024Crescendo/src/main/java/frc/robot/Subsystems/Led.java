package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Led extends SubsystemBase{
    public static Led m_Instance;
    Led(){

    }
    public static Led GetInstance(){
        return m_Instance==null?m_Instance = new Led():m_Instance;
    }
}
