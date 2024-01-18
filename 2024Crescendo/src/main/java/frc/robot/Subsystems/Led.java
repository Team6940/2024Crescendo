package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LedConstants;

public class Led extends SubsystemBase{
    public static Led m_Instance;
    private AddressableLED m_Led;
    private AddressableLEDBuffer m_Buffer;
    Led(){
        LedConfig();
    }
    private void LedConfig(){
        m_Led = new AddressableLED(LedConstants.LedPort);
        m_Led.setLength(LedConstants.LedLength);
        m_Buffer = new AddressableLEDBuffer(LedConstants.LedLength);    //用来存储每一个led的数据
        for(int i=0; i<m_Buffer.getLength(); ++i){
            m_Buffer.setRGB(i, 0, 0, 0);
        }
        m_Led.setData(m_Buffer);
    }
    public void LedStart(){
        m_Led.start();
    }
    public void LedStop(){
        m_Led.stop();
    }
    public void SetRGB(int _R, int _G, int _B){
        for(int i=0; i<m_Buffer.getLength(); ++i){
            m_Buffer.setRGB(i, _R, _G, _B);
        }
        m_Led.setData(m_Buffer);
    }
    public static Led GetInstance(){
        return m_Instance==null?m_Instance = new Led():m_Instance;
    }
}
