package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    public static Intake m_Instance;
    private static TalonFX m_Intake;
    private MotorOutputConfigs m_IntakeOutputConfigs=new MotorOutputConfigs();
    private Slot0Configs m_Slot0Configs = new Slot0Configs();
    private double m_MotorOutput;
    DigitalInput m_InfraredNoteSensor;
    /**
     * Set the output of the intake 1 stands for taking the note in
     * @param _Out range -1 to 1
     */
    public void SetIntakeOutput(double _Out)
    {
        m_Intake.set(_Out);
    }
    public double GetIntakeOutput(){
        return m_Intake.get();
    }
    /**
     * Is there a note in the intake?
     * @return true stands for there is a note
     */
    public boolean HasNote(){
        return !m_InfraredNoteSensor.get();//有球1没球0
    }
    /**
     * Get the Note into the intake and stop rotating when there is a note
     */
    public void NoteIn()
    {
        if(HasNote()) SetIntakeOutput(0);
        else SetIntakeOutput(IntakeConstants.m_NoteInPct);
    }
    /**
     * Get the Note out of the Intake into the shooter , keep rotating until there is no Note in the Intake
     */
    public void NoteOut()
    {
        if(!HasNote()) SetIntakeOutput(0);
        else SetIntakeOutput(IntakeConstants.m_NoteOutPct);
    }
    Intake()
    {
        m_Intake= new TalonFX(IntakeConstants.IntakeMotorPort);
        m_InfraredNoteSensor=new DigitalInput(IntakeConstants.m_NoteSensor);
        m_IntakeOutputConfigs.NeutralMode=NeutralModeValue.Brake;
        m_IntakeOutputConfigs.Inverted=InvertedValue.CounterClockwise_Positive;
        m_IntakeOutputConfigs.PeakForwardDutyCycle=1;
        m_IntakeOutputConfigs.PeakReverseDutyCycle=-1;
        // m_Slot0Configs.kP=IntakConstants.m_IntakeP;
        // m_Slot0Configs.kI=IntakConstants.m_IntakeI;
        // m_Slot0Configs.kD=IntakConstants.m_IntakeD;
        m_Intake.getConfigurator().apply(m_IntakeOutputConfigs);
        m_Intake.getConfigurator().apply(m_Slot0Configs);
    }
    public static Intake GetInstance()
    {
        return m_Instance==null?m_Instance=new Intake():m_Instance;
    }
}
