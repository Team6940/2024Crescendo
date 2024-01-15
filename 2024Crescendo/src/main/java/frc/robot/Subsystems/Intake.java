package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public static Intake m_Instance;
    /**
     * Set the output of the intake 1 stands for taking the note in
     * @param _Out range -1 to 1
     */
    void SetIntakeOutput(double _Out)
    {

    }
    /**
     * Is there a note in the intake?
     * @return true stands for there is a note
     */
    boolean HasNote()
    {
        return true;
    }
    /**
     * Get the Note into the intake and stop rotating when there is a note
     */
    void NoteIn()
    {

    }
    /**
     * Get the Note out of the Intake into the shooter , keep rotating until there is no Note in the Intake
     */
    void NoteOut()
    {

    }
    Intake()
    {

    }
    public Intake GetInstance()
    {
        return m_Instance==null?m_Instance=new Intake():m_Instance;
    }
}
