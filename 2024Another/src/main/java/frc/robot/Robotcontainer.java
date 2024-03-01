package frc.robot;

import frc.robot.subsystem.Swerve;
import edu.wpi.first.wpilibj.XboxController;


public class Robotcontainer {
  // The robot's subsystems and commands are defined here...
  public static XboxController m_driverController= new XboxController(0);
  public static Swerve m_Swerve = new Swerve();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Robotcontainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }

}