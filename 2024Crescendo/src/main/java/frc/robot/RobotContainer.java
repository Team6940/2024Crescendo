// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Led;
import frc.robot.Subsystems.Limelight;
import frc.robot.Subsystems.SwerveDriveTrain;

public class RobotContainer {
   public static SwerveDriveTrain m_swerve = SwerveDriveTrain.getInstance();
   public static Limelight m_Limelight= Limelight.GetInstance();
   public static Arm m_Arm=Arm.GetInstance();
   public static Shooter m_Shooter=Shooter.GetInstance();
   public static Intake m_Intake=Intake.GetInstance();
 public static XboxController m_driverController= new XboxController(0);
  
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

}
