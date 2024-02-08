// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Auto.Close4note;
import frc.robot.Commands.Auto.FarUp4note;
import frc.robot.Commands.SwerveControl.SwerveControll;
import frc.robot.Library.LimelightHelper.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
   m_robotContainer.m_swerve.setDefaultCommand(new SwerveControll());
    m_robotContainer = new RobotContainer();
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if(RobotContainer.m_driverController.getLeftBumper())
    {
      RobotContainer.m_Arm.SetArmPosition(4);
      RobotContainer.m_Intake.NoteIn();;

    }
    else if(RobotContainer.m_driverController.getLeftTriggerAxis()>0.3)
    {
      RobotContainer.m_Intake.SetIntakeOutput(-0.2);
      RobotContainer.m_Shooter.SetPct(-0.2);
    }
    else if(RobotContainer.m_driverController.getRightBumper())
    {
      RobotContainer.m_Arm.SetArmPosition(26);
      RobotContainer.m_Shooter.SetRPS(60);;;
      if(RobotContainer.m_driverController.getRightTriggerAxis()>0.3)
      {
        RobotContainer.m_Intake.SetIntakeOutput(1);
      }
    }
    else 
    {
      RobotContainer.m_Shooter.SetPct(0);
      RobotContainer.m_Intake.NoteOut();;;
      RobotContainer.m_Arm.SetArmPosition(20);
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    new FarUp4note().schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    RobotContainer.m_swerve.resetOdometry();
    RobotContainer.m_swerve.zeroGyro();
  }

  @Override
  public void teleopPeriodic() {
    if(m_robotContainer.m_driverController.getAButton())
    {
      
    }
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
