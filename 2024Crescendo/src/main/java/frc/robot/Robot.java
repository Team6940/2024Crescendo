// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.Auto.Close4note;
import frc.robot.Commands.Auto.FarUp4note;
import frc.robot.Commands.SwerveControl.SwerveControll;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Subsystems.ImprovedXboxController;
import frc.robot.Commands.NoteIntake.NoteIntake;
import frc.robot.Commands.Shoot.NewShoot;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
   RobotContainer.m_swerve.setDefaultCommand(new SwerveControll());
    m_robotContainer = new RobotContainer();
  }


  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
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
    RobotContainer.m_swerve.ResetOdometry(new Pose2d(0.67,7.43,new Rotation2d(0)));
    RobotContainer.m_swerve.zeroGyro();
  }

  @Override
  public void teleopPeriodic() {
    
    if(RobotContainer.m_driverController.getLeftBumperPressed())
    {
      // RobotContainer.m_Arm.SetArmPosition(4);
      // RobotContainer.m_Intake.NoteIn();;
      new NoteIntake().schedule();
    }
    else if(RobotContainer.m_driverController.getLeftTrigger())
    {
      RobotContainer.m_Intake.SetIntakeOutput(-0.2);
      RobotContainer.m_Shooter.SetPct(-0.2);
    }
    else if(RobotContainer.m_driverController.getRightBumperPressed())
    {
      // RobotContainer.m_Arm.SetArmPosition(20);
      // RobotContainer.m_Shooter.SetPct(0.8);
      // if(RobotContainer.m_driverController.getRightTrigger())

      // {
      //   RobotContainer.m_Intake.SetIntakeOutput(1);
      // }
      new NewShoot(
        ShootCommandConstants.SpeakerSet[0], 
        ImprovedXboxController.Button.kRightBumper.value,
        ImprovedXboxController.Button.kRightTrigger.value
      ).schedule();
    }
    else if(RobotContainer.m_driverController.getAButtonPressed())
    {
      new NewShoot(ShootCommandConstants.AMPSet, new Pose2d(1.86,7.43,new Rotation2d(Math.toRadians(-90))), ImprovedXboxController.Button.kA.value, ImprovedXboxController.Button.kB.value).schedule();;
    }
    else 
    {
      // RobotContainer.m_Intake.NoteOut();;;
      // RobotContainer.m_Shooter.SetPct(0);
      // RobotContainer.m_Intake.SetIntakeOutput(0);
      // RobotContainer.m_Arm.SetArmPosition(20);
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
