// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.ArmCoast;
import frc.robot.Commands.Auto.Close4note;
import frc.robot.Commands.Auto.Close5note;
import frc.robot.Commands.Auto.FarUp4note;
import frc.robot.Commands.Auto.HAHA;
import frc.robot.Commands.SwerveControl.SwerveControll;
import frc.robot.Constants.ShootCommandConstants;
import frc.robot.Constants.ShootConstants;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team3476.net.editing.LiveEditableValue;
import frc.robot.Subsystems.ImprovedXboxController;
import frc.robot.Subsystems.ImprovedXboxController.Button;
import frc.robot.Commands.NoteIntake.NoteIntake;
import frc.robot.Commands.Shoot.AMP;
import frc.robot.Commands.Shoot.AutoShoot;
import frc.robot.Commands.Shoot.NewShoot;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private LiveEditableValue<Double> m_ArmtargetAngle=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TuneArmAngle"));
  private LiveEditableValue<Double> m_ShooterRPS=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TuneShooterRPS"));
  private RobotContainer m_robotContainer;
  private int LastPov=12;
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    RobotContainer.m_Arm.SetCoast();
  }

  @Override
  public void disabledPeriodic() {
    if(RobotContainer.m_Arm.GetArmPosition()>=75)
    {
      RobotContainer.m_Arm.SetBrake();
    }

  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // new Close4note().withTimeout(15).schedule();
    new HAHA().withTimeout(15).schedule();
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
    // RobotContainer.m_swerve.ResetOdometry(new Pose2d(0.67,7.65,new Rotation2d(0)));
    RobotContainer.m_swerve.zeroGyro();
  }

  @Override
  public void teleopPeriodic() {
    if(RobotContainer.m_driverController.getPOV()==0&&LastPov!=0)
    {
      RobotContainer.m_Arm.ArmFixingFactor+=1;
    }
    if(RobotContainer.m_driverController.getPOV()==180&&LastPov!=180)
    {
      RobotContainer.m_Arm.ArmFixingFactor-=1;
    }
    if(RobotContainer.m_driverController.getLeftBumperPressed())
    {
      // RobotContainer.m_Arm.SetArmPosition(4);
      // RobotContainer.m_Intake.NoteIn();;
      new NoteIntake(Button.kLeftBumper.value).schedule();
    }
    else if(RobotContainer.m_driverController.getLeftTrigger())
    {
      RobotContainer.m_Intake.SetIntakeOutput(-0.2);
      RobotContainer.m_Shooter.SetPct(-0.2);
    }
    else if(RobotContainer.m_driverController.getRightBumperPressed())
    {
      // RobotContainer.m_Arm.SetArmPosition(12);
      // RobotContainer.m_Shooter.SetRPS(50);
      // if(RobotContainer.m_driverController.getRightTrigger())

      // {
      //   RobotContainer.m_Intake.SetIntakeOutput(1);
      // }
      // new NewShoot(m_ArmtargetAngle.get(), m_ShooterRPS.get(), Button.kRightBumper.value, Button.kRightBumper.value).schedule();;
        new AutoShoot().schedule();
    }
    else if(RobotContainer.m_driverController.getXButtonPressed())
    {
      new NewShoot(ShootCommandConstants.PassSet.ArmAngle, ShootCommandConstants.PassSet.ShooterRPS, Button.kX.value, Button.kAutoButton.value).schedule();;
    }
    else if(RobotContainer.m_driverController.getAButtonPressed())
    {
      new NewShoot(ShootCommandConstants.AMPSet.ArmAngle,ShootCommandConstants.AMPSet.ShooterRPS ,Button.kA.value , Button.kRightTrigger.value).schedule();
    }
    else 
    {
      // RobotContainer.m_Intake.NoteOut();;;
      // RobotContainer.m_Shooter.SetPct(0);
      // RobotContainer.m_Intake.SetIntakeOutput(0);
      // RobotContainer.m_Arm.SetArmPosition(20);
    }
    LastPov=RobotContainer.m_driverController.getPOV();
    
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
