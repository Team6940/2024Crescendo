// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.ArmCoast;
import frc.robot.Commands.Auto.C102;
import frc.robot.Commands.Auto.UpGo;
import frc.robot.Commands.Auto.Close4note;
import frc.robot.Commands.Auto.C131;
import frc.robot.Commands.Auto.MiddleGo;
import frc.robot.Commands.Auto.LowGo;
import frc.robot.Commands.Climb.Climb;
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
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private LiveEditableValue<Double> m_ArmtargetAngle=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TuneArmAngle"));
  private LiveEditableValue<Double> m_ShooterRPS=new LiveEditableValue<Double>(0., SmartDashboard.getEntry("TuneShooterRPS"));
  private RobotContainer m_robotContainer;
  private int LastPov=12;
  private int AutoSelected=3;
  private int LastAuto=0;
  // private Double[] m_ListArm=new Double[40];
  // private Double[] m_ListTy=new Double[40];
  
  private ArrayList<Double> m_ListArm=new ArrayList<>();
  private ArrayList<Double> m_ListTy=new ArrayList<>();
  private int i=0;
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    
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
    
    if(RobotContainer.m_Arm.GetArmPosition()>=65)
    {
      RobotContainer.m_Arm.SetBrake();
    }
  if(RobotContainer.m_driverController.getAButtonPressed())
    {
      AutoSelected+=1;
      AutoSelected%=4;
      
    }
    if(AutoSelected!=LastAuto)
    switch(AutoSelected)
    {
      case 0:
        m_autonomousCommand=new UpGo();
        break;
      case 1:
        m_autonomousCommand=new C102();
        break;
      case 2:
        m_autonomousCommand=new MiddleGo();
        break;
      case 3:
        m_autonomousCommand=new LowGo();
        break;
  
    }
    LastAuto=AutoSelected;
        SmartDashboard.putString("AutoSelected", m_autonomousCommand.toString());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // new Close4note().withTimeout(15).schedule();
    m_autonomousCommand.withTimeout(15).schedule();
    }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // RobotContainer.m_swerve.ResetOdometry(new Pose2d(0.67,7.65,new Rotation2d(0)));
    // RobotContainer.m_swerve.zeroGyro();
  }

  @Override
  public void teleopPeriodic() {
    if(RobotContainer.m_driverController.getYButtonPressed())
    {
      m_ListArm.add(AutoShoot.LastShootArmAngle);
      m_ListTy.add(AutoShoot.LastShootty);
      RobotContainer.m_swerve.resetOdometry();
      // SmartDashboard.putNumberArray("Limelight Data",m_ListTy.toArray(new Double[m_ListTy.size()]));
      // SmartDashboard.putNumberArray("ArmData",m_ListArm.toArray(new Double[m_ListTy.size()]));
    
    }
    if(RobotContainer.m_driverController.getPOV()==0&&LastPov!=0)
    {
      RobotContainer.m_Arm.ArmFixingFactor+=0.5;
    }
    if(RobotContainer.m_driverController.getPOV()==180&&LastPov!=180)
    {
      RobotContainer.m_Arm.ArmFixingFactor-=0.5;
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
    else if(RobotContainer.m_driverController.getBButtonPressed())
    {
      new Climb().schedule();
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
    // CommandScheduler.getInstance().cancelAll();
    RobotContainer.m_swerve.zeroGyro();
  }

  @Override
  public void testPeriodic() {
     if(RobotContainer.m_driverController.getYButtonPressed())
    {
      m_ListArm.add(AutoShoot.LastShootArmAngle);
      m_ListTy.add(AutoShoot.LastShootty);
      SmartDashboard.putNumberArray("Limelight Data",m_ListTy.toArray(new Double[m_ListTy.size()]));
      SmartDashboard.putNumberArray("ArmData",m_ListArm.toArray(new Double[m_ListTy.size()]));
    
    }
    if(RobotContainer.m_driverController.getPOV()==0&&LastPov!=0)
    {
      RobotContainer.m_Arm.ArmFixingFactor+=0.5;
    }
    if(RobotContainer.m_driverController.getPOV()==180&&LastPov!=180)
    {
      RobotContainer.m_Arm.ArmFixingFactor-=0.5;
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
    else if(RobotContainer.m_driverController.getBButtonPressed())
    {
      new Climb().schedule();
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
  public void testExit() {}
}
