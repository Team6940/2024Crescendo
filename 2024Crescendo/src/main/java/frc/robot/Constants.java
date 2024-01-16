// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Library.team1706.LinearInterpolationTable;
import frc.robot.Library.team95.BetterSwerveKinematics;

import java.awt.geom.Point2D;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final class SemiAutoConstants{
        public static final double kSemiAutoVelocityP=6 ;//TODO
        public static final double kSemiAutoVelocityI=0.00;//TODO
        public static final double kSemiAutoVelocityD=0.0;//TODO
        public static final Constraints kSemiAutoVelocityConstrants =new Constraints(6,2);//TODO
        public static final double SemiAutoVelocityMax=2;
        public static final double kSemiAutoOmegaP=5;//TODO
        public static final double kSemiAutoOmegaI=0.0;//TODO
        public static final double kSemiAutoOmegaD=0.0;//TODO
        public static final Constraints kSemiAutoOmegaConstrants =new Constraints(3,100);//TODO
        public static final double SemiAutoOmegaSlewRate=2;
        public static final double SemiAutoOmegaMax=2;
    }
    public static final class IntakeConstants {
        public static final int IntakerPort = 11;
        public static final int IntakerSolenoidPort = 0;
        public static final boolean vSwitchIntake = false;
        public static final double INTAKE_OPEN_TIME = 0.0;
        public static final double IntakeOutput = 1;
        public static final double INTAKE_EJECTION_SPEED = -0.5;
    }
    public static final class HoodConstants {
        
        public static final double kMinAngle = 0.5;
        public static final double kMaxAngle = 38;
        public static final double kHoodTolerance = 1.5;//degrees
        
        public static int HoodMotorPort = 20; //14
        public static double HOOD_GEAR_RATIO = (432/35)*(54/24)*(10/1) ;  
        
        public static final double HOOD_HOME_ANGLE = 0; 
        public static final double HOOD_MAX_ANGLE = 50; 
        public static final double HOOD_MIN_ANGLE = 0;
        
        public static final double HOOD_EJECT_ANGLE = 10;  //TODO
    }
    public static final class IntakConstants {
        public static final int IntakeMotorPort=0;
        public static final int m_NoteSensor=0;
        public static final double m_NoteInPct=0.;
        // public static final double m_IntakeP=0.;
        // public static final double m_IntakeI=0.;
        // public static final double m_IntakeD=0.;
    } 
    public static final class ArmConstants{
        public static final int ArmMotorPort=0;
        public static final double ArmMovementSpeed=0.;
        public static final double ArmDegreeTolerance=0.;
        public static final double m_ArmP=0.;
        public static final double m_ArmI=0.;
        public static final double m_ArmD=0.;
        public static final double m_ArmVelocity=0.;
        public static final double m_ArmAcceleration=0.;
    }
    public static final class GlobalConstants {
        public static final double kLoopTime = 0.020;
        public static final float INF = (float)Math.pow(10, 5); // This represents the Infinite
    }
    
    public static final class LimelightConstants {
        // Limelight Constants
        public static final double LL_HORIZONTAL_CORRECTION = 0.;   //limelight水平修正， + is left
        public static final Translation3d LL_POS_TO_ROBOT =        //limelight相对外框中心位置
            new Translation3d(0., 0., 0.);  //TODO
        public static final double LL_MOUNT_ANGLE = 0.; /* limelight竖直方向固定角度，+ is up */   //TODO
        public static final double kTrackTolerance = 0.; // Allowable Limelight angle(degree) error in radians //TODO
        public static final Translation3d[] Apriltag_Position = {       //x is for front/back, y is for left/right, z is for height
            //TODO
        };
        public static final Translation3d[] Apriltag_Facing = {
            //TODO
        };
}
    
    public static final class GoalConstants {
        public static final Translation2d kGoalLocation = new Translation2d(8.23, 4.115);
        public static final Translation2d kWrongBallGoal = new Translation2d(5.50, 4.115);
        public static final Translation2d kHangerLocation = new Translation2d(2.00, 6.00);

        public static final double LL_UPPER_HUB_HEIGHT = 2.64;
        public static final double CARGO_DIAMETER = 0.64;
        public static final double UPPER_HUB_DIAMETER = 1.22;
    }
    public static final class DriveConstants {
        public static final double kMaxAcceleration = 3.0;
        public static final double kMaxAngularSpeed = Math.PI*1.5; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly
        public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly

        public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                          // read when not in use (Eliminates "Stick Drift")
        public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                          // when aimed at a 45deg angle (Such that X and Y are are
                                                          // maximized simultaneously)
        public static final double kTranslationSlew = 3.5;
        public static final double kRotationSlew = 3.00;
    }
    
    public static final class SwerveConstants {	
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = 29/15*60/15;
        public static final double angleGearRatio = 56/6*60/10; 
        
        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.69552 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.8378 / 12);
        public static final double driveKA = (0.44473 / 12);

        public static final double piviotKS = (1.322); //divide by 12 to convert from volts to percent output for CTRE
        public static final double piviotKV = (0.0052807);
        public static final double piviotKA = (0.44473 / 12);

        // Pigeon Port
        public static final int PigeonIMUPort = 17;

        public static double kDriveMotorMaxOutput = 1;
        public static double kPivotMotorMaxOutput = 1;
    
        public static double kDriveMotorNeutralDeadband = 0;
        public static double kPivotMotorNeutralDeadband = 0;
        
        public static double VelocityCorrectionFactor = 0.5;//TODO
        /**
         * The first version of PID parameters
         *                         Value
         * @param kDriveMotorkP    0.025
         * @param kDriveMotorkI    0.0016
         * @param kDriveMotorkD    2.5
         * @param kDriveMotorkF    0.06
         * @param kDriveMotorIZone 240
         * 
         * 车子在行驶过程中基本不抖动，底盘PID大部分情况下是正常的。
         * 
         */
        public static double kDriveMotorkP = 0.15; // 5e-2 0.05   0.025
        public static double kDriveMotorkI = 0; //5e-4 0.005  0.0016
        public static double kDriveMotorkD = 0.00; //   5e-0 5 1.5  2.5
        public static double kDriveMotorkF = 0.042;//   0.045       0.06
        public static double kDriveMotorIZone = 0;// 90          240
        public static double kSensorVelocityMeasPeriod = 10;
    
        public static double kPivotMotorkP = 6;//3
        public static double kPivotMotorkI = 0;
        public static double kPivotMotorkD = 100;//100
        public static double kPivotMotorF = 0;
        public static double kPivotMotorkIZone = 0;
        public static double motionCruiseVelocity = 1200;
        public static double motionAcceleration = 3500;    
    
        public static double kLoopSeconds = 0.0;
    
        public static double kDriveMotorReductionRatio = 1.0 / (29/15*60/15); //29/15*60/15
        public static double kPivotMotorReductionRatio = 1.0 / (56/6*60/10); //56/6*60/10
        public static double kDriveEncoderReductionRatio = 1.0 / (29 / 15 * 60 / 15);
        public static double kDriveEncoderReductionRatioTest = (29 / 15 * 60 / 15);
        public static double kPivotEncoderReductionRatio = -1.0 / 1.0;
    
        public static final double FALCON_TICS_PER_ROTATION = 2048.0;
        public static final double TALON_TICS_PER_ROTATION = 4096.0;
    
        public static double kDriveEncoderResolution = 2048.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad
        public static double kPivotEncoderResolution = 4096.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad
    
        public static double kWidth  = 0.572936;//The unit is 0.342_m
        public static double kLength = 0.572936;//The unit is 0.342_m
    
        public static double kWheelDiameter = 0.093;//The unit is meter
    
        public static double kPeriod = 20;//The unit is 20_ms
    
        public static double kMaxSpeed = 4;//The unit is meters per second 每个轮子的可以达到的最大转速，单位是m/s
    
       
        public static final BetterSwerveKinematics swerveKinematics = new BetterSwerveKinematics(//define the position of each swervemodule by creating a coordinate system
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, -kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, -kWidth / 2.0));
    
        public static final double wheelCircumference = kWheelDiameter * Math.PI;
    
        public static final double MAX_SPEED_METERSperSECOND = 21900/0.1/(driveGearRatio*2048)*wheelCircumference;
        public static final double METERSperROBOT_REVOLUTION =  2*Math.PI*(kWidth/2*1.414213);//Math.sqrt(2)
        public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND / METERSperROBOT_REVOLUTION
                * (2 * Math.PI);
        
        public static final int SWERVE_MOTOR_CURRENT_LIMIT = 15;
        public static final int SWERVE_DRIVE_MOTOR_CURRENT_LIMIT = 15;
        public static final int SWERVE_DRIVE_VOLTAGE_LIMIT = 12;
    }
    public final class HopperConstants
    {
        public static final int UpFrontMotorPort=12;
        public static final int UpBackMotorPort=15;
        public static final int BottomMotorPort=19;
        public static final int  UpBallSensorChannel=4;
        public static final int  DownBallSensor=9;

        public static final double HopperOutput=0.7;
    }
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 6;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 16;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 16;
	
		public static final double kSlowMaxSpeedMetersPerSecond = 2.0;
		public static final double kSlowMaxAccelerationMetersPerSecondSquared = 3;

		public static final double kFastMaxSpeedMetersPerSecond = 2;
		public static final double kFastMaxAccelerationMetersPerSecondSquared = 2;
		
        public static final double kPXController = 8;
        public static final double kPYController = 8;
        public static final double kPThetaController = 6;
        public static final double kIThetaController = 0;
        public static final double kDThetaController = 0;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                        Math.PI, Math.PI);
                
        public static final TrapezoidProfile.Constraints kThetaAimControllerConstraints =
            new TrapezoidProfile.Constraints(
                        3 * Math.PI, 3 * Math.PI);
    }
    public static class ClimberConstants
    {
        public static final int ClimberOutUnits=870000;
        public static final int ClimberBackUnits=400000;//TODO
        public static final int ClimberPort=16;
    }
    public static class ShooterConstants{
        public static final int SHOOTER_L_MASTER_ID = 0;  //TODO
        public static final int SHOOTER_R_MASTER_ID = 0;  //TODO
        public static double SHOOTER_KP = 0.;             //TODO
        public static double SHOOTER_KI = 0.;             //TODO
        public static double SHOOTER_KD = 0.;      
        public static double kShooterF=0.;       //TODO
        public static double kShooterTorlerance=0.;//TODO
        private static final Point2D[] kHoodPoints = new Point2D.Double[] {
            // (distance, ty-angle)
            new Point2D.Double(2.48/*90*/,7), //
            new Point2D.Double(3/*105*/, 9), //
            new Point2D.Double(3.5/*120*/, 12.00), //
            new Point2D.Double(4/*135*/, 14.00), //
            new Point2D.Double(4.5/*150*/, 16.00), //
            new Point2D.Double(5/*165*/, 19.00), //
            new Point2D.Double(5.5/*180*/, 22.00), //
            new Point2D.Double(6/*195*/, 24.00), //
            new Point2D.Double(6.45/*210*/, 26.0), //
            new Point2D.Double(7.06/*225*/, 29),//
            new Point2D.Double(7.53/*270*/, 32),
        };
        public static final LinearInterpolationTable kHoodTable = new LinearInterpolationTable(kHoodPoints);
    
        private static final Point2D[] kRPSPoints = new Point2D.Double[] {
            // (distance, shooterSpeedRPM)
            new Point2D.Double(2.48/*90*/, 2300), //
            new Point2D.Double(3/*135*/, 2400), //
            new Point2D.Double(3.5/*150*/, 2450), //
            new Point2D.Double(4/*165*/, 2500), //
            new Point2D.Double(4.5/*180*/, 2570), //
            new Point2D.Double(5/*195*/, 2650), //
            new Point2D.Double(5.5/*210*/, 2680), //
            new Point2D.Double(6/*225*/, 2720), //
            new Point2D.Double(6.45/*240*/, 2720),
            new Point2D.Double(7.06/*270*/, 2720),
            new Point2D.Double(7.53/*270*/, 2720),
        };        
        public static final LinearInterpolationTable kRPMTable = new LinearInterpolationTable(kRPSPoints);


    }
}
