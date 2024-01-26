// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Optional;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathLTV;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.Constants.GoalConstants.FieldElement;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team1706.FieldRelativeAccel;
import frc.robot.Library.team1706.FieldRelativeSpeed;
import frc.robot.Library.team95.BetterSwerveKinematics;
import frc.robot.Library.team95.BetterSwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SwerveDriveTrain extends SubsystemBase {

  private static SwerveDriveTrain instance = null;
  /** Creates a new SwerveDriveTrain. */
  private SwerveModule swerve_modules_[] = new SwerveModule[4];

  //public PigeonIMU gyro;
  public Pigeon2 gyro;
 // public PixyCamSPI mPixy;
  //byte PixySignature;

  public boolean auto = false;

  double preverror;
  double responsetime = 0.02;
  int i = 1;
  double HEAD_P = 0.01;
  double HEAD_I = 0;
  double HEAD_D = 0;
  PIDController headController = new PIDController(HEAD_P, HEAD_I, HEAD_D);

  public boolean isOpenLoop = true;

  public boolean whetherstoreyaw = false;

  public boolean autoPixy = false;

  private Field2d m_field = new Field2d();

  private FieldRelativeSpeed m_fieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeSpeed m_lastFieldRelVel = new FieldRelativeSpeed();
  private FieldRelativeAccel m_fieldRelAccel = new FieldRelativeAccel();

  private double gyroRollVelocity = 0;
  private double lastGyroRoll = 0;

  ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Swerve");
  boolean enanbleTelemetry = true;

  public final static SwerveDriveKinematics kDriveKinematics =
      new SwerveDriveKinematics(
        new Translation2d( SwerveConstants.kLength / 2,  SwerveConstants.kWidth / 2),//front left
        new Translation2d( SwerveConstants.kLength / 2, -SwerveConstants.kWidth / 2),//front right
        new Translation2d(-SwerveConstants.kLength / 2,  SwerveConstants.kWidth / 2),//back left
        new Translation2d(-SwerveConstants.kLength / 2, -SwerveConstants.kWidth / 2)//back right
  );

  public SwerveDrivePoseEstimator m_SwervePoseEstimator;

  public SwerveDriveTrain() {

    addShuffleboardDebug();
    gyro = new Pigeon2(SwerveConstants.PigeonIMUPort);

    // The coordinate system may be wrong 
    swerve_modules_[0] = new SwerveModule(1, 2, false,  false, 2500, true, true);//front left
    swerve_modules_[1] = new SwerveModule(7, 8, true, false, 230, true, true);//front right
    swerve_modules_[2] = new SwerveModule(3, 4, false,  false, 711,  false, false);//back left
    swerve_modules_[3] = new SwerveModule(5, 6, false, false, 2651,  false, false);//back right
    
  m_SwervePoseEstimator=new SwerveDrivePoseEstimator(
    SwerveConstants.swerveKinematics,
    new Rotation2d(0),
     getModulePositions(), new Pose2d()
);

    //ahrs = new AHRS(SPI.Port.kMXP);

    //mPixy = PixyCamSPI.getInstance();
    /* select cargo color for sig */
    //PixySignature = SmartDashboard.getBoolean("Debug/Pixy/alliance", false) ? Pixy2CCC.CCC_SIG1 : Pixy2CCC.CCC_SIG2;

    SmartDashboard.putData("Debug/Drive/Field", m_field);

  }
  
  public static SwerveDriveTrain getInstance() {
    if (instance == null){
      instance = new SwerveDriveTrain();
    }
    return instance;
  }

  public void Drive(Translation2d translation,double omega,boolean fieldRelative,boolean isOpenloop){
    var states = SwerveConstants.swerveKinematics.toSwerveModuleStates(
      fieldRelative ? 
      ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(), translation.getY(), omega, GetGyroRotation2d())
      : new ChassisSpeeds(translation.getX() , translation.getY(), omega)
    );

    BetterSwerveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeed);
      
    for(int i = 0;i < swerve_modules_.length;i++ ){
      swerve_modules_[i].SetDesiredState(states[i],isOpenloop);
    }
  }
/**
 * 这个是为了兼容旧代码蔡保留的，晚点删掉
 * @param desiredStates
 */
  public void SetModuleStates(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeed);
      for(int i = 0;i < swerve_modules_.length;i++){
        swerve_modules_[i].SetDesiredState(desiredStates[i], true);
      }  
  }public void SetModuleStates(BetterSwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.kMaxSpeed);
    for(int i = 0;i < swerve_modules_.length;i++){
      swerve_modules_[i].SetDesiredState(desiredStates[i], true);
    }  
}

      /**
     * What the module states should be in hold mode. The wheels will be put in an X pattern to prevent the robot from moving.
     * <p>
     * 0 -> Left Front
     * <p>
     * 1 -> Left Back
     * <p>
     * 2 -> Right Front
     * <p>
     * 3 -> Right Back
     */
    public static final SwerveModuleState[] HOLD_MODULE_STATES = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
  };
public Command followPathCommand(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    
    // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
    return 
        new FollowPathHolonomic(
            path,
            this::getPose, // Robot pose supplier
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
           new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    
}
  /**
  * What the module states should be set to when we start climbing. All the wheels will face forward to make the robot easy to
  * push once it is disabled.
  * <p>
  * 0 -> Left Front
  * <p>
  * 1 -> Left Back
  * <p>
  * 2 -> Right Front
  * <p>
  * 3 -> Right Back
  */
  public static final SwerveModuleState[] SWERVE_MODULE_STATE_FORWARD = {
        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(90))
  };

  public void setSWERVE_MODULE_STATE_FORWARD(){
    //SetModuleStates(SWERVE_MODULE_STATE_FORWARD); //TODO
  }

  public void setHOLD_MODULE_STATES(){
    SetModuleStates(HOLD_MODULE_STATES);  //TODO
  }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
      return m_SwervePoseEstimator.getEstimatedPosition();
    }


  public ChassisSpeeds getChassisSpeeds() {
    return kDriveKinematics.toChassisSpeeds(
            getStates());
  }

  /**
    * Calculate and set the requred SwerveModuleStates for a given ChassisSpeeds
    * 
    * @param speeds
    */
  public void setChassisSpeeds (ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = kDriveKinematics.toSwerveModuleStates(speeds); //Generate the swerve module states
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.kMaxSpeed);
    SetModuleStates(moduleStates);
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return new ChassisSpeeds(
        getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getCos() - getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getSin(),
        getChassisSpeeds().vyMetersPerSecond * getPose().getRotation().getCos() + getChassisSpeeds().vxMetersPerSecond * getPose().getRotation().getSin(),
        getChassisSpeeds().omegaRadiansPerSecond);
  }

  public double getFieldRelativeXVelocity() {
      return getFieldRelativeChassisSpeeds().vxMetersPerSecond;
  }

  public double getFieldRelativeYVelocity() {
      return getFieldRelativeChassisSpeeds().vyMetersPerSecond;
  }

  public double getFieldRelativeAngularVelocity() {
      return getFieldRelativeChassisSpeeds().omegaRadiansPerSecond;
  }


  public double GetHeading_Rad(){
    /*The unit is radian */
    return GetGyroRotation2d().getRadians();
  }

  public double GetHeading_Deg(){
    /*The unit is radian */
    return GetGyroRotation2d().getDegrees();
  }

  public void ZeroHeading(){
    whetherstoreyaw = true;
    // ahrs version
    //ahrs.reset();
    //ResetOdometry(new Pose2d());
    //Pigeon version
    zeroGyro();
  }

  public void WhetherStoreYaw(){
    whetherstoreyaw = false;
  }

  public void turnOnPixy(){
    autoPixy = true;
  }

  public void turnOffPixy(){
    autoPixy = false;
  }

  public void ResetOdometry(Pose2d pose){
    m_SwervePoseEstimator.resetPosition(GetGyroRotation2d(), getModulePositions(),pose);
    //for (int i = 0 ; i < swerve_modules_.length; i++){
    //  swerve_modules_[i].setPose(pose);
    //}
  }

  public Rotation2d GetGyroRotation2d(){
    // An offset will be needed if the robot doesn't face downfield
    
    // AHRS version
    //return ahrs.getRotation2d();

    //Pigeon Version
    //return Rotation2d.fromDegrees(gyro.getFusedHeading());
    return Rotation2d.fromDegrees(gyro.getRotation2d().getDegrees());
  }

  public double deadband(double x){
    return ((x < 0 ? -x : x) < 0.05) ? 0 : x;
  }

  public double calcYawStraight(double targetAngle, double currentAngle,double kP, double kD){
    //The WPILIB's version
    double headadjust = headController.calculate(currentAngle, targetAngle);
    return headadjust;
  }

  public double remainderf(double m , double n){
    return (m / n) > 0.0 ? m - Math.floor(m / n) * n : m - Math.ceil(m / n) * n;
  }

  public void setControlModeOpen(){
    isOpenLoop = false;
  }

  public void setControlModeClosed(){
    isOpenLoop = true;
  }

  public void resetOdometry(){
    m_SwervePoseEstimator.resetPosition(new Rotation2d(),getModulePositions(),new Pose2d());
  }

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[swerve_modules_.length];
    for (int i = 0;i < swerve_modules_.length;i++ ){
      states[i] = swerve_modules_[i].GetState();
    }
    return states;
  }
  public SwerveModulePosition[] getModulePositions()
  {
    SwerveModulePosition[] _Positions=new SwerveModulePosition[swerve_modules_.length];
    for(int i=0;i<swerve_modules_.length;++i)
    {
      _Positions[i]=swerve_modules_[i].GetPosition();
    }
    return _Positions;
  }

  public void zeroGyro(){
    gyro.setYaw(0);

  }
  public void CollaborateGyro()
  {
  
  }
  public void zeroGyro(double reset){
    gyro.setYaw(reset);
  }

  public double getRoll() {
    return gyro.getAngle();
    
  }

  public double GetYaw() {
    return gyro.getYaw().getValue();
  }
  
  public FieldRelativeSpeed getFieldRelativeSpeed() {
    return m_fieldRelVel;
  }

  public FieldRelativeAccel getFieldRelativeAccel() {
    return m_fieldRelAccel;
  }
  public void FixPoseEstimator(Pose2d _Pose2d,double _TimeStamp)
  {
    m_SwervePoseEstimator.addVisionMeasurement(_Pose2d,_TimeStamp);
  }
  public void getGyroRollVelocity() {
    gyroRollVelocity = (getRoll() - lastGyroRoll) / GlobalConstants.kLoopTime;
    lastGyroRoll = getRoll();
  }

  @Override
  public void periodic() {
    m_fieldRelVel = new FieldRelativeSpeed(getChassisSpeeds(), GetGyroRotation2d());
    m_fieldRelAccel = new FieldRelativeAccel(m_fieldRelVel, m_lastFieldRelVel, GlobalConstants.kLoopTime);
    m_lastFieldRelVel = m_fieldRelVel;
    
    SwerveModuleState[] moduleStates = getStates();
    for(int i=0;i<moduleStates.length;++i)
    {
      moduleStates[i].speedMetersPerSecond*=Constants.SwerveConstants.VelocityCorrectionFactor;
    }
    // This method will be called once per scheduler run
    m_SwervePoseEstimator.update(
      GetGyroRotation2d(), 
      getModulePositions());
    

    m_field.setRobotPose(getPose());
    if(enanbleTelemetry){
      SmartDashboard.putNumber("GetSpeed0", swerve_modules_[0].GetSpeed());
      SmartDashboard.putNumber("GetSpeed1", swerve_modules_[1].GetSpeed());
      SmartDashboard.putNumber("GetSpeed2", swerve_modules_[2].GetSpeed());
      SmartDashboard.putNumber("GetSpeed3", swerve_modules_[3].GetSpeed());

      SmartDashboard.putNumber("Debug/Drive/x meters", getPose().getX());
      SmartDashboard.putNumber("Debug/Drive/y meters", getPose().getY());
      SmartDashboard.putNumber("Debug/Drive/rot radians", getPose().getRotation().getDegrees());
      SmartDashboard.putBoolean("Debug/Drive/isOpenloop", isOpenLoop);
      
    }
    if(LimelightHelpers.getTV("limelight"))
    {
      RobotContainer.m_swerve.FixPoseEstimator(LimelightHelpers.getBotPose2d_wpiBlue("limelight"),Timer.getFPGATimestamp()-LimelightHelpers.getLatency_Capture("limelight")+LimelightHelpers.getLatency_Pipeline("limelight"));
    }

  }
  public Translation2d GetRobotToTargetTranslation(Translation2d _TargetTranslation2d)
  {
    return _TargetTranslation2d.minus(getPose().getTranslation());
  }
   public Translation2d GetRobotToTargetTranslation(Optional<Alliance> optional,Constants.GoalConstants.FieldElement _FieldElement)
  {
    return GoalConstants.FieldElementLoctaion[optional.get().ordinal()][_FieldElement.ordinal()].minus(getPose().getTranslation());
  }
  public Rotation2d GetRobotToTargetRotation(Translation2d _TargetTranslation2d)
  {
    return _TargetTranslation2d.minus(getPose().getTranslation()).getAngle().minus(getPose().getRotation());
  }
  public Rotation2d GetRobotToTargetRotation(Optional<Alliance> optional,Constants.GoalConstants.FieldElement _FieldElement)
  {
    return GetRobotToTargetRotation(GoalConstants.FieldElementLoctaion[optional.get().ordinal()][_FieldElement.ordinal()]);
  }
  private boolean IsOpenLoopMode(){
    return isOpenLoop;
  }

  private void addShuffleboardDebug(){
    swerveDriveTab.addNumber("GetSpeed0", () ->this.swerve_modules_[0].GetSpeed())
    .withPosition(0, 0)
    .withSize(1, 1);
    swerveDriveTab.addNumber("GetSpeed1", () ->this.swerve_modules_[1].GetSpeed())
    .withPosition(0, 1)
    .withSize(1, 1);
    swerveDriveTab.addNumber("GetSpeed2", () ->this.swerve_modules_[2].GetSpeed())
    .withPosition(0, 2)
    .withSize(1, 1);
    swerveDriveTab.addNumber("GetSpeed3", () ->this.swerve_modules_[3].GetSpeed())
    .withPosition(0, 3)
    .withSize(1, 1);    

    swerveDriveTab.addNumber("x meters", () ->this.getPose().getX())
    .withPosition(1, 0)
    .withSize(1, 1);    

    swerveDriveTab.addNumber("y meters", () ->this.getPose().getY())
    .withPosition(1, 1)
    .withSize(1, 1);    

    swerveDriveTab.addNumber("rot radians", () ->this.getPose().getRotation().getDegrees())
    .withPosition(1, 2)
    .withSize(1, 1);  
    
    swerveDriveTab.addNumber("FieldRelativeSpeedX", () ->this.getFieldRelativeXVelocity())
    .withPosition(2, 0)
    .withSize(1, 1);
    
    swerveDriveTab.addNumber("FieldRelativeSpeedY", () ->this.getFieldRelativeYVelocity())
    .withPosition(2, 1)
    .withSize(1, 1);
    
    swerveDriveTab.addNumber("FieldRelativeSpeedRot", () ->this.getFieldRelativeAngularVelocity())
    .withPosition(2, 2)
    .withSize(1, 1);
             
  }

  /*
   * 以下是Command中可能会使用到的一些函数，
   */
  public boolean IsAtPosition(Pose2d _Pos){
    return 
      getPose().minus(_Pos).getTranslation().getNorm()<=DriveConstants.kPositionTolerance
      && getPose().minus(_Pos).getRotation().getDegrees()<=DriveConstants.kRotationTolerance;
  }
  public void MoveToPose2D(Pose2d _Pos){
    
  }
}
