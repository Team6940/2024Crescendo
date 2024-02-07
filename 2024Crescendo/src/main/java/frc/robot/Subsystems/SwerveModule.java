// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Library.team95.BetterSwerveModuleState;
public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final TalonFX drive_motor_;
  private final TalonSRX pivot_motor_;
  private double lastAngle;

  private Slot0Configs m_drive_slot0Configs = new Slot0Configs();
  private MotorOutputConfigs m_MotorOutputConfigs =new MotorOutputConfigs();
  final DutyCycleOut m_DutyCycleOut = new DutyCycleOut(0);
  final VelocityDutyCycle m_request = new VelocityDutyCycle(0,0,false,0,0,true, true, true);
  
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

  private int offset_;

  private double drive_motor_inverted = 1.0;
  private double pivot_motor_inverted = 1.0;
    
  private boolean drive_motor_output_enabled = true;
  private boolean pivot_motor_output_enabled = true;
  private double pivot_encoder_inverted = 1.0;

  public SwerveModule(int driveDeviceNumber, int pivotDeviceNumber,
                      boolean driveMotorInvert, boolean pivotMotorInvert,
                      int pivotEncoderOffset, boolean pivotEncoderPhase,
                      boolean pivotEncoderInvert) {
    drive_motor_ = new TalonFX(driveDeviceNumber);
    pivot_motor_ = new WPI_TalonSRX(pivotDeviceNumber); 

    drive_motor_.setNeutralMode(NeutralModeValue.Brake);
    pivot_motor_.setNeutralMode(NeutralMode.Coast);
    m_MotorOutputConfigs.PeakForwardDutyCycle=SwerveConstants.kDriveMotorMaxOutput;
    m_MotorOutputConfigs.PeakReverseDutyCycle=-SwerveConstants.kDriveMotorMaxOutput;
    drive_motor_.getConfigurator().apply(m_MotorOutputConfigs);
  
    pivot_motor_.configPeakOutputForward( SwerveConstants.kPivotMotorMaxOutput);
    pivot_motor_.configPeakOutputReverse(-SwerveConstants.kPivotMotorMaxOutput);
    
    pivot_motor_.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    m_drive_slot0Configs.kP=SwerveConstants.kDriveMotorkP;
    m_drive_slot0Configs.kI=SwerveConstants.kDriveMotorkI;
    m_drive_slot0Configs.kD=SwerveConstants.kDriveMotorkD;
    m_drive_slot0Configs.kV=SwerveConstants.kDriveMotorkF;
                        
    
    drive_motor_.getConfigurator().apply(m_drive_slot0Configs);
  
    pivot_motor_.config_kP(0, SwerveConstants.kPivotMotorkP);
    pivot_motor_.config_kI(0, SwerveConstants.kPivotMotorkI);
    pivot_motor_.config_kD(0, SwerveConstants.kPivotMotorkD);
    pivot_motor_.config_kF(0, SwerveConstants.kPivotMotorF);
    pivot_motor_.config_IntegralZone(0, SwerveConstants.kPivotMotorkIZone);
    pivot_motor_.configMotionCruiseVelocity(SwerveConstants.motionCruiseVelocity);
    pivot_motor_.configMotionAcceleration(SwerveConstants.motionAcceleration);


    pivot_motor_.configOpenloopRamp(SwerveConstants.kLoopSeconds);
    pivot_motor_.configClosedloopRamp(SwerveConstants.kLoopSeconds);

    pivot_encoder_inverted = pivotEncoderInvert ? -1.0 : 1.0;

    SetDriveMotorInverted(driveMotorInvert);
    SetPivotMotorInverted(pivotMotorInvert);
    SetPivotEncoderOffset((int)pivot_encoder_inverted * pivotEncoderOffset);
    SetPivotEncoderPhase(pivotEncoderPhase);
    
    lastAngle = GetState().angle.getRadians();
  }

  public void SetDriveMotorInverted(boolean invert){
    drive_motor_inverted = invert ? -1.0 : 1.0;
  }

  public void SetPivotMotorInverted(boolean invert){
    pivot_motor_inverted = invert ? -1.0 : 1.0;
    // TODO(Shimushu): This doesn't work as I expected
    // pivot_motor_.SetInverted(invert);
    // TODO(Shimushu): I don't know this works accidentally or not
    pivot_motor_.setSensorPhase(!invert);
  }

  public void SetPivotEncoderOffset(int offset){
    offset_ = offset;
  }

  public void SetPivotEncoderPhase(boolean phase){
    pivot_motor_.setSensorPhase(phase);
  }

  public void SetMotorOutputDisabled(boolean disable){
    drive_motor_output_enabled = !disable;
    pivot_motor_output_enabled = !disable;
  }

  public void SetDriveMotorOutputDisabled(boolean disable){
    drive_motor_output_enabled = !disable;
  }

  public void SetPivotMotorOutputDisabled(boolean disable){
    pivot_motor_output_enabled = !disable;
  }

  public double GetDriveMotorOutput(){
    return drive_motor_.get();
  }

  public double GetPivotMotorOutput(){
    return pivot_motor_.getMotorOutputPercent();
  }

  public SwerveModuleState GetState(){
    return new SwerveModuleState(
      GetSpeed(),
      new Rotation2d(GetAngle())
    );
  }
  public SwerveModulePosition GetPosition()
  {
    return new SwerveModulePosition(
      drive_motor_.getRotorPosition().getValue() *drive_motor_inverted*Math.PI*Constants.SwerveConstants.kWheelDiameter/SwerveConstants.driveGearRatio, new Rotation2d(GetAngle()));
  }
  public void SetDesiredState(SwerveModuleState state,boolean isOpenLoop){
    double rawAngle = GetRawAngle();
    double angle = MathUtil.angleModulus(rawAngle);

    /**
     * Prevent robot from driving automatically.
     */
    if(Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeed * 0.05)){
      state.speedMetersPerSecond = 0;
    }

    SwerveModuleState optimalState = SwerveModuleState.optimize(
      state, new Rotation2d(angle));

    double driveOutput = optimalState.speedMetersPerSecond /
    SwerveConstants.kWheelDiameter  /Math.PI*SwerveConstants.driveGearRatio;

    //final double CTREDriveOutput = Conversions.MPSToFalcon(optimalState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);

    double optimalAngle = MathUtil.angleModulus(optimalState.angle.getRadians());
    optimalAngle += rawAngle - angle;
    if(Math.abs(rawAngle - optimalAngle) >= Math.PI * 0.5){
      optimalAngle += Math.copySign(Math.PI * 2, 
                      rawAngle - optimalAngle);
    }

    //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    double Angle = (Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeed * 0.01)) ? lastAngle : optimalAngle;
    
    double pivotOutput = Angle /
    SwerveConstants.kPivotEncoderReductionRatio *
    SwerveConstants.kPivotEncoderResolution *
    pivot_encoder_inverted + offset_;

    //double MaxFreeSpeed = Conversions.RPMToFalcon(6380, 1);

    //double RealFreeSpeed = 6380 / 60 * Constants.SwerveConstants.driveGearRatio * Constants.kWheelDiameter * Math.PI;
    //SmartDashboard.putNumber("FreeRealSpeed", RealFreeSpeed);

    double percentOutput = feedforward.calculate(optimalState.speedMetersPerSecond);

    SmartDashboard.putNumber("OptimalSpeed", optimalState.speedMetersPerSecond);
    SmartDashboard.putNumber("Debug/Drive/PercentOut", percentOutput);
    if(isOpenLoop){
      if (drive_motor_output_enabled) {
        //drive_motor_.set(ControlMode.PercentOutput,
        //    drive_motor_inverted * percentOutput);
          
        //drive_motor_.set(ControlMode.Velocity, //TODO: Test combining FeedForward and velocity closed loop
        //    drive_motor_inverted * driveOutput/*, DemandType.ArbitraryFeedForward,
         //   drive_motor_inverted * percentOutput*/);
        drive_motor_.setControl(m_request.withVelocity(driveOutput*drive_motor_inverted));
      }
    }
    else{
      if (drive_motor_output_enabled) {
        drive_motor_.setControl(m_request.withVelocity(driveOutput*drive_motor_inverted));
        /* Try combing PID Control with FeedForward */
        
       
      }
    }
    if (pivot_motor_output_enabled) {
      pivot_motor_.set(ControlMode.MotionMagic,
          pivot_motor_inverted * pivotOutput);
    }
    lastAngle = Angle;
  }
  public void SetDesiredState(BetterSwerveModuleState state,boolean isOpenLoop){
    double rawAngle = GetRawAngle();
    double angle = MathUtil.angleModulus(rawAngle);

    /**
     * Prevent robot from driving automatically.
     */
    if(Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeed * 0.05)){
      state.speedMetersPerSecond = 0;
    }

    SwerveModuleState optimalState = SwerveModuleState.optimize(
      new SwerveModuleState(state.speedMetersPerSecond,state.angle), new Rotation2d(angle));
    
    double driveOutput = optimalState.speedMetersPerSecond /Math.PI/Constants.SwerveConstants.kWheelDiameter*SwerveConstants.driveGearRatio;

    //final double CTREDriveOutput = Conversions.MPSToFalcon(optimalState.speedMetersPerSecond, Constants.SwerveConstants.wheelCircumference, Constants.SwerveConstants.driveGearRatio);

    double optimalAngle = MathUtil.angleModulus(optimalState.angle.getRadians());
    optimalAngle += rawAngle - angle;
    if(Math.abs(rawAngle - optimalAngle) >= Math.PI * 0.5){
      optimalAngle += Math.copySign(Math.PI * 2, 
                      rawAngle - optimalAngle);
    }

    //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    double Angle = (Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeed * 0.01)) ? lastAngle : optimalAngle;
    
    double pivotOutput = Angle /
    SwerveConstants.kPivotEncoderReductionRatio *
    SwerveConstants.kPivotEncoderResolution *
    pivot_encoder_inverted + offset_;

    //double MaxFreeSpeed = Conversions.RPMToFalcon(6380, 1);

    //double RealFreeSpeed = 6380 / 60 * Constants.SwerveConstants.driveGearRatio * Constants.kWheelDiameter * Math.PI;
    //SmartDashboard.putNumber("FreeRealSpeed", RealFreeSpeed);

    double percentOutput = feedforward.calculate(optimalState.speedMetersPerSecond);

    SmartDashboard.putNumber("OptimalSpeed", optimalState.speedMetersPerSecond);
    SmartDashboard.putNumber("Angle", pivotOutput);
    SmartDashboard.putNumber("Debug/Drive/PercentOut", percentOutput);

    if(isOpenLoop){
      if (drive_motor_output_enabled) {
        //drive_motor_.set(ControlMode.PercentOutput,
        //    drive_motor_inverted * percentOutput);
          
        //drive_motor_.set(ControlMode.Velocity, //TODO: Test combining FeedForward and velocity closed loop
        //    drive_motor_inverted * driveOutput/*, DemandType.ArbitraryFeedForward,
        //    drive_motor_inverted * percentOutput*/);
        drive_motor_.setControl(m_request.withVelocity(driveOutput*drive_motor_inverted));
      }
    }
    else{
      if (drive_motor_output_enabled) {
          drive_motor_.setControl(m_request.withVelocity(driveOutput*drive_motor_inverted));
        /* Try combing PID Control with FeedForward */
        
        //drive_motor_.set(ControlMode.Velocity, //TODO: Test combining FeedForward and velocity closed loop
        //    drive_motor_inverted * driveOutput, DemandType.ArbitraryFeedForward,
        //    drive_motor_inverted * percentOutput);
      }
    }
    if (pivot_motor_output_enabled) {
      pivot_motor_.set(ControlMode.MotionMagic,//TODO
          pivot_motor_inverted * pivotOutput);
    }
    lastAngle = Angle;
  }
  
  public double GetDriveMotorVoltage(){
    //The unit is volt
    return drive_motor_.getMotorVoltage().getValue();
  }

  public double GetTurnMotorVoltage(){
    //The unit is volt
    return pivot_motor_.getBusVoltage();
  }

  public double GetDriveMotorOutputVoltage(){
    //The unit is volt
    return drive_motor_.getMotorVoltage().getValue();
  }

  public double GetTurnMotorOutputVoltage(){
    //The unit is volt
    return pivot_motor_.getMotorOutputVoltage();
  }

  public double GetDriveMotorCurrent(){
    //The unit is ampere
    return drive_motor_.getSupplyCurrent().getValue();
  }

  public double GetTurnMotorCurrent(){
    //The unit is ampere
    return pivot_motor_.getSupplyCurrent();
  }

  public double GetDriveMotorPower(){
    //The unit is watt
    return GetDriveMotorVoltage() * GetDriveMotorCurrent();
  }

  public double GetTurnMotorPower(){
    //The unit is watt
    return GetTurnMotorVoltage() * GetTurnMotorCurrent();
  }

  public double GetTotalCurrent(){
    //The unit is ampere
    return GetDriveMotorCurrent() + GetTurnMotorCurrent();
  }

  public double GetTotalPower(){
    //The unit is watt
    return GetDriveMotorPower() + GetTurnMotorPower();
  }

  public double GetDriveMotorTemperature(){
    //The unit is celsius 
    return drive_motor_.getDeviceTemp().getValue();
  }

  public double GetPivotMotorTemperature(){
    //The unit is celsius 
    return pivot_motor_.getTemperature();
  }

  public double GetAngle(){
    /*The unit is radian*/
    return MathUtil.angleModulus(      
    (pivot_motor_.getSelectedSensorPosition() - offset_) /
    SwerveConstants.kPivotEncoderResolution *
    SwerveConstants.kPivotEncoderReductionRatio *
    pivot_encoder_inverted);
  }

  public double GetRawAngle(){
    /*The unit is radian*/
    return 
        (pivot_motor_.getSelectedSensorPosition() - offset_) /
        SwerveConstants.kPivotEncoderResolution *
        SwerveConstants.kPivotEncoderReductionRatio *
        pivot_encoder_inverted;
  }

  public double GetSpeed(){
    /*The unit is meters_per_second */
    return
        drive_motor_.getVelocity().getValue() * drive_motor_inverted *Math.PI*Constants.SwerveConstants.kWheelDiameter /SwerveConstants.driveGearRatio;
    //return Conversions.falconToMPS(drive_motor_.getSelectedSensorVelocity(), Constants.kWheelDiameter * Math.PI,
    //    Constants.kDriveEncoderReductionRatioTest);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
