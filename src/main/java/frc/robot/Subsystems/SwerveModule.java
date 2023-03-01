// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
// This code is adapted from Sean Sun's FRC 0 to Autonomous Swerve Drive code
// and 4201 Vitruvian Bots https://github.com/4201VitruvianBots/2023SwerveSim/tree/main/2023RevSwerve/src/main/java/frc/robot
// all we changed was removing deprecated stuff and the constants, and combined the Vitruvian Bots code with Sean Sun's code to update it to 2023
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends SubsystemBase {
  private final CANSparkMax driveMotor;
  private final WPI_TalonSRX turningMotor;

  private final RelativeEncoder driveEncoder;
  

  //Moves the turning motor - L
  private final PIDController turningPidController;

  //Connected to analog ports on roborio -L
  //Abs encoder might differ from acc angle in first boot
  private final WPI_CANCoder absoluteTurnEncoder;
  private final boolean absoluteEncoderReversed;
  //This stores the difference between the initial abs encoder value and the wheel direction
  //Used later in the code to compensate for difference
  private final double absoluteEncoderOffsetRad;
  
  /** Creates a new SwerveModule. */ 
  //Asks for the port #'s of motors, sensors, and whether or not they are reversed
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
      int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
    
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteTurnEncoder = new WPI_CANCoder(absoluteEncoderId);
    
    CANCoderConfiguration config = new CANCoderConfiguration();
    // set units of the CANCoder to radians, with velocity being radians per second
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    absoluteTurnEncoder.configAllSettings(config);
    
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new WPI_TalonSRX(turningMotorId);
  
    driveMotor.setInverted(driveMotorReversed);
    turningMotor.setInverted(turningMotorReversed);

    driveEncoder = driveMotor.getEncoder();
    

    //EXPLAIN TO LIAM AND AHNAF PLEASE - L
    driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
    
    
    turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
    //Tells the PID controller that our system is a circle and that 180 and -180 are beside eachother
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    
    resetEncoders();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public double getTurningPosition() {
    return absoluteTurnEncoder.getPosition();
    
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return absoluteTurnEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    //gives the percentage of a full rotation by dividing abs encoder voltage by supplied voltage
    double angle = absoluteTurnEncoder.getAbsolutePosition();
    //Adjusts radians by radian offset
    angle -= absoluteEncoderOffsetRad;
    //Reverses the angle if the abs encoder has been reversed
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }
  //Resets drive enc, and changes turning encoder to the value of the abs encoder
  //Called during boot
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    absoluteTurnEncoder.setPosition(getAbsoluteEncoderRad());
  }
  public void setModuleZero() {
    SwerveModuleState zero = new SwerveModuleState(0, new Rotation2d(0));
    zero = SwerveModuleState.optimize(zero, getState().angle);
    
    driveMotor.set(zero.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //PID controller calculates what to set the turning motor to
    //I don't understand how it does this
    turningMotor.set(turningPidController.calculate(getTurningPosition(), zero.angle.getRadians()));
 
    
  }
  //WPI libraries request encoder values in the format of "swerve module state"
  //This method returns the encoder values in these formats
  public SwerveModuleState getState() {
    
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  /*public void zeroModule(){
    turningMotor.set(turningPidController.calculate(getTurningPosition(), 0));
    driveMotor.stopMotor();
  }*/
  //Takes in the desired state of the swerve module
  //state contains desired angle and speed of the module
  public void setDesiredState (SwerveModuleState state) {
    //If the robot receives no input, the method will automatically reset the wheels to 0 degrees
    //This checks if no substantial speed has been input, in which case it will end the method
    //without resetting the wheels
    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    //"state" is desired angle and "getState().angle" is the current angle
    //if the desired angle has a +90 degree difference from the current angle
    //it will turn the other way and reverse the motor
    state = SwerveModuleState.optimize(state, getState().angle);

    //Sets motor value from 1 to -1, depending on max
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //PID controller calculates what to set the turning motor to
    //I don't understand how it does this
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteTurnEncoder.getDeviceID() + "] state", state.toString());
  }
  
  public SwerveModulePosition getSMPosition(){
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getHeadingDegrees()));
  }
  public double getHeadingDegrees() {
    return getTurningPosition();
  }
  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}  