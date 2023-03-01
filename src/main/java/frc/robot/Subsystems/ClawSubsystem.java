// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ArmConstants;

public class ClawSubsystem extends SubsystemBase {
  /** Creates a new ClawSubsystem. */
  CANSparkMax pivotMotor;

  DoubleSolenoid clawPistons;
  Value clawState;
  RelativeEncoder pivotEnc;
  public ClawSubsystem() {
    pivotMotor = new CANSparkMax(ArmConstants.PIVOTMOTORID, MotorType.kBrushless);
    clawPistons = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ArmConstants.CLAWSOLREV, ArmConstants.CLAWSOLFWD);
    clawState = Value.kOff;
    pivotEnc = pivotMotor.getEncoder();
    pivotEnc.setPositionConversionFactor(ArmConstants.kPivotEncoderRot2Rad);
    pivotEnc.setVelocityConversionFactor(ArmConstants.kPivotEncoderRPM2RadPerSec);
  }

  public void toggleClaw() {
    clawState = (clawState == Value.kReverse || clawState == Value.kOff) ? Value.kForward : Value.kReverse;
    clawPistons.set(clawState);
  }
  public void setOn(){
    clawPistons.set(Value.kForward);
  }
  public void setOff(){
    clawPistons.set(Value.kReverse);
  }
  public void rotateArm(double power) {
    pivotMotor.set(power);
  }
  public double getRotation() {
    return pivotEnc.getPosition();
  }
  public void stop(){
    pivotMotor.set(0);
  }
  public void setPistonOff(){
    clawPistons.set(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
