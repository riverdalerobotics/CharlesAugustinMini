// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Subsystems.ClawSubsystem;

public class ClawDefaultCommand extends CommandBase {
  /** Creates a new ClawDefaultCommand. */
  private final double limitAngle1Rad = 10 * (Math.PI/180);
  private final double limitAngle2Rad = 100 * (Math.PI/180);
  private final ClawSubsystem CLAW_SUBSYSTEM;
  private final Supplier<Boolean> toggleClawFunction;
  private final Supplier<Double> operatePivotFunction;
  private final Supplier<Boolean> pO;
  private final Supplier<Boolean> pOf;
  public ClawDefaultCommand(ClawSubsystem clawSubsystem, Supplier<Boolean> toggleClawFunction, Supplier<Double> operatePivotFunction, Supplier<Boolean> pistOn, Supplier<Boolean> pistOff) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.CLAW_SUBSYSTEM = clawSubsystem;
    this.toggleClawFunction = toggleClawFunction;
    this.pO = pistOn;
    this.pOf = pistOff;
    this.operatePivotFunction = operatePivotFunction;
    addRequirements(this.CLAW_SUBSYSTEM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(this.pO.get()){
      System.out.println("I have been called");
      CLAW_SUBSYSTEM.setOn();
    }
    if(this.pOf.get()){
      System.out.println("I have been called");
      CLAW_SUBSYSTEM.setOff();
    }
    if(toggleClawFunction.get()) { //Robot.OI.getBPressedArm()
      
      CLAW_SUBSYSTEM.toggleClaw();
    }
 
    SmartDashboard.putNumber("Claw", CLAW_SUBSYSTEM.getRotation());
    double clawRotation = operatePivotFunction.get();
    if ((CLAW_SUBSYSTEM.getRotation() <= limitAngle1Rad && clawRotation > 0) || (CLAW_SUBSYSTEM.getRotation() >= limitAngle2Rad && clawRotation < 0)) {
      CLAW_SUBSYSTEM.stop();
    } 
    else if(Math.abs(clawRotation) > ArmConstants.ARMDEADBAND){
      if (clawRotation > 0) { //Robot.OI.getRightYArm()
        CLAW_SUBSYSTEM.rotateArm(-0.3);
      } else if (clawRotation < 0) {
        CLAW_SUBSYSTEM.rotateArm(0.3);
      }
    }
    else{
      CLAW_SUBSYSTEM.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
