package frc.robot;

import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Commands.ClawDefaultCommand;
import frc.robot.Commands.SwerveJoystickCMD;
import frc.robot.Subsystems.ClawSubsystem;
import frc.robot.Subsystems.SwerveChassisSubsystem;

public class RobotContainer {

    private final SwerveChassisSubsystem swerveSubsystem = new SwerveChassisSubsystem();
    public static final ClawSubsystem CLAW_SUBSYSTEM = new ClawSubsystem();
 
  
    private final OperatorInput OI = new OperatorInput();

    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCMD(
                swerveSubsystem,
                () -> OI.getLeftYMovement(),
                () -> OI.getLeftXMovement(),
                () -> OI.getRightXMovement(),
                () -> OI.getBackReleasedMovement(),
                () -> OI.setWheelsZero(),
                () -> OI.configureButtonBindings()
        ));
        CLAW_SUBSYSTEM.setDefaultCommand(new ClawDefaultCommand(CLAW_SUBSYSTEM, ()-> OI.getBPressedArm(), ()-> OI.getRightYArm(), ()->OI.getXPistOn(), ()->OI.getYPistOff()));

    }



    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(2, 0)),
                new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem
        );
        

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
