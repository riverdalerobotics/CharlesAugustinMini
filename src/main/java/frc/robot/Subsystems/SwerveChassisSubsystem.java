package frc.robot.Subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
// This code is adapted from Sean Sun's FRC 0 to Autonomous Swerve Drive code
// and 4201 Vitruvian Bots https://github.com/4201VitruvianBots/2023SwerveSim/tree/main/2023RevSwerve/src/main/java/frc/robot
// all we changed was removing deprecated stuff and the constants, and combined the Vitruvian Bots code with Sean Sun's code to update it to 2023
public class SwerveChassisSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed
            
            
    );

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed
    );

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed

            
    );
    
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            getRotation2d(), getSwerveModulePositions());
    
    public SwerveChassisSubsystem() {
        resetModules();
    
        new Thread(() -> {
            try {
                //Delays 1 second before resetting heading
                //Allows gyro to recalibrate first on boot
                //Is on a seperate thread so the rest of the code is unaffected by sleep(1000)
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();

    }
    //Resets gyro's heading on boot to recalibrate what it considers forwards
    public void zeroHeading() {
        gyro.reset();
    }
    //PITCH AND ROLL ARE REVERSED (ROLL is currently turning on side-to-side axis PITCH is currently turning on front-to-back axis)
    public double getYawRad() {
        return gyro.getYaw() * (Math.PI / 180);
    }
    public double getPitchRad() {
        return gyro.getPitch() * (Math.PI / 180);
    }
    public double getRollRad() {
        return gyro.getRoll() * (Math.PI / 180);  
    }

    //Gyro's value is continuous, it can go past 360
    //This function clamps it between 180 and -180 degrees to make it easier to work with
    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360);
    }
    
    public void resetModules() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
    public void straightenModules() {
        frontLeft.setModuleZero();
        frontRight.setModuleZero();
        backLeft.setModuleZero();
        backRight.setModuleZero();
        
    }

    //Converts gyro heading into Rotation2d so the robot can use it
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }
    public SwerveModulePosition[] getSwerveModulePositions(){
        return new SwerveModulePosition[]{
            frontLeft.getSMPosition(),
            frontRight.getSMPosition(),
            backLeft.getSMPosition(),
            backRight.getSMPosition()

        };
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }

    @Override
    public void periodic() {
        odometer.update(getRotation2d(), getSwerveModulePositions());
        //Programming info
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        SmartDashboard.putNumber("drive enc pos FR (m)", frontRight.getDrivePosition());
        SmartDashboard.putNumber("turn enc pos FR (rad)", frontRight.getTurningPosition());
        SmartDashboard.putNumber("abs enc pos FR (rad)", frontRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("drive enc FR velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("angular velocity FR", frontRight.getTurningVelocity());
  
        SmartDashboard.putNumber("drive enc pos FL (m)", frontLeft.getDrivePosition());
        SmartDashboard.putNumber("turn enc pos FL (rad)", frontLeft.getTurningPosition());
        SmartDashboard.putNumber("abs enc pos FL (rad)", frontLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("drive enc FL velocity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("angular velocity FL", frontLeft.getTurningVelocity());  
        
        SmartDashboard.putNumber("drive enc pos BR (m)", backRight.getDrivePosition());
        SmartDashboard.putNumber("turn enc pos BR (rad)", backRight.getTurningPosition());
        SmartDashboard.putNumber("abs enc pos BR (rad)", backRight.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("drive enc BR velocity", backRight.getDriveVelocity());
        SmartDashboard.putNumber("angular velocity BR", backRight.getTurningVelocity()); 

        SmartDashboard.putNumber("drive enc pos BL (m)", backLeft.getDrivePosition());
        SmartDashboard.putNumber("turn enc pos BL (rad)", backLeft.getTurningPosition());
        SmartDashboard.putNumber("abs enc pos BL (rad)", backLeft.getAbsoluteEncoderRad());
        SmartDashboard.putNumber("drive enc BL velocity", backLeft.getDriveVelocity());
        SmartDashboard.putNumber("angular velocity BL", backLeft.getTurningVelocity()); 


        //Driver info
        SmartDashboard.putNumber("Absolute Turn Angle (FR)", frontRight.getAbsoluteEncoderRad() * (180 / Math.PI));
        SmartDashboard.putNumber("Velocity (FR)", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("Angular Velocity (FR)", frontRight.getTurningVelocity() * (180 / Math.PI));

        //Gyro info
        SmartDashboard.putNumber("Pitch(degrees)", gyro.getPitch());
        SmartDashboard.putNumber("Roll(degrees)", gyro.getRoll());
        SmartDashboard.putNumber("Yaw(degrees)", gyro.getYaw());
        SmartDashboard.putNumber("Pitch(radians)", getPitchRad());
        SmartDashboard.putNumber("Roll(radians)", getRollRad());
        SmartDashboard.putNumber("Yaw(radians)", getYawRad());
        
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    //Takes in an array of the desired states for each swerve module
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);  
        backRight.setDesiredState(desiredStates[3]);
        
    }
    
}
