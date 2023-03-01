//Swerve Constants
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ArmConstants{
        public static final int PIVOTMOTORID = 7;
        public static final double ARMDEADBAND = 0.4;
        public static final int CLAWSOLFWD = 0;
        public static final int CLAWSOLREV = 2;
        public static final double NeoCPR = 42;
       
        public static final double kPivotMotorGearRatio = (1/29.4)/5; //
        
        public static final double kPivotEncoderRot2Rad = kPivotMotorGearRatio * 2 * Math.PI;
        public static final double kPivotEncoderRPM2RadPerSec = kPivotEncoderRot2Rad / 60;
       
    }

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); 
        public static final double kDriveMotorGearRatio = 240.0/1600.0; 
        //public static final double kTurningMotorGearRatio = 48.0/40.0; 
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kPTurning = 0.5;
        public static final double kCANCoderCPR = 4096;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(22); 
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(22);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 10; 
        public static final int kBackLeftDriveMotorPort = 23; 
        public static final int kFrontRightDriveMotorPort = 50;
        public static final int kBackRightDriveMotorPort = 8;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 51;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackRightTurningMotorPort = 17;

        public static final int kFrontLeftEncChanA = 2; 
        public static final int kFrontLeftEncChanB = 3;
        public static final int kFrontRightEncChanA = 4;
        public static final int kFrontRightEncChanB = 5;
        public static final int kBackLeftEncChanA = 0;
        public static final int kBackLeftEncChanB = 1;
        public static final int kBackRightEncChanA = 6;
        public static final int kBackRightEncChanB = 7;

        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3; 
        public static final int kBackLeftDriveAbsoluteEncoderPort = 4; 
        public static final int kFrontRightDriveAbsoluteEncoderPort = 2; 
        public static final int kBackRightDriveAbsoluteEncoderPort = 1;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
        //left in, right out
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 1.295;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.664;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.139;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 5.03;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 1;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 0.5 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }
}
