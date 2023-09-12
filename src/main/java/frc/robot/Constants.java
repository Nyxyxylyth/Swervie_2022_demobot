// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


=======
>>>>>>> 73780bf7f470dbb7a75c6328383eb80c7dd4f9ff
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
<<<<<<< HEAD

    public static final int INTAKE_ARM_MOTOR = 7;
    public static final int INTAKE_MOTOR = 8;
=======
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_INCHES = 20.5;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_INCHES = 20.5;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 15;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 14;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(175.9); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 12;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 10;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 11;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(143.7); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 18;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 16;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 17;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(201.7); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 21;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 19;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 20;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(165.0); // FIXME Measure and set back right steer offset

    public static final int INTAKE_ARM_MOTOR = 7;
    public static final int INTAKE_MOTOR = 8;

    public static final int CLIMB_6_MOTOR = 30;
    public static final int CLIMB_10_MOTOR = 31;
    public static final int CLIMB_15_MOTOR = 32;

    public static final int CLIMB_6_ENCODER = 33;
    public static final int CLIMB_10_ENCODER = 34;
    public static final int CLIMB_15_ENCODER = 35;

    
>>>>>>> 73780bf7f470dbb7a75c6328383eb80c7dd4f9ff
    public static final int KICKER_MOTOR = 41;
    public static final int KICKER_ENCODER = 42;

    public static final int YEET_MOTOR = 40;
    public static final int YEET_SPEED_HIGH = 2500;
<<<<<<< HEAD
    public static final int YEET_SPEED_LOW = 1600;
    public static final double YEET_SPEED_TOLERANCE = 0.03; //0.015;
=======
    public static final int YEET_SPEED_LOW = 1300;
    public static final double YEET_SPEED_TOLERANCE = 0.015;
>>>>>>> 73780bf7f470dbb7a75c6328383eb80c7dd4f9ff

    public static final int TALON_TIMEOUT_MS = 5000;

    public static final double INTAKE_ARM_MOTOR_KF = 0.07;
    public static final double INTAKE_ARM_MOTOR_KP = 1.3;
    public static final double INTAKE_ARM_MOTOR_KI = 0.000;
    public static final double INTAKE_ARM_MOTOR_KD = 0.00;
    public static final int INTAKE_ARM_POSITION_IN_FULL = 0;
    public static final int INTAKE_ARM_POSITION_IN = 220;
    public static final int INTAKE_ARM_POSITION_OUT = 650;

<<<<<<< HEAD
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = /* FIXME 1 */ 0.5 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.2;
    }
    
    public static final class DriveConstants {
    
        public static final double kTrackWidth = Units.inchesToMeters(20.5);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(20.5);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
    
        // FIXME: patch these motor IDs up to match the Swervie 2022 configuration
        public static final int kFrontLeftDriveMotorPort = 15;
        public static final int kFrontRightDriveMotorPort = 12;
        public static final int kBackLeftDriveMotorPort = 18;
        public static final int kBackRightDriveMotorPort = 21;
    
        public static final int kFrontLeftTurningMotorPort = 13;
        public static final int kFrontRightTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 16;
        public static final int kBackRightTurningMotorPort = 19;
    
        // CANCoder IDs
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 14;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 17;
        public static final int kBackRightDriveAbsoluteEncoderPort = 20;

        public static final double kFrontLeftDriveAbsoluteEncoderOffset  = -4.4;
        public static final double kFrontRightDriveAbsoluteEncoderOffset = -36.65;
        public static final double kBackLeftDriveAbsoluteEncoderOffset   = 29.04;
        public static final double kBackRightDriveAbsoluteEncoderOffset  = -15.117;

        public static final int gyroPort = 60;
    
        public static final boolean kFrontLeftTurningEncoderReversed = false;
        public static final boolean kFrontRightTurningEncoderReversed = false;
        public static final boolean kBackLeftTurningEncoderReversed = false;
        public static final boolean kBackRightTurningEncoderReversed = false;
    
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;
    
    
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
    
    
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 6;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 6;
    
        public static final double CreepLoading = -0.37;
        public static final double CreepBalance = -0.30;
        public static final double CreepBalanceMobility = 0.35;
        public static final double CreepBalanceMobilityBackup = -0.2;
    
    }
    
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.15;
    }

}

=======
    public static final double CLIMB6_MOTOR_KF = 0.00;
    public static final double CLIMB6_MOTOR_KP = 5.4;
    public static final double CLIMB6_MOTOR_KI = 0.0150;
    public static final double CLIMB6_MOTOR_KD = 0.0135;
    public static final double CLIMB6_MOTOR_CRUISE = 4000;
    public static final double CLIMB6_MOTOR_ACCELERATION = 8000;
    public static final int CLIMB6_POSITION_IN = 0;
}
>>>>>>> 73780bf7f470dbb7a75c6328383eb80c7dd4f9ff
