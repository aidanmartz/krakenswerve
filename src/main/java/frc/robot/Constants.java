package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class VortexMotorConstants {
        public static final double kFreeSpeedRpm = 0.5;
    }

    public static final class CANConstants {
        public static final int pivotLeftId = 20;
        public static final int pivotRightId = 21;
        public static final int elevatorLeftId = 30;
        public static final int elevatorRightId = 31;

    }

    public static final class elevatorConstants {
        /* IDS for elevator motors*/
        public static final int elevatorLeftId = 30;
        public static final int elevatorRightId = 31;

        /* CANBus */
        public static final String canBus = "rio";

        /* Elevator dimensions */
        public static final double thickness = 2.0; // Thickness of the elevator (only for Mechanism2d visualization)
        public static final double setback = 9.5; // Distance from front edge of robot (only for Mechanism2d visualization)
        public static final double bellyHeight = 0.755; // Height of the top surface of the belly pan from the ground
        public static final double baseHeight = 12.0 + bellyHeight; // Height of elevator in inches when it is at zero position
        public static final double maxHeight = 72.0 + bellyHeight; // Height that elevator should never exceed
        public static final double endEffectorHeight = 6.0; // Height of end effector "target" above elevator height
        public static final double rotPerInch = 0.704;

    }

    public static final class Swerve {

        public static final int pigeonID = 10;
        public static final String CanBus = "Drivetrain"; //TODO: if on canivore, change this to CANivore name or serial number
        public static final boolean focEnabled = true; // TODO: This must be tuned to specific robot
        public static final boolean isOnCANivore = true;

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSTalonFXSwerveConstants.WCP.SwerveXFlipped
                        .KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X3_10);

        /* Drivetrain Constants */
        /* Center to Center distance of left and right modules in meters. */
        public static final double trackWidth = Units.inchesToMeters(25); // TODO: This must be tuned to specific robot
        /* Center to Center distance of front and rear module wheels in meters. */
        public static final double wheelBase = Units.inchesToMeters(25); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 30;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = false;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = false;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        /*
         * These are theorectial values to start with, tune after
         * Kraken FOC-DIS (L1.0): ft/s = 12.9 | m/s = 3.93192
         * Kraken FOC-ENB (L1.0): ft/s = 12.4 | m/s = 3.77952
         * Kraken FOC-DIS (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC-ENB (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC-DIS (L2.0): ft/s = 15.5 | m/s = 4.7244
         * Kraken FOC-ENB (L2.0): ft/s = 15.0 | m/s = 4.572
         * Kraken FOC-DIS (L2.5): ft/s = 17.7 | m/s = 5.39496
         * Kraken FOC-ENB (L2.5): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC-DIS (L3.0): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC-ENB (L3.0): ft/s = 16.5 | m/s = 5.0292
         * Kraken FOC-DIS (L3.5): ft/s = 19.5 | m/s = 5.9436
         * Kraken FOC-ENB (L3.5): ft/s = 18.9 | m/s = 5.76072
         * Kraken FOC-DIS (L4.0): ft/s = 20.4 | m/s = 6.21792
         * Kraken FOC-ENB (L4.0): ft/s = 19.7 | m/s = 6.00456
         */
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-135.0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-135.0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.0);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionConstants {
        public static final double kCameraRangeScaler = 0.5;
        public static final double kCameraAimScaler = 0.033;
        public static final double kCameraAmpTargetArea = 1.1;
        public static final double kCameraSpeakerTargetArea = 0.71;
        public static final double kCamHeight = 0.41;
        public static final double kTagHeight = 1.27;
        public static final double kCamPitch = Math.PI / 4; // ~45 degrees (pi/4 rad)
    }

}
