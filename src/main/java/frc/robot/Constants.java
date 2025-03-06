package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import static edu.wpi.first.units.Units.*;
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
        public static final int intakeId = 40;
        public static final String canBus = "rio";
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

    public static final class Localization {
        public static final double fieldWidth = FlippingUtil.fieldSizeY;
        public static final double fieldLength = FlippingUtil.fieldSizeX;

        public static final double reefElevatorZoneRadius = Units.inchesToMeters(80.0); // TODO Revisit
        public static final double autoUpDistance = Units.inchesToMeters(44.0);
        public static final double wingLength = Units.inchesToMeters(280);

        public static final double robotFrameLength = Units.inchesToMeters(30);
        public static final double bumperWidth = Units.inchesToMeters(3);
        public static final double reefStandoff = Units.inchesToMeters(1.0);
        public static final double reefOffset = robotFrameLength / 2.0 + bumperWidth + reefStandoff;
        public static final double reefExtraOffset = Units.inchesToMeters(16.0); // reef wood to outside of tape line
        public static final double bonusStandoff = Units.inchesToMeters(4.0);

        // Locations from the Blue Alliance perspective
        // 144-14+(93.5/2)
        public static final Translation2d reefCenter = new Translation2d(Units.inchesToMeters(176.75), fieldWidth / 2.0);
        // center - distance from wall to face 176.75 - 144 = 36.75
        public static final double reefToFaceDistance = reefCenter.getX() - Units.inchesToMeters(144.0);
        public static final double branchSeparation = Units.inchesToMeters(12.0 + 15.0 / 16.0);

        // Offset to the reef face, not at the branches, but on the faces directly in front
        public static final Translation2d centerOffset = new Translation2d(reefToFaceDistance + reefOffset, 0.0);
        private static final Translation2d leftOffset = new Translation2d(reefToFaceDistance + reefOffset, -branchSeparation / 2.0);
        private static final Translation2d rightOffset = new Translation2d(reefToFaceDistance + reefOffset, branchSeparation / 2.0);
        private static final Translation2d extraOffset = new Translation2d(reefExtraOffset, 0.0);
        private static final Translation2d centerApproachOffset = centerOffset.plus(extraOffset);
        private static final Translation2d leftApproachOffset = leftOffset.plus(extraOffset);
        private static final Translation2d rightApproachOffset = rightOffset.plus(extraOffset);
        private static final Translation2d bonusOffset = new Translation2d(bonusStandoff, 0.0);
        private static final Translation2d leftBonusOffset = leftOffset.plus(bonusOffset);
        private static final Translation2d rightBonusOffset = rightOffset.plus(bonusOffset);

        // Dont climb the reef
        public static final double elevatorNoDownDistance = reefToFaceDistance + reefOffset + Units.inchesToMeters(12.0);

        public static enum ReefFace {
            AB(-180, true),
            CD(-120, false),
            EF(-60, true),
            GH(0, false),
            IJ(60, true),
            KL(120, false);

            ReefFace(double directionDegrees, boolean algaeHigh) {
                directionFromCenter = Rotation2d.fromDegrees(directionDegrees);
                alignMiddle = new Pose2d(reefCenter.plus(centerOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                alignLeft = new Pose2d(reefCenter.plus(leftOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                alignRight = new Pose2d(reefCenter.plus(rightOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                approachMiddle = new Pose2d(reefCenter.plus(centerApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                approachLeft = new Pose2d(reefCenter.plus(leftApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                approachRight = new Pose2d(reefCenter.plus(rightApproachOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                alignBonusLeft = new Pose2d(reefCenter.plus(leftBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                alignBonusRight = new Pose2d(reefCenter.plus(rightBonusOffset).rotateAround(reefCenter, directionFromCenter), directionFromCenter.plus(Rotation2d.k180deg));
                this.algaeHigh = algaeHigh;
            }

            public final Rotation2d directionFromCenter;
            public final Pose2d alignLeft, alignMiddle, alignRight;
            public final Pose2d approachLeft, approachMiddle, approachRight;
            public final Pose2d alignBonusLeft, alignBonusRight;
            public final boolean algaeHigh;       
        }
    }

    public static final class Swerve {

        public static final int pigeonID = 10;
        public static final String CanBus = "Drivetrain"; 
        public static final boolean focEnabled = true; 
        public static final boolean isOnCANivore = true;

        public static final COTSTalonFXSwerveConstants chosenModule = 
                COTSTalonFXSwerveConstants.WCP.SwerveXFlipped
                        .KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X3_10);

        /* Drivetrain Constants */
        /* Center to Center distance of left and right modules in meters. */
        public static final double trackWidth = Units.inchesToMeters(25); 

        /* Center to Center distance of front and rear module wheels in meters. */
        public static final double wheelBase = Units.inchesToMeters(25); 
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
        public static final double maxAngularVelocity = 10.0; 
        //public static final double maxAngularVelocity = Math.hypot(wheelBase, trackWidth) / 2;

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

      public static final class PathPlanner {
        public static final RobotConfig robotConfig = new RobotConfig(
            Mass.ofRelativeUnits(130, Pounds),
            MomentOfInertia.ofRelativeUnits(7.0, KilogramSquareMeters),
            new ModuleConfig(
                Swerve.wheelCircumference / (Math.PI * 2.0),
                Swerve.maxSpeed * 0.95, // Leave a little headroom for inefficiencies
                1.916, // 3847 Spectrum Vex GripLock v2 CoF
                DCMotor.getKrakenX60Foc(1),
                Swerve.chosenModule.driveGearRatio,
                Swerve.driveCurrentLimit,
                1),
            new Translation2d(Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
            new Translation2d(Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0));
    }

}
