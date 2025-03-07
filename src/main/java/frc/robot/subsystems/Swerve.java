package frc.robot.subsystems;

import frc.lib.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.Localization.ReefFace;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Boolean ll;
    public boolean doRejectUpdate = false;

    // private final ReentrantLock swerveModLock = new ReentrantLock();
    //private final Notifier odoNotifier;
    private final SwerveDrivePoseEstimator m_poseEstimator;
    private final Field2d field;
    private final Pigeon2 gyro;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.CanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        ll = false;

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Let the field be visible in elastic
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        m_poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))
        );

        // Add support for Canivore and use 250 Hz
        // boolean isFastOdo = Constants.Swerve.isOnCANivore;
        //odoNotifier = new Notifier(this::updateSwerveOdom);
        //odoNotifier.startPeriodic(isFastOdo ? 1.0 / 250.0 : 1.0 / 50.0);       
       
        try {
            //RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelativeAuto(speeds), // Method that will drive the robot given
                                                                              // ROBOT RELATIVE ChassisSpeeds. Also
                                                                              // optionally outputs individual module
                                                                              // feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller
                                                    // for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)),
                    //config,
                    Constants.PathPlanner.robotConfig,
                    Robot::isRed,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }

        SmartDashboard.putData("Gyro Heading", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("Gyro");
                builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
            }
        });

    }

    private double limelightRotation() {
        double targetAngularVel = LimelightHelpers.getTXNC("limelight") * Constants.VisionConstants.kCameraAimScaler;
        //double targetAngleVelocity = LimelightHelpers.getTargetPose_CameraSpace2D("limelight")[5];
        targetAngularVel *= -1;
        SmartDashboard.putNumber("limelight/Angular Velocity", targetAngularVel);
        return targetAngularVel;
    }

    private double limelightX() {
        double targetForwardSpeed = Constants.VisionConstants.kCameraRangeScaler / LimelightHelpers.getTA("limelight");
        SmartDashboard.putNumber("limelight/Forward Speed", targetForwardSpeed);
        return targetForwardSpeed;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isLimelight) {
        SwerveModuleState[] swerveModuleStates;

        if (isLimelight) {
            ll = true;
            swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    new ChassisSpeeds(
                            limelightX(),
                            translation.getY(),
                            limelightRotation()));
        } else {
            ll = false;
            swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                    fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                            translation.getX(),
                            translation.getY(),
                            rotation,
                            getHeading())
                            : new ChassisSpeeds(
                                    translation.getX(),
                                    translation.getY(),
                                    rotation));
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);   
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeAuto(ChassisSpeeds desirChassisSpeeds) {
        driveRobotRelative(desirChassisSpeeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics
                .toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void alignStraight() {
        SwerveModuleState aligned = new SwerveModuleState(0.0, new Rotation2d());

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(aligned, false);
        }
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public double getYawRate() {
        return gyro.getAngularVelocityZWorld().getValueAsDouble();
    }

    public void setHeading(Rotation2d heading) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public static Translation2d flipIfRed(Translation2d position) {
        return Robot.isRed() ? FlippingUtil.flipFieldPosition(position) : position;
    }

    public static Pose2d flipIfRed(Pose2d pose) {
        return Robot.isRed() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public static Rotation2d flipIfRed(Rotation2d rotation) {
        return Robot.isRed() ? FlippingUtil.flipFieldRotation(rotation) : rotation;
    }

    public static Rotation2d reefBearing(Translation2d position) {
        Translation2d reefCenter = flipIfRed(Constants.Localization.reefCenter);
        Translation2d relativePosition = reefCenter.minus(position);

        return relativePosition.getAngle();
    }

    public static ReefFace nearestFace(Translation2d position) {
        Rotation2d reefBearing = flipIfRed(reefBearing(position));
        double bearingAngle = MathUtil.inputModulus(reefBearing.getDegrees(), -180, 180);

        if (bearingAngle > 150 || bearingAngle < -150) {
            return ReefFace.GH;
        } else if (bearingAngle > 90) {
            return ReefFace.EF;
        } else if (bearingAngle > 30) {
            return ReefFace.CD;
        } else if (bearingAngle > -30) {
            return ReefFace.AB;
        } else if (bearingAngle > -90) {
            return ReefFace.KL;
        } else { // bearingAngle > -150
            return ReefFace.IJ;
        }
    }

    public void zeroHeading() {
        if (Robot.isRed()) {
            setHeading(new Rotation2d(Math.PI));
        }
        else {
            setHeading(new Rotation2d());
        }
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public Command resetModulesToAbsolute() {
        return Commands.runOnce(
                () -> {
                    for (SwerveModule mod : mSwerveMods) {
                        mod.resetToAbsolute();
                    }
                },
                this);
    }



    @Override
    public void periodic() {
        SmartDashboard.putBoolean("limelight/use limelight", ll);
        SmartDashboard.putNumber("limelight/TX", LimelightHelpers.getTXNC("limelight"));
        SmartDashboard.putNumber("limelight/TA", LimelightHelpers.getTA("limelight"));
        SmartDashboard.putNumber("limelight/Target ID", LimelightHelpers.getFiducialID("limelight"));
        
        SmartDashboard.putData("Gyro Data", gyro);
        SmartDashboard.putNumber("Gyro Yaw", getGyroYaw().getDegrees());
       
        LimelightHelpers.SetRobotOrientation("limelight", -m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(),0,0,0,0,0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        //SmartDashboard.putString("Limelight Pose ", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose.toString());
        
        Pose2d pose = getPose();
        field.setRobotPose(pose);   

        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        if (mt2 != null) {
            if ((Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) < 720) && mt2.tagCount > 0) {
                // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        } 
            
    }
}
