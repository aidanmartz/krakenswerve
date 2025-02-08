package frc.robot.subsystems;

import frc.lib.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.concurrent.locks.ReentrantLock;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Boolean ll;
    
    //private final ReentrantLock swerveModLock = new ReentrantLock();
    private final Notifier odoNotifier;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Swerve.pigeonCanBus);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        ll = false;


        boolean isFastOdo = Constants.Swerve.isOnCANivore;
        odoNotifier = new Notifier(this::updateSwerveOdom);
        odoNotifier.startPeriodic(isFastOdo ? 1.0 / 250.0 : 1.0 / 50.0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
         try{
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
            this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelativeAuto(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) 
        ),
        config,
        () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        },
        this
      );
    }catch(Exception e){
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }
    }

    private double limelightRotation() {
        double targetAngularVel = LimelightHelpers.getTXNC("limelight") * Constants.VisionConstants.kCameraAimScaler;
        targetAngularVel *= -1;
        SmartDashboard.putNumber("limelight/Angular Velcity", targetAngularVel);
        return targetAngularVel;
    }


  private double limelightX() {
        double targetForwardSpeed = Constants.VisionConstants.kCameraRangeScaler / LimelightHelpers.getTA("limelight");
        //targetForwardSpeed *= 1;
        SmartDashboard.putNumber("limelight/Forward Speed", targetForwardSpeed);
        return targetForwardSpeed; 
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop, boolean isLimelight) {
        SwerveModuleState[] swerveModuleStates;
        if( isLimelight){ 
        ll = true;
        swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                                limelightX(), 
                                translation.getY(), 
                                limelightRotation())
                            );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        
       } else{
        ll=false;
        swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
        } 
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
     
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelativeAuto(ChassisSpeeds desirChassisSpeeds) {
        driveRobotRelative(desirChassisSpeeds, false);
    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds, boolean isOpenLoop) {
        ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02); 

        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds); 
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void alignStraight() {
        SwerveModuleState aligned = new SwerveModuleState(0.0, new Rotation2d());

        for(SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(aligned, false);
        }
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public Command zeroHeading(){
        return Commands.runOnce(
            () -> {
                swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
            }, 
            this);
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public Command resetModulesToAbsolute(){
        return Commands.runOnce(
            () -> {
                for(SwerveModule mod : mSwerveMods){
                    mod.resetToAbsolute();
                }
            }, 
            this);
    }

    private void updateSwerveOdom() { // function will be called 250 times a second
        swerveOdometry.update(getGyroYaw(), getModulePositions());
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putBoolean("limelight/use limelight", ll);
        SmartDashboard.putNumber("limelight/TX", LimelightHelpers.getTXNC("limelight"));
        SmartDashboard.putNumber("limelight/TA", LimelightHelpers.getTA("limelight"));
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond); 

   
        }
    }
}