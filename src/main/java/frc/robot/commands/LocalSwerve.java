package frc.robot.commands;
import frc.lib.util.LoggedCommandBase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LocalSwerve extends LoggedCommandBase{
    private final Swerve m_swerve;
    private final Pose2d targetPose;
    private final boolean precise;
    private final double positionIZone = 4;
    private final double positionKS = 0.02;
    private final double positionTolerance = 1;
    private final double roughPositionTolerance = 2.5; // inches
    private final double rotationKS = 0.02;
    private final double rotationIZone = 4;
    private final double maxSpeed = Constants.Swerve.maxSpeed / 5.0; // 3.0
    private final double maxAngularVelocity = Constants.Swerve.maxAngularVelocity / 3.0; // 2.0

    private final double rotationTolerance = 0.5; // degrees
    private final double roughRotatationTolerance = 1.5; // degrees
    
    private final PIDController xPID = new PIDController(Constants.Swerve.driveKP, 0, 0);
    private final PIDController yPID = new PIDController(Constants.Swerve.driveKP, 0, 0);
    private final PIDController rPID = new PIDController(0.10, 0, 0);

    public LocalSwerve(Swerve m_swerve, Pose2d targetPose, boolean precise){
        super();

        targetPose = Swerve.flipIfRed(targetPose);


        this.m_swerve = m_swerve;
        this.targetPose = targetPose;
        this.precise = precise;
        SmartDashboard.putBoolean("Precise?", precise);
        SmartDashboard.putString("target pose", targetPose.toString());
        //addRequirements(m_swerve);

        xPID.setIZone(positionIZone); // Only use Integral term within this range
        xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        xPID.setSetpoint(Units.metersToInches(targetPose.getX()));
        xPID.setTolerance(precise ? positionTolerance : roughPositionTolerance);

        yPID.setIZone(positionIZone); // Only use Integral term within this range
        yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        yPID.setSetpoint(Units.metersToInches(targetPose.getY())); // TODO Set derivative, too
        yPID.setTolerance(precise ? positionTolerance : roughPositionTolerance);

        rPID.enableContinuousInput(-180.0, 180.0);
        rPID.setIZone(rotationIZone); // Only use Integral term within this range
        rPID.setIntegratorRange(rotationKS * 2, rotationKS * 2);
        rPID.setSetpoint(targetPose.getRotation().getDegrees());
        rPID.setTolerance(precise ? rotationTolerance : roughRotatationTolerance); // TODO Set derivative, too

    }

    @Override
    public void initialize() {
        super.initialize();

        xPID.reset();
        yPID.reset();
        rPID.reset();
    }

        @Override
    public void execute() {
        Pose2d pose = m_swerve.getPose();
        Translation2d position = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();

        /* TODO Consider a potential need to rotate most of the way first, then translate */

        double xCorrection = xPID.calculate(Units.metersToInches(position.getX()));
        double xFeedForward = positionKS * Math.signum(xCorrection);
        double xVal = MathUtil.clamp(xCorrection + xFeedForward, -1.0, 1.0);

        double yCorrection = yPID.calculate(Units.metersToInches(position.getY()));
        double yFeedForward = positionKS * Math.signum(yCorrection);
        double yVal = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);

        double correction = rPID.calculate(rotation.getDegrees());
        double feedForward = rotationKS * Math.signum(correction);
        double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);

        /* Drive */
        m_swerve.drive(
            new Translation2d(xVal, yVal).times(maxSpeed),
            rotationVal * maxAngularVelocity,
            true, true, false
         );
    }

}
