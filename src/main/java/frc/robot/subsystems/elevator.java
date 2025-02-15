package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.ElevatorFeedforward;

public class elevator extends SubsystemBase {
    private SparkFlex pivotLeft;
    private SparkFlex pivotRight;
    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;

    private SparkClosedLoopController closedLoopControllerLeft;
    private SparkClosedLoopController closedLoopControllerRight;
    private SparkClosedLoopController closedLoopControllerPivotLeft;
    private SparkClosedLoopController closedLoopControllerPivotRight;

    public double elevatorLeftSpeedReq;
    public double elevatorRightSpeedReq;
    public double pivotSpeedReq;
    private Stop nextStop = Stop.SAFE;
    private double currentLevel = 0.0;
    private double currentPivot = 0.0;
    private final ElevatorFeedforward el_Feedforward = new ElevatorFeedforward(1.0, 0.0, 0.0);

    public elevator() {
        pivotLeftSubsystem();
        pivotRightSubsystem();
        elevatorLeftSubsystem();
        elevatorRightSubsystem();
    }

    public void elevatorLeftSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        elevatorLeft = new SparkFlex(Constants.CANConstants.elevatorLeftId, MotorType.kBrushless);
        elevatorLeftSpeedReq = 0.05;
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0.0, 0.0);
        elevatorLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorLeft.getEncoder().setPosition(0);
        closedLoopControllerLeft = elevatorLeft.getClosedLoopController();
    }

    public void elevatorRightSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        elevatorRight = new SparkFlex(Constants.CANConstants.elevatorRightId, MotorType.kBrushless);
        elevatorRightSpeedReq = 0.1;
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0.0, 0.0);
        elevatorRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorRight.getEncoder().setPosition(0);
        closedLoopControllerRight = elevatorRight.getClosedLoopController();
    }

    public void pivotLeftSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        pivotLeft = new SparkFlex(Constants.CANConstants.pivotLeftId, MotorType.kBrushless);
        pivotSpeedReq = 0.05;
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.05, 0.0, 0.0);
        pivotLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotLeft.getEncoder().setPosition(0);

        closedLoopControllerPivotLeft = pivotLeft.getClosedLoopController();
    }

    public void pivotRightSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        pivotRight = new SparkFlex(Constants.CANConstants.pivotRightId, MotorType.kBrushless);
        pivotSpeedReq = 0.05;
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.05, 0.0, 0.0);
        pivotRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotRight.getEncoder().setPosition(0);

        closedLoopControllerPivotRight = pivotRight.getClosedLoopController();
    }

    public enum Stop {
        // Intake occurs at zero
        SAFE,
        L1,
        L2,
        L2_ALGAE,
        L3,
        L3_ALGAE,
        L4
    };

    private final EnumMap<Stop, Double> elevatorHeights = new EnumMap<>(Map.ofEntries(
            Map.entry(Stop.SAFE, 1.0),
            Map.entry(Stop.L1, 6.0),
            Map.entry(Stop.L2, 8.0),
            Map.entry(Stop.L2_ALGAE, 13.0),
            Map.entry(Stop.L3, 12.0),
            Map.entry(Stop.L3_ALGAE, 16.0),
            Map.entry(Stop.L4, 19.5)));

    public enum Pivots {
        Intake,
        Shoot
    };

    private final EnumMap<Pivots, Double> pivotsPos = new EnumMap<>(Map.ofEntries(
            Map.entry(Pivots.Intake, 5.0),
            Map.entry(Pivots.Shoot, -5.0)));

    public Command moveTo(Stop stop) {
        return Commands.runOnce(() -> setLevel(elevatorHeights.get(stop)), this);
    }

    public void setLevel(double level) {
        currentLevel = level;
        double feedforward = el_Feedforward.calculate(1.0);
        closedLoopControllerLeft.setReference(level, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward);
        closedLoopControllerRight.setReference(level, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward);
    }

    public double getLevel() {
        return currentLevel;
    }

    public Command pivotTo(Pivots pivot) {
        return Commands.runOnce(() -> setPivotPos(pivotsPos.get(pivot)));
    }

    public void setPivotPos(double pos) {
        currentLevel = pos;
        closedLoopControllerPivotLeft.setReference(pos, SparkFlex.ControlType.kPosition);
        closedLoopControllerPivotRight.setReference(pos, SparkFlex.ControlType.kPosition);
    }

    public double getPivotPos() {
        return currentPivot;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator Left position", elevatorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Left velocity", elevatorLeft.getEncoder().getVelocity());

        SmartDashboard.putNumber("Pivot Right position", pivotRight.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot Right velocity", pivotRight.getEncoder().getVelocity());

    }

}
