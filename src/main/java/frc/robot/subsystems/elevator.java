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
    private SparkClosedLoopController closedLoopControllerPivotLeft;
    private SparkClosedLoopController closedLoopControllerPivotRight;

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;
    private SparkClosedLoopController closedLoopControllerLeft;
    private SparkClosedLoopController closedLoopControllerRight;

    //unused// private Stop nextStop = Stop.SAFE;
    private double currentLevel = 0.0;
    private double currentPivot = 0.0;
    private final ElevatorFeedforward el_Feedforward = new ElevatorFeedforward(1.0, 0.0, 0.0);

    public elevator() {
        elevatorLeft = setupElevatorSparkFlex(true, Constants.CANConstants.elevatorLeftId);
        closedLoopControllerLeft = elevatorLeft.getClosedLoopController();
        
        elevatorRight = setupElevatorSparkFlex(false, Constants.CANConstants.elevatorRightId);
        closedLoopControllerRight = elevatorRight.getClosedLoopController();

        pivotLeft = setupPivotSparkFlex(true, Constants.CANConstants.pivotLeftId);
        closedLoopControllerPivotLeft = pivotLeft.getClosedLoopController();

        pivotRight = setupPivotSparkFlex(false, Constants.CANConstants.pivotRightId);
        closedLoopControllerPivotRight = pivotRight.getClosedLoopController();
    }

    public SparkFlex setupElevatorSparkFlex(boolean left, int canid) {
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlex elevatorSpark = new SparkFlex(canid, MotorType.kBrushless);
        config
                .inverted(left) // left: inverted=true, right: inverted=false
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0.0, 0.0);
        elevatorSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorSpark.getEncoder().setPosition(0);
        return elevatorSpark;
    }

    // Maybe could be combined with setupElevatorSparkFlex?
    public SparkFlex setupPivotSparkFlex(boolean left, int canid) {
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlex pivotSpark = new SparkFlex(canid, MotorType.kBrushless);
        config
                .inverted(left) // left: inverted=true, right: inverted=false
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0.0, 0.0);
        pivotSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotSpark.getEncoder().setPosition(0);
        return pivotSpark;
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
        currentPivot = pos;
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
