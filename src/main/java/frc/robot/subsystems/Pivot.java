package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.ClosedLoopSlot;
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
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.math.controller.ElevatorFeedforward;

public class Pivot extends SubsystemBase {
    private SparkFlex pivotLeft;
    private SparkFlex pivotRight;
    private SparkClosedLoopController closedLoopControllerPivotLeft;
    private SparkClosedLoopController closedLoopControllerPivotRight;

    private double currentPivot = 0.0;

    public Pivot() {
        pivotLeft = setupPivotSparkFlex(true, Constants.CANConstants.pivotLeftId);
        closedLoopControllerPivotLeft = pivotLeft.getClosedLoopController();

        pivotRight = setupPivotSparkFlex(false, Constants.CANConstants.pivotRightId);
        closedLoopControllerPivotRight = pivotRight.getClosedLoopController();
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

    public enum Pivots {
        Intake,
        Shoot
    };

    private final EnumMap<Pivots, Double> pivotsPos = new EnumMap<>(Map.ofEntries(
            Map.entry(Pivots.Intake, 5.0),
            Map.entry(Pivots.Shoot, -5.0)));

    
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
        SmartDashboard.putNumber("Pivot Right position", pivotRight.getEncoder().getPosition());
        SmartDashboard.putNumber("Pivot Right velocity", pivotRight.getEncoder().getVelocity());
    }

    
}
