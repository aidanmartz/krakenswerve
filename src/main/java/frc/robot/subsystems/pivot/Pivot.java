package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

import java.util.EnumMap;
import java.util.Map;

public class Pivot extends SubsystemBase {
    

    private SparkFlex pivotLeft;
    private SparkFlex pivotRight;
    private SparkClosedLoopController closedLoopControllerPivotLeft;
    private SparkClosedLoopController closedLoopControllerPivotRight;

    private double currentPivot = 0.0;

    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", 0.1);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", 0.1);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", 0.1);
    private static final LoggedTunableNumber kS =
        new LoggedTunableNumber("Pivot/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV =
        new LoggedTunableNumber("Pivot/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA =
        new LoggedTunableNumber("Pivot/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG =
        new LoggedTunableNumber("Pivot/Gains/kG", 0.1);

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final PivotVisualizer measuredVisualizer;
    private final PivotVisualizer setpointVisualizer;
    private final PivotVisualizer goalVisualizer;

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;

    private Angle setpoint = Degrees.of(0.0);

    public Pivot(PivotIO io) {
        this.io = io;
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();

        this.measuredVisualizer = new PivotVisualizer("measured", Color.kWhite);
        this.setpointVisualizer = new PivotVisualizer("setpoint", Color.kBlue);
        this.goalVisualizer = new PivotVisualizer("goal", Color.kGreen);

    }

    public enum Pivots {
        Intake,
        Shoot
    };

    private final EnumMap<Pivots, Angle> pivotsPos = new EnumMap<>(Map.ofEntries(
            Map.entry(Pivots.Intake, Degrees.of(5.0)),
            Map.entry(Pivots.Shoot, Degrees.of(0.0))));

    
    public Command pivotTo(Pivots pivot) {
        return Commands.runOnce(() -> setPivotPosition(pivotsPos.get(pivot)));
    }

    public Command setPivotPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }

    public double getPivotPos() {
        return currentPivot;
    }

    @Override
    public void periodic() {
        super.periodic();

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.updateInputs(inputs);
            Logger.processInputs("Pivot", inputs);

            LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);

            LoggedTunableNumber.ifChanged(
                hashCode(), () -> io.setFF(kS.get(), kG.get(), kV.get(), kA.get()), kS, kG, kV, kA);

            this.io.runSetpoint(this.setpoint);
        }

        this.measuredVisualizer.update(this.inputs.position, actual.getElevatorPosition());
        this.setpointVisualizer.update(this.inputs.setpointPosition, target.getElevatorPosition());
        this.goalVisualizer.update(this.setpoint, goal.getElevatorPosition());

        actual.updatePivotAngle(this.inputs.position);
        target.updatePivotAngle(this.inputs.setpointPosition);
        goal.updatePivotAngle(this.setpoint);
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }
    

}
