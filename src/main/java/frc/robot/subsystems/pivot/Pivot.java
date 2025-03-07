package frc.robot.subsystems.pivot;

import java.util.EnumMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

//import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class Pivot extends SubsystemBase {
    
    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", 0.0);
    
    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", 0.1);

    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private final PivotVisualizer measuredVisualizer;
    private final PivotVisualizer setpointVisualizer;
    private final PivotVisualizer goalVisualizer;

    
    private Angle setpoint = Degrees.of(0.0);

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;


    public Pivot(PivotIO io) {
        this.io = io;
        this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();

        this.measuredVisualizer = new PivotVisualizer("Measured", Color.kWhite);
        this.setpointVisualizer = new PivotVisualizer("Setpoint", Color.kBlue);
        this.goalVisualizer = new PivotVisualizer("Goal", Color.kGreen);

    }

    public enum Pivots {
        Intake,
        Shoot,
        Up, 
        Down
    };

    // geared at 25:1
    private final EnumMap<Pivots, Angle> pivotsPos = new EnumMap<>(Map.ofEntries(
            Map.entry(Pivots.Intake, Degrees.of(18.8)), //7.5
            Map.entry(Pivots.Up, Degrees.of(10)), //11
            Map.entry(Pivots.Shoot, Degrees.of(5)), //18
            Map.entry(Pivots.Down, Degrees.of(1.5))));

    
    public Command pivotTo(Pivots pivot) {
        return Commands.runOnce(() -> this.setpoint = pivotsPos.get(pivot));
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }

    public boolean pivotSafe(){
        return (setpoint.compareTo(pivotsPos.get(Pivots.Up)) > -0.5);
    }

    @Override
    public void periodic() {
        super.periodic();

        this.io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.runSetpoint(this.setpoint);
        }

        actual.updatePivotAngle(this.inputs.position);
        target.updatePivotAngle(this.inputs.setpointPosition);
        goal.updatePivotAngle(this.setpoint);

        this.measuredVisualizer.update(this.inputs.position, actual.getElevatorPosition());
        this.setpointVisualizer.update(this.inputs.setpointPosition, target.getElevatorPosition());
        this.goalVisualizer.update(this.setpoint, goal.getElevatorPosition());
    }

    

}
