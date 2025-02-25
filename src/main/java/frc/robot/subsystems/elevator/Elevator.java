package frc.robot.subsystems.elevator;

import java.util.EnumMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.ElevatorIOReal;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorVisualizer measuredVisualizer;
    private final ElevatorVisualizer goalVisualizer;

    private Distance setpoint = Inches.of(0.0);

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;


    public Elevator(ElevatorIO io) {
        this.io = io;
        this.io.setPID(2.5, 0, 0.2);
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();

        measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
        goalVisualizer = new ElevatorVisualizer("Goal", Color.kBlue);
    }

    
    public enum ElevatorStop {
        // Intake occurs at zero
        SAFE,
        L1,
        L2,
        L2_ALGAE,
        L3,
        L3_ALGAE,
        L4
    };

    private final EnumMap<ElevatorStop, Double> elevatorHeights = new EnumMap<>(Map.ofEntries(
            Map.entry(ElevatorStop.SAFE, 0.1),
            Map.entry(ElevatorStop.L1, 3.0),
            Map.entry(ElevatorStop.L2, 6.0),
            Map.entry(ElevatorStop.L2_ALGAE, 13.0),
            Map.entry(ElevatorStop.L3, 11.5),
            Map.entry(ElevatorStop.L3_ALGAE, 18.0),
            Map.entry(ElevatorStop.L4, 21.0))); //19.5


    public Command moveTo(ElevatorStop stop) {
       Distance level = Inches.of(elevatorHeights.get(stop));
       if(this.actual.getElevatorPosition().lt(level)) {
        this.io.setPID(2.5, 0, 0);
           return Commands.runOnce(() -> this.setpoint = level);
    }  else if(this.actual.getElevatorPosition().gt(level)) {
        this.io.setPID(2.5, 0, 0);
        return Commands.runOnce(() -> this.setpoint = level);
    } else {
        return Commands.runOnce(() -> this.setpoint = level);
    }
}

    public Command setPosition(Distance position) {
        return runOnce(() -> this.setpoint = position);
    }

    public Command waitForGreaterThanPosition(Distance position) {
        return Commands.waitUntil(() -> this.inputs.position.gt(position));
    }

    public Command waitForLessThanPosition(Distance position) {
        return Commands.waitUntil(() -> this.inputs.position.lt(position));
    }

    @Override
    public void periodic() {
        super.periodic();

        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        measuredVisualizer.update(this.inputs.position);
        goalVisualizer.update(this.setpoint);

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.runSetpoint(this.setpoint);
        }

        actual.updateElevatorPosition(this.inputs.position);
        target.updateElevatorPosition(this.inputs.setpointPosition);
        goal.updateElevatorPosition(this.setpoint);
    }

}
