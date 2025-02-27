package frc.robot.subsystems.elevator;

import java.util.EnumMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {

    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/Gains/kP", 2.5);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/Gains/kD", 0.2);

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/Gains/kS", 0.0);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/Gains/kV", 0.0);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/Gains/kA", 1.45);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/Gains/kG", 0.0);
     
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
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();

        measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
        goalVisualizer = new ElevatorVisualizer("Goal", Color.kBlue);
    }

    
    public enum ElevatorStop {
        SAFE, // Intake occurs at "zero"
        L1,
        L2,
        L2_ALGAE,
        L3,
        L3_ALGAE,
        L4
    };

    private final EnumMap<ElevatorStop, Distance> elevatorHeights = new EnumMap<>(Map.ofEntries(
            Map.entry(ElevatorStop.SAFE, Inches.of(0.1)),
            Map.entry(ElevatorStop.L1, Inches.of(3.0)),
            Map.entry(ElevatorStop.L2, Inches.of(6.0)),
            Map.entry(ElevatorStop.L2_ALGAE, Inches.of(13.0)),
            Map.entry(ElevatorStop.L3, Inches.of(11.5)),
            Map.entry(ElevatorStop.L3_ALGAE, Inches.of(18.0)),
            Map.entry(ElevatorStop.L4, Inches.of(20.0))  //19.5
        ));


    public Command moveTo(ElevatorStop stop) {
        return Commands.runOnce(() -> {
            this.setpoint = elevatorHeights.get(stop);
        });

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
