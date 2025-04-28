package frc.robot.subsystems.pivot;

import java.lang.annotation.ElementType;
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
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStop;
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
        Down,
        ShootL1,
        ShootL4
    };

    /*
     * Note that the real pivot is geared at 27:1.
     * TODO: AI says we should fix the positionConversionFactor in PivotIOReal
     * Double-check the positionConversionFactor in the SparkFlexConfig within
     *  PivotIOReal. It should be set up to correctly account for the 27:1 gearing
     *   ratio. If this is wrong, it will throw off all of your angle calculations.
     * The positionConversionFactor should be set to 360.0 / 27.0 if you want the
     *   encoder to report the angle of the pivot arm.
     * Ensure that the leader.getAbsoluteEncoder().getPosition() is returning the
     *   correct value. If it is not, then the positionConversionFactor is likely
     *   wrong.
     */
    private final EnumMap<Pivots, Angle> pivotsPos = new EnumMap<>(Map.ofEntries(
            Map.entry(Pivots.Intake, Degrees.of(18.8)), //7.5
            Map.entry(Pivots.Up, Degrees.of(10)), //11
            Map.entry(Pivots.Shoot, Degrees.of(4.9)), //18
            Map.entry(Pivots.Down, Degrees.of(1.5)),
            Map.entry(Pivots.ShootL1, Degrees.of(8)),
            Map.entry(Pivots.ShootL4, Degrees.of(4))));

    
    public Command pivotTo(Pivots pivot) {
        return Commands.runOnce(() -> this.setpoint = pivotsPos.get(pivot));
    }

    /**
     * Turn pivot to the correct Shoot-angle based on the elevator setting.
     * @param stop the ElevatorStop
     */
    public Command pivotToOnElevator(ElevatorStop stop) {
        return Commands.runOnce( 
            () -> {
                if (stop == ElevatorStop.L4) {
                    this.setpoint = pivotsPos.get(Pivots.ShootL4);
                }
                else if (stop == ElevatorStop.L1) {
                    this.setpoint = pivotsPos.get(Pivots.ShootL1);
                }
                else {
                    this.setpoint = pivotsPos.get(Pivots.Shoot);    
                }
            }
        );
    }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }

    // public boolean isPivotSafe() {
    //     return (setpoint.compareTo(pivotsPos.get(Pivots.Up)) > -0.5);
    // }

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
