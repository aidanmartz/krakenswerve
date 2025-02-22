package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.*;

public class ElevatorIOSim implements ElevatorIO {

    // TODO: Create some Constants for these
    private final ElevatorSim sim = new ElevatorSim(
        DCMotor.getNeoVortex(2),
        4, 
        Pounds.of(35).in(Kilograms), // carriage-mass (fixed, doubled second stage)
        Inches.of(1.75).in(Meters), // sprocket radius 0.875 est
        Inches.of(0).in(Meters), // minheight
        Inches.of(90).in(Meters), // maxheight
        true, 
        Inches.of(0).in(Meters) // starting height
    );

    private final MutVoltage appliedVolts = Volts.mutable(0);
    private final PIDController controller = new PIDController(100, 0, 0);
    private final ElevatorFeedforward ff = new ElevatorFeedforward(0, 1, 0);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        sim.update(0.02); // 20ms update
        inputs.position.mut_replace(sim.getPositionMeters(), Meters);
        inputs.velocity.mut_replace(sim.getVelocityMetersPerSecond(), MetersPerSecond);

        inputs.appliedVoltsLeader.mut_replace(appliedVolts);
        inputs.appliedVoltsFollower.mut_replace(appliedVolts);

        inputs.supplyCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.supplyCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

        inputs.torqueCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

        inputs.temperatureLeader.mut_replace(0, Celsius);
        inputs.temperatureFollower.mut_replace(0, Celsius);

        inputs.setpointPosition.mut_replace(controller.getSetpoint(), Meters);
        inputs.setpointVelocity.mut_replace(0, MetersPerSecond);
    }

    @Override
    public void runSetpoint(Distance position) {
        Distance currentHeight = Meters.of(sim.getPositionMeters());
        LinearVelocity currentVelocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());

        Voltage controllerVoltage = Volts.of(controller.calculate(currentHeight.in(Inches), position.in(Inches)));
        Voltage feedForwardVoltage = Volts.of(ff.calculate(currentVelocity.in(MetersPerSecond)));

        Voltage effort = controllerVoltage.plus(feedForwardVoltage);

        runVolts(effort);
    }

    @Override
    public void runVolts(Voltage volts) {
        double clampedEffort = MathUtil.clamp(volts.in(Volts), -12, 12);
        appliedVolts.mut_replace(clampedEffort, Volts);
        sim.setInputVoltage(clampedEffort);
    }

    @Override
    public void setPID(double p, double i, double d) {
        controller.setPID(p, i, d);
    }

    @Override
    public void stop() {
        runVolts(Volts.of(0));
    }
}
