package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class PivotIOSim implements PivotIO{
    private final DCMotor armMotors = DCMotor.getKrakenX60(2);
    private final double gearing = 75;
    private final Distance armLength = Inches.of(24.719);
    private final Mass armWeight = Pounds.of(11);
    private final Angle minimumAngle = Degrees.of(0);
    private final Angle maximumAngle = Degrees.of(360);
    private final Angle startingAngle = Degrees.of(0);
    private final SingleJointedPivotSim sim = new DCMotorSim(
        1, //plant
        armMotors, // gearbox
         3 // measurement-std-devs
        );
    /*  This is what the training-day demo used for their arm
            SingleJointedArmSim(
        armMotors,
        gearing,
        SingleJointedArmSim.estimateMOI(
            armLength.in(Meters),
            armWeight.in(Kilograms)
        ),
        armLength.in(Meters), 
        minimumAngle.in(Radians),
        maximumAngle.in(Radians),
        true, 
        startingAngle.in(Radians)*/
    

    private final MutVoltage appliedVolts = Volts.mutable(0);
    private final double kS = 0.0;
    private final double kG = 0.126;
    private final double kV = 1.3;
    private final double kA = 5;
    private final double kP = 1.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final AngularVelocity maxVelocity = DegreesPerSecond.of(360);
    private final AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(360); 
    private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kG, kV, kA);
    private final ProfiledPIDController controller = new ProfiledPIDController(
        kP, 
        kI,
        kD,
        new Constraints(
            maxVelocity.magnitude(), 
            maxAcceleration.magnitude()
        )
    );

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        sim.update(0.02);
        inputs.position.mut_replace(sim.getAngleRads(), Radians);
        inputs.velocity.mut_replace(sim.getVelocityRadPerSec(), RadiansPerSecond);

        inputs.appliedVoltsLeader.mut_replace(appliedVolts);
        inputs.appliedVoltsFollower.mut_replace(appliedVolts);

        inputs.supplyCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.supplyCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

        inputs.torqueCurrentLeader.mut_replace(sim.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrentFollower.mut_replace(sim.getCurrentDrawAmps(), Amps);

        inputs.temperatureLeader.mut_replace(0, Celsius);
        inputs.temperatureFollower.mut_replace(0, Celsius);

        inputs.setpointPosition.mut_replace(controller.getSetpoint().position, Degrees);
        inputs.setpointVelocity.mut_replace(controller.getSetpoint().velocity, DegreesPerSecond);
    }

    @Override
    public void runSetpoint(Angle angle) {
        Angle currentAngle = Radians.of(sim.getAngleRads());

        Angle setpointAngle = Degrees.of(controller.getSetpoint().position);
        AngularVelocity setpointVelocity = DegreesPerSecond.of(controller.getSetpoint().velocity);

        Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees), angle.in(Degrees)));
        Voltage feedForwardVoltage = ff.calculateWithVelocities(setpointAngle, setpointVelocity);

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
    public void setFF(double kS, double kG, double kV, double kA) {
        this.ff = new SimpleMotorFeedforward(kS, kG, kV, kA);
    }

    @Override
    public void stop() {
        Angle currentAngle = Radians.of(sim.getAngleRads());
        controller.reset(currentAngle.in(Degrees));
        runVolts(Volts.of(0));
    }
}
