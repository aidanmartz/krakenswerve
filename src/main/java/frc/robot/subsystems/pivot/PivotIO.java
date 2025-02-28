package frc.robot.subsystems.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface PivotIO {
    // Removed Autolog because it was finicky so implemented manually.
    class PivotIOInputs {
        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;

        public MutAngle position = Degrees.mutable(0);
        public MutAngularVelocity velocity = DegreesPerSecond.mutable(0);

        public MutVoltage appliedVoltsLeader = Volts.mutable(0);
        public MutVoltage appliedVoltsFollower = Volts.mutable(0);

        public MutCurrent supplyCurrentLeader = Amps.mutable(0);
        public MutCurrent supplyCurrentFollower = Amps.mutable(0);

        public MutCurrent torqueCurrentLeader = Amps.mutable(0);
        public MutCurrent torqueCurrentFollower = Amps.mutable(0);

        public MutTemperature temperatureLeader = Celsius.mutable(0);
        public MutTemperature temperatureFollower = Celsius.mutable(0);

        public MutAngle setpointPosition = Degrees.mutable(0);
        public MutAngularVelocity setpointVelocity = DegreesPerSecond.mutable(0);
    }

    void updateInputs(PivotIOInputs inputs);

    default void runSetpoint(Angle position) {}

    default void runVolts(Voltage volts) {}

    default void runCurrent(Current current) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double p, double i, double d) {}

    default void setFF(double kS, double kG, double kV, double kA) {}

    default void stop() {}

}
