package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;

        public MutDistance position = Inches.mutable(0);
        public MutLinearVelocity velocity = InchesPerSecond.mutable(0);

        public MutVoltage appliedVoltsLeader = Volts.mutable(0);
        public MutVoltage appliedVoltsFollower = Volts.mutable(0);

        public MutCurrent supplyCurrentLeader = Amps.mutable(0);
        public MutCurrent supplyCurrentFollower = Amps.mutable(0);

        public MutCurrent torqueCurrentLeader = Amps.mutable(0);
        public MutCurrent torqueCurrentFollower = Amps.mutable(0);

        public MutTemperature temperatureLeader = Celsius.mutable(0);
        public MutTemperature temperatureFollower = Celsius.mutable(0);

        public MutDistance setpointPosition = Inches.mutable(0);
        public MutLinearVelocity setpointVelocity = InchesPerSecond.mutable(0);
    }

    void updateInputs(ElevatorIOInputs inputs);

    default void runSetpoint(Distance position) {}

    default void runVolts(Voltage volts) {}

    default void runCurrent(Current current) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double p, double i, double d) {}

    default void stop() {}
}
