package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;

import static edu.wpi.first.units.Units.*;

public interface IntakeIO {
    class IntakeIOInputs {
        public boolean MotorConnected = true;
        
        public MutAngularVelocity velocity = DegreesPerSecond.mutable(0);

        public MutVoltage appliedVolts = Volts.mutable(0);

        public MutCurrent supplyCurrent = Amps.mutable(0);

        public MutCurrent torqueCurrent = Amps.mutable(0);

        public MutTemperature temperature = Celsius.mutable(0);

    }   
    void updateInputs(IntakeIOInputs inputs);

    default void setPID(double p, double i, double d) {}

    default void setFF(double kS, double kG, double kV, double kA) {}

    default void setSpeed(double speed) {}


}
