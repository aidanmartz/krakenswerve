package frc.robot.subsystems.intake;
import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class IntakeIOInputsAutoLogged extends IntakeIO.IntakeIOInputs implements LoggableInputs, Cloneable {
    
    @Override
    public void toLog(LogTable table) {
        table.put("MotorConnected", MotorConnected);
        table.put("Velocity", velocity);
        table.put("AppliedVolts", appliedVolts);
        table.put("SupplyCurrent", supplyCurrent);
        table.put("TorqueCurrent", torqueCurrent);
        table.put("Temperature", temperature);
      }

    @Override
    public void fromLog(LogTable table) {
        MotorConnected = table.get("MotorConnected", MotorConnected);
        velocity = table.get("Velocity", velocity);
        appliedVolts = table.get("AppliedVolts", appliedVolts);
        supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
        torqueCurrent = table.get("TorqueCurrent", torqueCurrent);
        temperature = table.get("Temperature", temperature);
    }

    public IntakeIOInputsAutoLogged clone() {
        IntakeIOInputsAutoLogged copy = new IntakeIOInputsAutoLogged();
        copy.MotorConnected = this.MotorConnected;
        copy.velocity = this.velocity;
        copy.appliedVolts = this.appliedVolts;
        copy.supplyCurrent = this.supplyCurrent;
        copy.torqueCurrent = this.torqueCurrent;
        copy.temperature = this.temperature;
        return copy;
    }
}
