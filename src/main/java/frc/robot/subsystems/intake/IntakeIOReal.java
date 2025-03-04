package frc.robot.subsystems.intake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class IntakeIOReal implements IntakeIO {
    private TalonFX intakeMotor;
    private CANcoder intakeEncoder;

    public IntakeIOReal() {
        intakeMotor = new TalonFX(Constants.CANConstants.intakeId, Constants.CANConstants.canBus);
        intakeMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        intakeMotor.getConfigurator().setPosition(0.0);
        intakeEncoder = new CANcoder(Constants.CANConstants.intakeId, Constants.CANConstants.canBus);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        
        inputs.velocity.mut_replace(intakeEncoder.getVelocity().getValueAsDouble(), DegreesPerSecond);

        inputs.appliedVolts.mut_replace(intakeMotor.getSupplyVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(intakeMotor.getSupplyCurrent().getValueAsDouble(), Amps);

        inputs.torqueCurrent.mut_replace(intakeMotor.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.temperature.mut_replace(intakeMotor.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    @Override
    public void setSpeed(double speed){
        intakeMotor.set(speed);
    }

}

