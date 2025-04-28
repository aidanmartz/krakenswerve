package frc.robot.subsystems.intake;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class IntakeIOReal implements IntakeIO {
    private TalonFX intakeMotor;
    private CANcoder intakeEncoder;

    // Leaving these here in case we want voltage limits for the intake 
    private final VoltageOut intakeVoltage = new VoltageOut(2).withEnableFOC(true);
    private final VoltageOut shootVoltage = new VoltageOut(2).withEnableFOC(true);
    private final VoltageOut boostVoltage = new VoltageOut(2).withEnableFOC(true);

    public IntakeIOReal() {
        // Create the intake kraken
        intakeMotor = new TalonFX(Constants.CANConstants.intakeId, Constants.CANConstants.canBus);
        
        // Configure it differently than the swerves
        var intakeConfig = new TalonFXConfiguration();

        // Brake mode
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Make sure current limiting is enabled
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Limit the current to the intake to max 70A
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 10;

        // Wait for time at limit then lower the limit - useful to not brownout 
        intakeConfig.CurrentLimits.SupplyCurrentLowerLimit = 8;
        
        // How long to wait before throttling
        intakeConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        // Keep the voltages within range
        intakeConfig.Voltage.PeakForwardVoltage = 12;
        intakeConfig.Voltage.PeakReverseVoltage = 12;

        intakeMotor.getConfigurator().apply(intakeConfig);

        // Not really used except for tracking velocity but keep jic
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
    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void periodic() {
    }

}

