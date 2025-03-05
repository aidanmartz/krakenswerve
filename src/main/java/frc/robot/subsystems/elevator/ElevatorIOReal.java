package frc.robot.subsystems.elevator;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.ElevatorFeedforward;
//import edu.wpi.first.units.TemperatureUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class ElevatorIOReal implements ElevatorIO {

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;
    private SparkClosedLoopController closedLoopControllerLeft;
    private SparkClosedLoopController closedLoopControllerRight;
    
    //unused// private Stop nextStop = Stop.SAFE;
    private MutDistance currentPosition = Inches.mutable(0.0);
    private ElevatorFeedforward el_Feedforward = new ElevatorFeedforward(0.0, 1.0, 0);
    
    public ElevatorIOReal() {
        
        elevatorLeft = setupElevatorSparkFlex(true, Constants.CANConstants.elevatorLeftId);
        closedLoopControllerLeft = elevatorLeft.getClosedLoopController();
        
        elevatorRight = setupElevatorSparkFlex(false, Constants.CANConstants.elevatorRightId);
        closedLoopControllerRight = elevatorRight.getClosedLoopController();
    }

    public SparkFlex setupElevatorSparkFlex(boolean left, int canid) {
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlex elevatorSpark = new SparkFlex(canid, MotorType.kBrushless);
        config
        .inverted(left) // left: inverted=true, right: inverted=false
                .idleMode(IdleMode.kBrake);
                config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(0.1);
                
                config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0.0, 0.0);
                elevatorSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                elevatorSpark.getEncoder().setPosition(0);
        return elevatorSpark;
    }
    
    @Override
    public void runSetpoint(Distance position) {
        currentPosition.mut_replace(position); // probably unneeded since we are capturing inputs elsewhere?
        double level = position.in(Inches);
        double feedforward = el_Feedforward.calculate(0.01);
        closedLoopControllerLeft.setReference(level, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward);
        closedLoopControllerRight.setReference(level, SparkFlex.ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward);
    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
       // el_Feedforward = new ElevatorFeedforward( kS, kG, kV, kA);
    }
    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        //inputs.position.mut_replace(sim.getPositionMeters(), Meters); // ????
        //inputs.velocity.mut_replace(sim.getVelocityMetersPerSecond(), MetersPerSecond); // ????
        //inputs.setpointPosition.mut_replace(controller.getSetpoint(), Meters); // ????
        //inputs.setpointVelocity.mut_replace(0, MetersPerSecond); // ????
        
        inputs.position.mut_replace(elevatorLeft.getAbsoluteEncoder().getPosition(), Meters);
        inputs.velocity.mut_replace(elevatorLeft.getAbsoluteEncoder().getVelocity(), MetersPerSecond);
        
        inputs.appliedVoltsLeader.mut_replace(elevatorLeft.getBusVoltage(), Volts);
        inputs.appliedVoltsFollower.mut_replace(elevatorRight.getBusVoltage(), Volts);

        inputs.supplyCurrentLeader.mut_replace(elevatorLeft.getOutputCurrent(), Amps);
        inputs.supplyCurrentFollower.mut_replace(elevatorRight.getOutputCurrent(), Amps);

        inputs.torqueCurrentLeader.mut_replace(elevatorLeft.getOutputCurrent(), Amps);
        inputs.torqueCurrentFollower.mut_replace(elevatorRight.getOutputCurrent(), Amps);

        inputs.temperatureLeader.mut_replace(elevatorLeft.getMotorTemperature(), Celsius);
        inputs.temperatureFollower.mut_replace(elevatorRight.getMotorTemperature(), Celsius);
        
    }

    public void runVolts(Voltage volts) {
        elevatorLeft.setVoltage(volts);
        elevatorRight.setVoltage(volts);
    }

    /*
    public void runCurrent(Current current) {}
    public void setBrakeMode(boolean enabled) {}
    public void setPID(double p, double i, double d) {}
    */
    
    public void stop() {
        runVolts(Volts.of(0));
    }
    
    public void periodic() {
        SmartDashboard.putNumber("Elevator Left position", elevatorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator Left velocity", elevatorLeft.getEncoder().getVelocity());
    }




}
