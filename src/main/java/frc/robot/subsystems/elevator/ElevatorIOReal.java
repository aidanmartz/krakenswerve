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
// import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

public class ElevatorIOReal implements ElevatorIO {

    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;
    SparkFlexConfig elevatorLeftConfig;
    SparkFlexConfig elevatorRightConfig;
    private SparkClosedLoopController closedLoopControllerLeft;
    
    private MutDistance currentPosition = Inches.mutable(0.0);

    public ElevatorIOReal() {

        elevatorLeft = new SparkFlex(Constants.CANConstants.elevatorLeftId, MotorType.kBrushless);
        elevatorLeftConfig = new SparkFlexConfig();

        elevatorRight = new SparkFlex(Constants.CANConstants.elevatorRightId, MotorType.kBrushless);
        elevatorRightConfig = new SparkFlexConfig();

        elevatorLeftConfig.inverted(true);
        elevatorLeftConfig.idleMode(IdleMode.kBrake);

        elevatorLeftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        elevatorLeftConfig.closedLoop.pid(0.350, 0.0, 0.0); // 0.2515, 0.0, 0.0
        elevatorLeftConfig.closedLoop.maxMotion.allowedClosedLoopError(0.1); //0.5
        elevatorLeftConfig.closedLoop.maxMotion.maxVelocity(4000); //8000
        elevatorLeftConfig.closedLoop.maxMotion.maxAcceleration(3500);//2000

        elevatorRightConfig.follow(Constants.CANConstants.elevatorLeftId,true);

        elevatorLeft.configure(elevatorLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //elevatorLeftConfig.softLimit
            //.reverseSoftLimit(-71)
            //.reverseSoftLimitEnabled(true).forwardSoftLimit(0).forwardSoftLimitEnabled(true);

        elevatorRight.configure(elevatorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        closedLoopControllerLeft = elevatorLeft.getClosedLoopController();
        elevatorLeft.getEncoder().setPosition(0);
    }

    @Override
    public void runSetpoint(Distance position) {
        currentPosition.mut_replace(position); // probably unneeded since we are capturing inputs elsewhere?
        double level = position.in(Inches);

        closedLoopControllerLeft.setReference(level, SparkFlex.ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0,
                0.54);
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
        SmartDashboard.putNumber("Elevator/Left position", elevatorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("Elevator/Left velocity", elevatorLeft.getEncoder().getVelocity());
    }




}
