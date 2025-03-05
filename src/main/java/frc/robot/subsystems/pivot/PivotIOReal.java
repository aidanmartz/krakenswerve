package frc.robot.subsystems.pivot;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;


public class PivotIOReal implements PivotIO {
   

    private SparkFlex leader;
    //private SparkFlex follower;
    private SparkClosedLoopController closedLoopControllerPivotLeft;
    //private SparkClosedLoopController closedLoopControllerPivotRight;


    public PivotIOReal() {
        
        leader = setupPivotSparkFlex(true, Constants.CANConstants.pivotLeftId);
        closedLoopControllerPivotLeft = leader.getClosedLoopController();

        //follower = setupPivotSparkFlex(false, Constants.CANConstants.pivotRightId);
        //closedLoopControllerPivotRight = follower.getClosedLoopController();
        
    }
            
    public SparkFlex setupPivotSparkFlex(boolean left, int canid) {
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlex pivotSpark = new SparkFlex(canid, MotorType.kBrushless);
        config
                .inverted(left) // left: inverted=true, right: inverted=false
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0.0, 0.0);
        pivotSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotSpark.getEncoder().setPosition(0);
        return pivotSpark;
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        
        inputs.position.mut_replace(leader.getAbsoluteEncoder().getPosition(), Degrees);
        //inputs.velocity.mut_replace(leader.getAbsoluteEncoder().getVelocity(), DegreesPerSecond);

        inputs.appliedVoltsLeader.mut_replace(leader.getBusVoltage(), Volts);
        //inputs.appliedVoltsFollower.mut_replace(follower.getBusVoltage(), Volts);

        inputs.supplyCurrentLeader.mut_replace(leader.getOutputCurrent(), Amps);
        //inputs.supplyCurrentFollower.mut_replace(follower.getOutputCurrent(), Amps);

        inputs.torqueCurrentLeader.mut_replace(leader.getOutputCurrent(), Amps);
        //inputs.torqueCurrentFollower.mut_replace(follower.getOutputCurrent(), Amps);

        inputs.temperatureLeader.mut_replace(leader.getMotorTemperature(), Celsius);
        //inputs.temperatureFollower.mut_replace(follower.getMotorTemperature(), Celsius);
    }

    @Override
    public void runSetpoint(Angle position) {
        double setpoint = position.in(Degree);
        closedLoopControllerPivotLeft.setReference(setpoint, SparkFlex.ControlType.kPosition);
        //closedLoopControllerPivotRight.setReference(setpoint, SparkFlex.ControlType.kPosition); 
    }

    @Override
    public void runVolts(Voltage volts) {
        leader.setVoltage(volts.in(Volts));
    }

}

