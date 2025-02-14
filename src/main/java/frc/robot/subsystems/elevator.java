package frc.robot.subsystems;
import java.util.EnumMap;
import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private SparkFlex pivotLeft;
    private SparkFlex pivotRight;
    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;

    private SparkClosedLoopController closedLoopControllerLeft;
    private SparkClosedLoopController closedLoopControllerRight;
    private SparkClosedLoopController closedLoopControllerPivotLeft;
    private SparkClosedLoopController closedLoopControllerPivotRight;
    
    public double elevatorLeftSpeedReq;
    public double elevatorRightSpeedReq;
    public double pivotSpeedReq;
    private Stop nextStop = Stop.SAFE;
    private double currentLevel = 0.0;
    private double currentPivot = 0.0;


    public elevator(){
        pivotLeftSubsystem();
        pivotRightSubsystem();
        elevatorLeftSubsystem();
        elevatorRightubsystem();
    }
    

    public void elevatorLeftSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        elevatorLeft = new SparkFlex(Constants.CANConstants.elevatorLeftId, MotorType.kBrushless);
        elevatorLeftSpeedReq = 0.2;
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(2, 0.0, 0.1);
        elevatorLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorLeft.getEncoder().setPosition(0);
        closedLoopControllerLeft = elevatorLeft.getClosedLoopController();
    }

    public void elevatorRightubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        elevatorRight = new SparkFlex(Constants.CANConstants.elevatorRightId, MotorType.kBrushless);
        elevatorRightSpeedReq = 0.2;
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(2, 0.0, 0.1);
        elevatorRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorRight.getEncoder().setPosition(0);
        closedLoopControllerRight = elevatorRight.getClosedLoopController();
    }


    

    public void pivotLeftSubsystem(){
        SparkFlexConfig config = new SparkFlexConfig();
        pivotLeft = new SparkFlex(Constants.CANConstants.pivotLeftId, MotorType.kBrushless);
        pivotSpeedReq = 0;
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
        pivotLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoopControllerPivotLeft = pivotLeft.getClosedLoopController();
    }

    public void pivotRightSubsystem(){
        SparkFlexConfig config = new SparkFlexConfig();
        pivotRight = new SparkFlex(Constants.CANConstants.pivotRightId, MotorType.kBrushless);
        pivotSpeedReq = 0;
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
        pivotRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        closedLoopControllerPivotRight = pivotRight.getClosedLoopController();
    }



    public enum Stop {
        // Intake occurs at zero
        SAFE,
        L1,
        L2,
        L2_ALGAE,
        L3,
        L3_ALGAE,
        L4
    };

    private final EnumMap<Stop, Double> elevatorHeights = new EnumMap<>(Map.ofEntries(
        Map.entry(Stop.SAFE, 0.0),
        Map.entry(Stop.L1, 6.0),
        Map.entry(Stop.L2, 10.0),
        Map.entry(Stop.L2_ALGAE, 13.0),
        Map.entry(Stop.L3, 15.0),
        Map.entry(Stop.L3_ALGAE,16.0),
        Map.entry(Stop.L4, 20.0)
      ));




    public enum Pivots {
        Intake,
        Shoot
    };

    private final EnumMap<Pivots, Double> pivotsPos = new EnumMap<>(Map.ofEntries(
      Map.entry(Pivots.Intake, 0.1),
      Map.entry(Pivots.Shoot, 20.0)
    ));




    public Command moveTo(Stop stop){
        return Commands.runOnce(() ->  setLevel(elevatorHeights.get(stop)), this);
    }

    public void setLevel(double level){
        currentLevel = level;
       closedLoopControllerLeft.setReference(level,SparkFlex.ControlType.kPosition);
       closedLoopControllerRight.setReference(level,SparkFlex.ControlType.kPosition);
    }

    public double getLevel(){
        return currentLevel;
    }





    public Command pivotTo(Pivots pivot){
        return Commands.runOnce(() -> setPivotPos(pivotsPos.get(pivot)));
    }

    public void setPivotPos(double pos){
        currentLevel = pos;
        closedLoopControllerPivotLeft.setReference(pos,SparkFlex.ControlType.kPosition);
        closedLoopControllerPivotRight.setReference(pos,SparkFlex.ControlType.kPosition);
    }

    public double getPivotPos(){
        return currentPivot;
    }




    @Override
    public void periodic(){
        SmartDashboard.putNumber("encoderLeft position", elevatorLeft.getEncoder().getPosition());
        SmartDashboard.putNumber("elevatorLeft velocity", elevatorLeft.getEncoder().getVelocity());

    }



}
