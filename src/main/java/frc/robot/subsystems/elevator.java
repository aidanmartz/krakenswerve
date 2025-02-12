package frc.robot.subsystems;
import java.util.EnumMap;
import java.util.Map;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private SparkFlex pivotLeft;
    private SparkFlex pivotRight;
    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;
    public double elevatorLeftSpeedReq;
    public double pivotSpeedReq;
    //private Stop nextStop = Stop.SAFE;
    private double currentLevel = 0.0;
    private double currentPivot = 0.0;
    
    public void intakeSubsystem(){
        SparkFlexConfig config = new SparkFlexConfig();
        pivotLeft = new SparkFlex(Constants.CANConstants.pivotLeftId, MotorType.kBrushless);
        pivotRight = new SparkFlex(Constants.CANConstants.pivotRightId, MotorType.kBrushless);
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
        pivotRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void elevatorSubsytem() {
        SparkFlexConfig config = new SparkFlexConfig();
        elevatorLeft = new SparkFlex(Constants.CANConstants.elevatorLeftId, MotorType.kBrushless);
        elevatorRight = new SparkFlex(Constants.CANConstants.elevatorRightId, MotorType.kBrushless);
        elevatorLeftSpeedReq = 0;
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0.0, 0.0);
        elevatorLeft.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorRight.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private final EnumMap<Pivots, Double> pivotsPos = new EnumMap<>(Map.ofEntries(
      Map.entry(Pivots.Intake, 0.1),
      Map.entry(Pivots.Shoot, 20.0)
    ));

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

    public Command pivotTo(Pivots pivot){
        return Commands.runOnce(() -> setPivotPos(pivotsPos.get(pivot)));
    }

    public void setPivotPos(double pos){
        currentPivot = pos;
        pivotRight.getEncoder().setPosition(pos);
        pivotRight.getEncoder().setPosition(pos);
    }

    public double getPivotPos(){
        return currentPivot;
    }

    public enum Pivots {
        Intake,
        Shoot
    };

    private final EnumMap<Stop, Double> elevatorHeights = new EnumMap<>(Map.ofEntries(
      Map.entry(Stop.SAFE, Constants.elevatorConstants.baseHeight + 2.5),
      Map.entry(Stop.L1, 26.0 - Constants.elevatorConstants.endEffectorHeight),
      Map.entry(Stop.L2, 35.5 - Constants.elevatorConstants.endEffectorHeight),
      Map.entry(Stop.L2_ALGAE, 38.0 - Constants.elevatorConstants.endEffectorHeight),
      Map.entry(Stop.L3, 52.5 - Constants.elevatorConstants.endEffectorHeight),
      Map.entry(Stop.L3_ALGAE, 55.0 - Constants.elevatorConstants.endEffectorHeight),
      Map.entry(Stop.L4, 77.5 - Constants.elevatorConstants.endEffectorHeight)
    ));

    public Command moveTo(Stop stop){
        return Commands.runOnce(() ->  setLevel(elevatorHeights.get(stop)), this);
    }

    public void setLevel(double level){
        currentLevel = level;
        elevatorLeft.getEncoder().setPosition(level);
        elevatorRight.getEncoder().setPosition(level);
   
    }

    public double getLevel(){
        return currentLevel;
    }

}
