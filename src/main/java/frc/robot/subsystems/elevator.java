package frc.robot.subsystems;

//import java.util.EnumMap;
//import java.util.Map;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//port com.revrobotics.spark.config.;

import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {
    private SparkFlex elevatorLeft;
    private SparkFlex elevatorRight;
    public double elevatorLeftSpeedReq;

    public void intakeSubsytem() {
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

    public void elevatorLeft() {
        elevatorLeftSpeedReq = -1 * Constants.VortexMotorConstants.kFreeSpeedRpm;
        elevatorLeft.set(elevatorLeftSpeedReq);
        elevatorRight.set(elevatorLeftSpeedReq);
    }

    public void elevatorRight() {
        elevatorLeftSpeedReq = -1 * Constants.VortexMotorConstants.kFreeSpeedRpm;
        elevatorLeft.set(elevatorLeftSpeedReq);
        elevatorRight.set(elevatorLeftSpeedReq);
    }

}
