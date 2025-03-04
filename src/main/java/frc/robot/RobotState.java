package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutDistance;

import static edu.wpi.first.units.Units.*;

public class RobotState {
    private static RobotState measuredInstance;
    private static RobotState desiredInstance;
    private static RobotState goalInstance;

    private MutDistance elevatorPosition;
    private MutAngle pivotPosition;
    private MutAngularVelocity intakeVelocity;
    private MutCurrent intakeSupplyCurrent;
    private MutCurrent intakeTorqueCurrent;

    private RobotState() {
        elevatorPosition = Inches.mutable(0);
        pivotPosition = Degrees.mutable(0);
        intakeVelocity = DegreesPerSecond.mutable(0);
        intakeSupplyCurrent = Amps.mutable(0);
        intakeTorqueCurrent = Amps.mutable(0);
    }

    public static RobotState getMeasuredInstance() {
        if (measuredInstance == null) {
            measuredInstance = new RobotState();
        }
        return measuredInstance;
    }

    public static RobotState getDesiredInstance() {
        if (desiredInstance == null) {
            desiredInstance = new RobotState();
        }
        return desiredInstance;
    }

    public static RobotState getGoalInstance() {
        if (goalInstance == null) {
            goalInstance = new RobotState();
        }
        return goalInstance;
    }

    public Distance getElevatorPosition() {
        return elevatorPosition;
    }

    public void updateElevatorPosition(Distance position) {
        elevatorPosition.mut_replace(position);
    }

    public Angle getPivotPosition() {
        return pivotPosition;
    }
    
    public void updatePivotAngle(Angle position) {
        pivotPosition.mut_replace(position);
    }

    public AngularVelocity getIntakeVelocity(){
        return intakeVelocity;
    }

    public Current getIntakeSupplyCurrent(){
        return intakeSupplyCurrent;
    }
   
    public Current getIntakeTorqueCurrent(){
        return intakeTorqueCurrent;
    }
}
