package frc.robot.subsystems.pivot;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("LeaderMotorConnected", leaderMotorConnected);
    table.put("FollowerMotorConnected", followerMotorConnected);
    table.put("Position", position);
    table.put("Velocity", velocity);
    table.put("AppliedVoltsLeader", appliedVoltsLeader);
    table.put("AppliedVoltsFollower", appliedVoltsFollower);
    table.put("SupplyCurrentLeader", supplyCurrentLeader);
    table.put("SupplyCurrentFollower", supplyCurrentFollower);
    table.put("TorqueCurrentLeader", torqueCurrentLeader);
    table.put("TorqueCurrentFollower", torqueCurrentFollower);
    table.put("TemperatureLeader", temperatureLeader);
    table.put("TemperatureFollower", temperatureFollower);
    table.put("SetpointPosition", setpointPosition);
    table.put("SetpointVelocity", setpointVelocity);
  }

  @Override
  public void fromLog(LogTable table) {
    leaderMotorConnected = table.get("LeaderMotorConnected", leaderMotorConnected);
    followerMotorConnected = table.get("FollowerMotorConnected", followerMotorConnected);
    position = table.get("Position", position);
    velocity = table.get("Velocity", velocity);
    appliedVoltsLeader = table.get("AppliedVoltsLeader", appliedVoltsLeader);
    appliedVoltsFollower = table.get("AppliedVoltsFollower", appliedVoltsFollower);
    supplyCurrentLeader = table.get("SupplyCurrentLeader", supplyCurrentLeader);
    supplyCurrentFollower = table.get("SupplyCurrentFollower", supplyCurrentFollower);
    torqueCurrentLeader = table.get("TorqueCurrentLeader", torqueCurrentLeader);
    torqueCurrentFollower = table.get("TorqueCurrentFollower", torqueCurrentFollower);
    temperatureLeader = table.get("TemperatureLeader", temperatureLeader);
    temperatureFollower = table.get("TemperatureFollower", temperatureFollower);
    setpointPosition = table.get("SetpointPosition", setpointPosition);
    setpointVelocity = table.get("SetpointVelocity", setpointVelocity);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.leaderMotorConnected = this.leaderMotorConnected;
    copy.followerMotorConnected = this.followerMotorConnected;
    copy.position = this.position;
    copy.velocity = this.velocity;
    copy.appliedVoltsLeader = this.appliedVoltsLeader;
    copy.appliedVoltsFollower = this.appliedVoltsFollower;
    copy.supplyCurrentLeader = this.supplyCurrentLeader;
    copy.supplyCurrentFollower = this.supplyCurrentFollower;
    copy.torqueCurrentLeader = this.torqueCurrentLeader;
    copy.torqueCurrentFollower = this.torqueCurrentFollower;
    copy.temperatureLeader = this.temperatureLeader;
    copy.temperatureFollower = this.temperatureFollower;
    copy.setpointPosition = this.setpointPosition;
    copy.setpointVelocity = this.setpointVelocity;
    return copy;
  }
}
