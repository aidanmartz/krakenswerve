package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class limelight extends SubsystemBase{

    // Setup basics objects for the limelight
    private final NetworkTable m_lime;
    private final String m_name;
    private final ArrayList<double[]> m_poses = new ArrayList<double[]>();

    public limelight(String name) {
        m_lime = NetworkTableInstance.getDefault().getTable(name);
        m_lime.getEntry("ledmode").setDouble(3.0);
        m_name = name;
    }

    // Override some methods for using the limelight
    @Override
    public void periodic() {
        SmartDashboard.putNumber("LL LED Mode", m_lime.getEntry("ledmode").getDouble(-1));
        SmartDashboard.putNumber("LL Pipeline", getPipeline());

    }

    // poses
    public void storePose(double[] pose) {
        if (pose != new double[7]) {
            m_poses.add(pose);
        }
    }

    // Deal with the pipeline indexes
    public void setPipeline(int pipelineIndex) {
        m_lime.getEntry("pipeline").setDouble((double) pipelineIndex);
    }

    public double getPipeline() {
        return (double) m_lime.getEntry("getpipe").getDouble(-1.0);
    }

    // Return ll name
    public String getName() {
        return m_name;
    }

    public boolean getTVVal() {
        return LimelightHelpers.getTV(m_name);
    }
    // Update LED setting
    public void setLights(int status) {
        m_lime.getEntry("ledmode").setNumber(status);
    }
//LED on
    public void ledOn() {
        m_lime.getEntry("ledmode").setNumber(3.0);
    }

    //LED off
    public void ledOff() {
        m_lime.getEntry("ledMode").setNumber(1.0);
    }

    //take snapshot
    public void takeSnap() {
        m_lime.getEntry("snapshot").setNumber(1.0);
    }
}
