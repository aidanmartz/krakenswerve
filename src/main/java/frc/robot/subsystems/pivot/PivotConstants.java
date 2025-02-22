package frc.robot.subsystems.pivot;

public class PivotConstants {

    public static final PivotGains SimGains = new PivotGains(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final PivotGains TalonFXGains = new PivotGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    record PivotGains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        public PivotGains {
            if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0) {
                throw new IllegalArgumentException("Gains must be non-negative");
            }
        }
    }
}
