package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

import static edu.wpi.first.units.Units.*;

public class ElevatorVisualizer {
    private final String key;
    private final Mechanism2d panel;
    private final MechanismRoot2d root;
    private final MechanismLigament2d elevator;

    private static final double ElevatorXModifier = -Math.cos(Degrees.of(90).in(Radians));
    private static final double ElevatorZModifier = Math.sin(Degrees.of(90).in(Radians));

    public ElevatorVisualizer(String key, Color color) {
        this.key = key;
        this.panel = new Mechanism2d(Inches.of(100).in(Meters), Inches.of(100).in(Meters), new Color8Bit(Color.kWhite));
        this.root = panel.getRoot("elevator", Inches.of(7.35).in(Meters), Inches.of(10).in(Meters));
        this.elevator = root.append(
            new MechanismLigament2d(
                "Elevator",
                Inches.of(0).in(Meters),
                90,
                10,
                new Color8Bit(color)
            )
        );
        //BROKEN// Logger.recordOutput("Elevator/Mechanism2d/" + key, this.panel);
    }

    public void update(Distance position) {
        elevator.setLength(position.in(Meters));
        //BROKEN// Logger.recordOutput("Elevator/Mechanism2d/" + key, this.panel);

        Distance elevatorX = position.times(ElevatorXModifier);
        Distance elevatorZ = position.times(ElevatorZModifier);
        Pose3d elevator3d = new Pose3d(elevatorX, Inches.zero(), elevatorZ, new Rotation3d());
        Logger.recordOutput("Elevator/Mechanism3d/" + key, elevator3d);
    }
  }
