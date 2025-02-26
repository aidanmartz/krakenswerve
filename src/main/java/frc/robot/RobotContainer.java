package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.ledSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.Elevator.ElevatorStop; // enum of stops

import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.Pivots;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;   

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Auto */
    private final SendableChooser<Command> autoChooser;

    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final POVButton dPadUp = new POVButton(driver, 0); // zero Gyro
    private final POVButton dPadRight = new POVButton(driver, 45); // robot centric
    private final POVButton dPadDown = new POVButton(driver, 180); // reset to Absolute
    private final JoystickButton rightStick = new JoystickButton(driver, XboxController.Button.kRightStick.value);

    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton startButton = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton backButton = new JoystickButton(driver, XboxController.Button.kBack.value);

    //private final JoystickButton leftStick = new JoystickButton(driver, XboxController.Button.kLeftStick.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator elevators;
    private final Pivot pivot;
    private final ledSubsystem m_led = new ledSubsystem();
   // private final Pivot pivot = new Pivot();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Robot.isReal()) {
            this.elevators = new Elevator(new ElevatorIOReal());
            this.pivot = new Pivot(new PivotIOReal());
        } else {
            this.elevators = new Elevator(new ElevatorIOSim());
            this.pivot = new Pivot(new PivotIOSim());
        }
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> dPadRight.getAsBoolean(), () -> rightStick.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        dPadUp.onTrue(s_Swerve.zeroHeading());
        aButton.whileTrue(new InstantCommand(() -> m_led.setRGB(255, 255, 255)));
        
        dPadDown.onTrue(s_Swerve.resetModulesToAbsolute());

        aButton.whileTrue(elevators.moveTo(ElevatorStop.L1));
       // .andThen(elevators.pivotTo(Pivots.Shoot)));
        xButton.whileTrue(elevators.moveTo(ElevatorStop.L2));
       // .andThen(elevators.pivotTo(Pivots.Shoot)));
        yButton.whileTrue(elevators.moveTo(ElevatorStop.L3));
      //  .andThen(elevators.pivotTo(Pivots.Shoot)));
        bButton.whileTrue(elevators.moveTo(ElevatorStop.L4));
       // .andThen(elevators.pivotTo(Pivots.Shoot)));
        leftBumper.whileTrue(elevators.moveTo(ElevatorStop.SAFE));
       // .andThen(elevators.pivotTo(Pivots.Intake)));

       startButton.whileTrue(pivot.pivotTo(Pivots.Intake));
       backButton.whileTrue(pivot.pivotTo(Pivots.Shoot));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
