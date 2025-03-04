package frc.robot;

import java.util.EnumMap;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;

import frc.robot.subsystems.ledSubsystem;
import frc.robot.subsystems.dontuse_Elevator.Stop;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.Elevator.ElevatorStop; // enum of stops
import frc.lib.util.LoggedCommands;
import frc.robot.Constants.Localization.ReefFace;
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

    EnumMap<ReefFace, Command> alignLeftCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> alignRightCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> pullAlgaeLeftCommands = new EnumMap<>(ReefFace.class);
    EnumMap<ReefFace, Command> pullAlgaeRightCommands = new EnumMap<>(ReefFace.class);

    /* Controllers */
    CommandXboxController driver = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translationAxis = driver::getLeftY;
    private final Supplier<Double> strafeAxis = driver::getLeftX;
    private final Supplier<Double> rotationAxis = driver::getRightX;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Elevator elevators;
    private final Intake intake;
    private final Pivot pivot;
    private final ledSubsystem m_led = new ledSubsystem();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        if (Robot.isReal()) {
            this.elevators = new Elevator(new ElevatorIOReal());
            this.pivot = new Pivot(new PivotIOReal());
            this.intake = new Intake(new IntakeIOReal());
        } else {
            this.elevators = new Elevator(new ElevatorIOSim());
            this.pivot = new Pivot(new PivotIOSim());
            this.intake = new Intake(new IntakeIOReal()); //TODO SIM
        }

        for (ReefFace face: ReefFace.values()) {
            setReefCommands(face);
        }
       
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -translationAxis.get(),
                        () -> -strafeAxis.get(),
                        () -> -rotationAxis.get(),
                        () -> driver.povUp().getAsBoolean(), 
                        () -> driver.rightStick().getAsBoolean()));

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


        driver.povUp().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        //driver.rightBumper().whileTrue(new InstantCommand(() -> m_led.setRGB(255, 255, 255)));
        
        driver.povDown().onTrue(s_Swerve.resetModulesToAbsolute());

        driver.a().onTrue(elevators.moveTo(ElevatorStop.L1)
            .andThen(new InstantCommand(() -> m_led.setColor(Color.kSkyBlue))));
       // .andThen(elevators.pivotTo(Pivots.Shoot)));
        driver.x().onTrue(elevators.moveTo(ElevatorStop.L2)
            .andThen(new InstantCommand(() -> m_led.setColor(Color.kBlueViolet))));
       // .andThen(elevators.pivotTo(Pivots.Shoot)));
        driver.y().onTrue(elevators.moveTo(ElevatorStop.L3)
            .andThen(new InstantCommand(() -> m_led.setColor(Color.kMediumPurple))));
      //  .andThen(elevators.pivotTo(Pivots.Shoot)));
        driver.b().onTrue(elevators.moveTo(ElevatorStop.L4)
            .andThen(new InstantCommand(() -> m_led.setColor(Color.kWhite))));
       // .andThen(elevators.pivotTo(Pivots.Shoot)));
        driver.leftBumper().whileTrue(
            Commands.either(
                Commands.select(alignLeftCommands, () -> Swerve.nearestFace(s_Swerve.getPose().getTranslation())),
                Commands.select(pullAlgaeLeftCommands, () -> Swerve.nearestFace(s_Swerve.getPose().getTranslation())),
                intake::hasCoral
            )
        );

        driver.rightBumper().whileTrue(
            Commands.either(
                Commands.select(alignRightCommands, () -> Swerve.nearestFace(s_Swerve.getPose().getTranslation())),
                Commands.select(pullAlgaeRightCommands, () -> Swerve.nearestFace(s_Swerve.getPose().getTranslation())),
                intake::hasCoral
            )
        );

       driver.start().whileTrue(pivot.pivotTo(Pivots.Intake));
       driver.back().whileTrue(pivot.pivotTo(Pivots.Down));
       driver.leftStick().whileTrue(intake.setIntakeSpeed(0.5));
       driver.leftStick().whileFalse(intake.setIntakeSpeed(0));

    }

    /*  
     * Top level commands to chain common operations
     * these are useful to bind to a button and can be
     * reused as autobuilder commands so changes are made
     * in 1 spot
     */

    
    // feed - get to feeder station with pivot and elevator in place, spin up intake when close, and wait for coral sensor, stop intake and pivot to shoot
    private Command feed(ReefFace face, boolean left) {
        return new InstantCommand(() -> m_led.setColor(Color.kCoral));
    }

    // scoreCoral - aligns, elevates, ensure proper position, outtake, waits for empty, stop intake, pivot up, lowers to safe, pivot to feed 
    private Command scoreCoral(ReefFace face, boolean left) {
        return new LocalSwerve(s_Swerve, left ? face.approachLeft : face.approachRight, true);
        //return new InstantCommand(() -> m_led.setColors(Color.kBlue, Color.kGreen));
    }

    // pullAlgae - aligns, elevates, turns on intake for time period since algae wont hit sensor, reverses bot some
    private Command pullAlgae(ReefFace face){
        // Check the map to see if the algae is L2 or L3
        Stop algaeHeight = face.algaeHigh ? Stop.L3_ALGAE : Stop.L2_ALGAE;
        return new LocalSwerve(s_Swerve, face.approachMiddle, true);

        //return new InstantCommand(() -> m_led.setColor(Color.kGreen));
    }

    // scoreBarge - elevates to max, move forward?, reverse intake, back up?, lower elevator, pivot to feed
    private Command scoreBarge() {
        return new InstantCommand(() -> m_led.setColors(Color.kBlue, Color.kGreen));
    }

    // Setup basic last foot options
    private void setReefCommands(ReefFace face) {
        alignLeftCommands.put(face, scoreCoral(face, true));
        alignRightCommands.put(face, scoreCoral(face, false));
        pullAlgaeLeftCommands.put(face, pullAlgae(face));
        pullAlgaeRightCommands.put(face, pullAlgae(face));
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
