package frc.robot;

import java.lang.annotation.ElementType;
import java.util.EnumMap;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    public boolean leftSide;

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
       
        leftSide = true;

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
        /* 
        NamedCommands.registerCommand("Shoot L4", ShootCoral(ElevatorStop.L4));
        NamedCommands.registerCommand("Shoot L3", ShootCoral(ElevatorStop.L3));
        NamedCommands.registerCommand("Shoot L2", ShootCoral(ElevatorStop.L2));
        NamedCommands.registerCommand("Shoot L1", ShootCoral(ElevatorStop.L1));
        NamedCommands.registerCommand("Go to intake", feed());
        NamedCommands.registerCommand("Intake In", intake.setIntakeSpeed(-0.4));
        NamedCommands.registerCommand("Intake Out", intake.setIntakeSpeed(0.4));
```
        NamedCommands.registerCommand("Pivot Up", pivot.pivotTo(Pivots.Up));
        NamedCommands.registerCommand("Pivot Down", pivot.pivotTo(Pivots.Down));
        NamedCommands.registerCommand("Pivot Shoot", pivot.pivotTo(Pivots.Shoot));
        NamedCommands.registerCommand("Pivot Intake", pivot.pivotTo(Pivots.Intake));
        */
    }


    private void configureButtonBindings() {
        /* Driver Buttons */

        driver.povUp().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));        
        driver.povDown().onTrue(s_Swerve.resetModulesToAbsolute());

       /*  driver.a().onTrue(new InstantCommand(() -> setStop(ElevatorStop.L1)));
        driver.x().onTrue(new InstantCommand(() -> setStop(ElevatorStop.L2)));
        driver.y().onTrue(new InstantCommand(() -> setStop(ElevatorStop.L3)));
        driver.b().onTrue(new InstantCommand(() -> setStop(ElevatorStop.L4))); */

        driver.a().onTrue(align(ElevatorStop.L1));
        driver.x().onTrue(align(ElevatorStop.L2));
        driver.y().onTrue(align(ElevatorStop.L3));
        driver.b().onTrue(align(ElevatorStop.L4));

        driver.start().onTrue(feed());

        driver.leftBumper().onTrue(new InstantCommand(() -> setSide(true)));
        driver.rightBumper().onTrue(new InstantCommand(() -> setSide(false)));
        Trigger coralSensed = new Trigger(() -> intake.hasCoral());

        coralSensed.whileTrue((new InstantCommand(() -> m_led.setColor(Color.kWhite)))
            .andThen(new InstantCommand(()-> m_led.startBlinking())));

    }

    /*  
     * Top level commands to chain common operations
     * these are useful to bind to a button and can be
     * reused as autobuilder commands so changes are made
     * in 1 spot
     */

    private Command alignReef(ReefFace face, boolean left, ElevatorStop stop) {

        return Commands.sequence(
            pivot.pivotTo(Pivots.Down),
            new WaitCommand(0.5),
            Commands.either(
                Commands.sequence( // score
                    //new LocalSwerve(s_Swerve, left ? face.approachLeft : face.approachRight, false),
                    elevators.moveTo(stop),
                    new WaitCommand(1.5)
                    //new LocalSwerve(s_Swerve, left ? face.alignLeft : face.alignRight, true)
                ),
                Commands.sequence( // remove algae
                    //new LocalSwerve(s_Swerve, face.approachMiddle, false),
                    //new LocalSwerve(s_Swerve, face.alignMiddle, true),
                    elevators.moveTo(ElevatorStop.L4)
                ),
                intake::hasCoral
            ),
            ShootCoral()
        );
    }

    
    private Command align(ElevatorStop stop){
        return alignReef(Swerve.nearestFace(s_Swerve.getPose().getTranslation()), leftSide, stop);
    }

    private Command alignLeft(ElevatorStop stop){
        return alignReef(Swerve.nearestFace(s_Swerve.getPose().getTranslation()), true, stop);
    }

    private Command alignRight(ElevatorStop stop){
        return alignReef(Swerve.nearestFace(s_Swerve.getPose().getTranslation()), false, stop);
    }



    // feed - get to feeder station with pivot and elevator in place, spin up intake when close, and wait for coral sensor, stop intake and pivot to shoot
    private Command feed() {
        return elevators.moveTo(ElevatorStop.INTAKE)
            .andThen(new WaitCommand(1.5))
            .andThen(pivot.pivotTo(Pivots.Intake))
            .andThen(intake.setIntakeSpeed(-0.2));
    }

    // scoreCoral - aligns, elevates, ensure proper position, outtake, waits for empty, stop intake, pivot up, lowers to safe, pivot to feed 

    private Command ShootCoral(){
        return pivot.pivotTo(Pivots.Shoot)
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(() -> m_led.setColor(Color.kPurple)))
            .andThen(intake.setIntakeSpeed(0.2))
            .andThen(new WaitCommand(0.5))
            .andThen(intake.setIntakeSpeed(0.0))
            .andThen(pivot.pivotTo(Pivots.Up))
            .andThen(new WaitCommand(0.5))
            .andThen(feed()); 
   
    }

    // pullAlgae - aligns, elevates, turns on intake for time period since algae wont hit sensor, reverses bot some
    private Command pullAlgae(ReefFace face){
        // Check the map to see if the algae is L2 or L3
        ElevatorStop algaeHeight = face.algaeHigh ? ElevatorStop.L3_ALGAE : ElevatorStop.L2_ALGAE;

        return new LocalSwerve(s_Swerve, face.approachMiddle, true);
    }

    // scoreBarge - elevates to max, move forward?, reverse intake, back up?, lower elevator, pivot to feed
    private Command scoreBarge() {
        return new InstantCommand(() -> m_led.setColors(Color.kBlue, Color.kGreen));
    }

    // Setup basic last foot options
    private void setReefCommands(ReefFace face) {
        pullAlgaeLeftCommands.put(face, pullAlgae(face));
        pullAlgaeRightCommands.put(face, pullAlgae(face));
    }

    public void setSide(boolean left){
        leftSide = left;
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
