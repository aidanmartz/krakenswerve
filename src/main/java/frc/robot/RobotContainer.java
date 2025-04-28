package frc.robot;

import java.util.EnumMap;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Localization.ReefFace;
import frc.robot.commands.LocalSwerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorStop; // enum of stops
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.led.LedSubsystem.LedMode;
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
    private final LedSubsystem m_led = new LedSubsystem();
    private final Field2d targetField;
    
    /* Alliance colors */
    private final Color redBumper = Color.kDarkRed;
    private final Color blueBumper = Color.kDarkBlue;
    private Color original_color;

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
            this.intake = new Intake(new IntakeIOReal()); 
        }


        NamedCommands.registerCommand("Intake On", intake.setIntakeSpeed(-0.3));
        NamedCommands.registerCommand("Pivot to Shoot", intake.setIntakeSpeed(-0.3).andThen(new WaitCommand(1.0)).andThen(pivot.pivotTo(Pivots.Shoot).andThen(colorCommand(Color.kRed))));
        NamedCommands.registerCommand("Elevator L4",
            Commands.sequence( 
                elevators.setNextStopCommand(ElevatorStop.L4),
                elevators.moveToNext()
            ));
        NamedCommands.registerCommand("Shoot", autoShootCoral().andThen(colorCommand(Color.kGreen)));
        NamedCommands.registerCommand("Feed", feed().until(intake::hasCoral).andThen(pivot.pivotTo(Pivots.Shoot)).andThen(colorCommand(Color.kOrange)));
        NamedCommands.registerCommand("FindCoral",
            (new InstantCommand(() -> LimelightHelpers.setLEDMode_ForceOn("limelight")))
            .andThen (new RunCommand(
                () -> s_Swerve.drive(
                    MathUtil.applyDeadband(driver.getLeftY(), 0.125),
                    MathUtil.applyDeadband(driver.getLeftX(), 0.125),
                    Translation2d.kZero,
                    MathUtil.applyDeadband(driver.getRightX(), 0.125),
                    true, false, true)))
                    .until(intake ::hasCoral)
                    .withTimeout(1.25)
            .andThen(new InstantCommand(()-> LimelightHelpers.setLEDMode_ForceOff("limelight")))
        );

        // set color at startup
        original_color = Robot.isRed() ? redBumper : blueBumper;
        m_led.setColor(original_color);

        targetField = new Field2d();
        SmartDashboard.putData("Target Field", targetField);

        //for (ReefFace face: ReefFace.values()) {
        //    setReefCommands(face);
        //}
       
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                    s_Swerve,
                    () -> -translationAxis.get(),
                    () -> -strafeAxis.get(),
                    () -> -rotationAxis.get(),
                    () -> false, 
                    () -> false
            )
        );

        // Configure the button bindings
        configureButtonBindings();

    }


    private void configureButtonBindings() {
        /* Driver Buttons */

        driver.povUp().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));        
        driver.povDown().onTrue(s_Swerve.resetModulesToAbsolute());

        driver.a().onTrue(elevators.setNextStopCommand(ElevatorStop.L1)
            .andThen(pivot.pivotToOnElevator(ElevatorStop.L1))
            .andThen(ledCommand(LedMode.SOLID, Color.kGreen, Color.kBlue)));
                //.andThen(ledCommand(LedMode.WAVE, Color.kGreen, Color.kBlue)));

        driver.x().onTrue(elevators.setNextStopCommand(ElevatorStop.L2)
            .andThen(pivot.pivotToOnElevator(ElevatorStop.L2))
            .andThen(ledCommand(LedMode.WAVE2, Color.kBlue, Color.kPink)));

        driver.y().onTrue(elevators.setNextStopCommand(ElevatorStop.L3)
            .andThen(pivot.pivotToOnElevator(ElevatorStop.L3))
            .andThen(ledCommand(LedMode.WATER, Color.kBlack, Color.kBlack)));

                //.andThen(ledCommand(LedMode.WAVE2, Color.kPurple, Color.kOrange)));

        driver.b().onTrue(elevators.setNextStopCommand(ElevatorStop.L4)
            .andThen(pivot.pivotToOnElevator(ElevatorStop.L4))
            .andThen(ledCommand(LedMode.FIRE, Color.kBlack, Color.kBlack)));

        driver.leftBumper().onTrue(elevators.moveToNext());
        driver.rightBumper().onTrue(intake.ejectCoralCmd());

        driver.leftTrigger().whileTrue(alignReef(true, elevators));
        driver.rightTrigger().whileTrue(alignReef(false,elevators));

        //driver.back().onTrue(pivot.pivotTo(Pivots.ShootL4));
        
        driver.back().onTrue(pivot.pivotToOnElevator(elevators.getNextStop()));

        driver.start().onTrue(feed());

        // Current sense the intake but make sure it is high for > 0.5s to reduce false triggers
        Trigger coralSensed = new Trigger(() -> intake.hasCoral()).debounce(0.5, DebounceType.kBoth);
  
        coralSensed.onTrue(
            colorCommand(Color.kCoral).andThen(pivot.pivotTo(Pivots.Shoot))
        );

    }

    /*  
     * Top level commands to chain common operations
     * these are useful to bind to a button and can be
     * reused as autobuilder commands so changes are made
     * in 1 spot
     */



    // feed - get to feeder station with pivot and elevator in place, spin up intake when close, and wait for coral sensor, stop intake and pivot to shoot
    private Command feed() {
        return elevators.moveToIntake()
            .andThen(new WaitCommand(1.5))
            .andThen(intake.setIntakeSpeed(-0.2));
    }

    // scoreCoral - aligns, elevates, ensure proper position, outtake, waits for empty, stop intake, pivot up, lowers to safe, pivot to feed 
    private Command shootCoral() {
        return (colorCommand(Color.kPurple))
            .andThen(intake.ejectCoralCmd())
            .andThen(new WaitCommand(0.5))
            .andThen(intake.stopCmd())
            .andThen(pivot.pivotTo(Pivots.Up))
            .andThen(new WaitCommand(1.5))
            .andThen(feed())
            .andThen(colorCommand(original_color));
    }

    public Command autoShootCoral(){
        return intake.ejectCoralCmd(elevators)
                 .andThen(new WaitCommand(1.0))
                 .andThen(pivot.pivotTo(Pivots.Up)); 
     }

    private Command colorCommand(Color acolor) {
        return new InstantCommand(() -> m_led.setColor(acolor));
    }

    private Command ledCommand(LedMode mode, Color primaryColor, Color secondaryColor) {
        return new InstantCommand( () -> {
            m_led.setMode(mode);
            m_led.setColor(primaryColor);
            m_led.setSecondaryColor(secondaryColor);
        }
        );
    }
    

    private Command moveToReef(boolean left, Elevator elevator) {
        return Commands.sequence(
            // elevator.moveToNext(),
            // new WaitCommand(0.5),
            Commands.runOnce(() -> {
                ReefFace currentFace = s_Swerve.goalFace; // Capture the current value
                new LocalSwerve(s_Swerve, left ? currentFace.alignLeft : currentFace.alignRight, true)
                        .withTimeout(5).schedule();
            }));
    }

    private Command alignReef(boolean left, Elevator elevator) {
        // If we have a coral align to left/right to shoot, if not, align to middle for
        // algae
        return Commands.either(
            moveToReef(left, elevator),
            pullAlgae(),
            intake::hasCoral
        );
    }
    private Command pullAlgae() {
        return Commands.sequence(
            colorCommand(Color.kCyan), // algae ball is cyan, yo.
            pivot.pivotTo(Pivots.ShootL1),
            new WaitCommand(0.5),
            Commands.runOnce(() -> {
                ReefFace currentFace = s_Swerve.goalFace; // Capture the current value
                elevators.setNextStopCommand(getAlgaeStop(currentFace)).schedule();
            }),
            elevators.moveToNext(),
            intake.ejectCoralCmd(elevators),
            Commands.runOnce(() -> {
                ReefFace currentFace = s_Swerve.goalFace; // Capture the current value
                new LocalSwerve(s_Swerve, currentFace.alignMiddle, true).withTimeout(5).schedule();
            })
        );

            // // // //colorCommand(Color.kCyan),
            // // // // if we are higher than L2_ALGAE then move elevator to L2.    
            // // // elevators.moveTo(belowHeight),
            // // // // then move using auto to middle of the reef
            // // // new LocalSwerve(s_Swerve, face.approachMiddle, true),
            // // // // then move pivot to shooting position
            // // // pivot.pivotTo(Pivots.Shoot),
            // // // // then run intake motor as if it's shooting
            // // // intake.setIntakeSpeed(0.40).withTimeout(0.5),
            // // // // then raise to correct algae height
            // // // elevators.moveTo(algaeHeight),
            // // // // then slow down or stop the intake
            // // // intake.setIntakeSpeed(0.20).withTimeout(0.1),
            // // // intake.setIntakeSpeed(0.05).withTimeout(0.1)
    }

    // pullAlgae - aligns, elevates, turns on intake for time period since algae wont hit sensor, reverses bot some
    private ElevatorStop getAlgaeStop(ReefFace face){
        // Check the map to see if the algae is L2 or L3
        ElevatorStop algaeHeight = face.algaeHigh ? ElevatorStop.L3_ALGAE : ElevatorStop.L2_ALGAE;
        return algaeHeight;
    }
    private ElevatorStop getAlgaeBelowStop(ReefFace face) {
        return face.algaeHigh ? ElevatorStop.L3 : ElevatorStop.L2;
    }

    // scoreBarge - elevates to max, move forward?, reverse intake, back up?, lower elevator, pivot to feed
    //private Command scoreBarge() {
    //    return new InstantCommand(() -> m_led.setColors(Color.kBlue, Color.kGreen));
    //}
    /* 
    private Command pullAlgae(ReefFace face) {
        // NOTE: Check the map to see if the algae is L2 or L3
        ElevatorStop algaeHeight = face.algaeHigh ? ElevatorStop.L3_ALGAE : ElevatorStop.L2_ALGAE;
        // want to get underneath the algae if we are not already so we arent smashing
        // down on it.
        // if we were previously at (e.g.) L1 then no harm in going to L2.
        ElevatorStop belowHeight = face.algaeHigh ? ElevatorStop.L3 : ElevatorStop.L2;

        return Commands.sequence(
            colorCommand(Color.kCyan),
            // if we are higher than L2_ALGAE then move elevator to L2.    
            elevators.moveTo(belowHeight),
            // then move using auto to middle of the reef
            new LocalSwerve(s_Swerve, face.approachMiddle, true),
            // then move pivot to shooting position
            pivot.pivotTo(Pivots.Shoot),
            // then run intake motor as if it's shooting
            intake.setIntakeSpeed(0.40).withTimeout(0.5),
            // then raise to correct algae height
            elevators.moveTo(algaeHeight),
            // then slow down or stop the intake
            intake.setIntakeSpeed(0.20).withTimeout(0.1),
            intake.setIntakeSpeed(0.05).withTimeout(0.1)
            // TODO: then reverse by some amount and let go.
            // Or will the driver let it go somewhere else?
        );

    }*/
    // Setup basic last foot options
    //private void setReefCommands(ReefFace face) {
    //    pullAlgaeLeftCommands.put(face, pullAlgae(face));
    //    pullAlgaeRightCommands.put(face, pullAlgae(face));
    //}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
