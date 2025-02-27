package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.emu.Button.*;
import static frc.robot.utils.emu.Direction.*;
import static frc.robot.utils.emu.ElevatorState.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.sequencing.*;
import frc.robot.commands.sequencing.NewAutos.AutomaticScoreAuto;
import frc.robot.subsystems.*;
import frc.robot.utils.emu.*;
import frc.robot.utils.pingu.*;
import java.util.EnumMap;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Map<Button, JoystickButton> aacrnButtons = new EnumMap<>(Button.class);
  private final Map<Button, JoystickButton> calamityCowButtons = new EnumMap<>(Button.class);

  public final SendableChooser<Command> networkChooser;
  public final XboxController aacrn;
  public final XboxController calamityCow;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    aacrn = new XboxController(0);
    calamityCow = new XboxController(1);

    Elevator.getInstance().setDefaultCommand(padElevator(aacrn, calamityCow));
    Coral.getInstance();
    Swerve.getInstance().setDefaultCommand(drive(aacrn));
    Algae.getInstance();
    Button.getEntries()
        .forEach(
            button ->
                aacrnButtons.put(button, new JoystickButton(aacrn, button.getButtonNumber())));

    Button.getEntries()
        .forEach(
            button ->
                calamityCowButtons.put(
                    button, new JoystickButton(calamityCow, button.getButtonNumber())));

    NamedCommands.registerCommand("ScoreL1", new ScoreL1());
    NamedCommands.registerCommand("ScoreL2", new ScoreL2());
    NamedCommands.registerCommand("ScoreL3", new ScoreL3());
    NamedCommands.registerCommand("ScoreL4Left", new AutomaticScoreAuto(LEFT, L4));
    NamedCommands.registerCommand("ScoreL4Right", new AutomaticScoreAuto(RIGHT, L4));
    NamedCommands.registerCommand("HasPieceFalse", hasPieceFalse());

    // TODO add autoalign for auto

    NamedCommands.registerCommand("AlignLeft", new AlignToPose(LEFT).withTimeout(1.3));
    NamedCommands.registerCommand("AlignRight", new AlignToPose(RIGHT).withTimeout(1.3));

    // jayden did this 2/27 not tested

    NamedCommands.registerCommand("ScoreCoralLeft", new ScoreCoralAuto(LEFT).withTimeout(1.3));
    NamedCommands.registerCommand("ScoreCoralRight", new ScoreCoralAuto(RIGHT).withTimeout(1.3));

    NamedCommands.registerCommand("MoveElevatorL4Auto", moveElevatorState(L4));


    networkChooser = AutoBuilder.buildAutoChooser();

    configureBindings();

    new CommandPingu()
        .bind("scoreLeft", score(LEFT))
        .bind("scoreRight", score(RIGHT))
        .bind("SetL1", setElevatorState(L1))
        .bind("SetL2", setElevatorState(L2))
        .bind("SetL3", setElevatorState(L3))
        .bind("SetL4", setElevatorState(L4));

    //    networkChooser.addDefaultOption("Straight Auto", new PathPlannerAuto("Straight Auto"));
    //    networkChooser.addOption("Straight Auto", new InstantCommand());
  }

  /**
   * Use this method to define your trigger -> command mappings. Triggers can be created via the
   * {@link JoystickButton} constructor with an arbitrary predicate, or via the named factories in
   * {@link CommandGenericHID}'s subclasses for {@link CommandXboxController}/{@link
   * CommandPS4Controller} controllers or {@link CommandJoystick}.
   */
  private void configureBindings() {
    new Bingu(aacrnButtons)
        .bind(START, resetPidgey())
        //        .bind(B, setElevatorState(DEFAULT))
        //        .bind(B, align(CENTER).onlyWhile(pad::getAButton))
        .bind(B, new ResetScore())
        //        .bind(B, createPathfindingCmd(reefs.get(0)))
        .bind(A, setIntakeAlgae())
        //        .bind(A, align(RIGHT))
        .bind(Y, startCoralMotors())
        .bind(LEFT_BUMPER, score(LEFT))
        .bind(RIGHT_BUMPER, score(RIGHT))
        .bind(X, reverseIntake().onlyWhile(aacrn::getXButton));

    new Bingu(calamityCowButtons).bind(X, toggleVisionKillSwitch());
  }
}
