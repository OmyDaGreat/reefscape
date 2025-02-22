package frc.robot.commands;

import static frc.robot.utils.RobotParameters.MotorParameters.MAX_ANGULAR_SPEED;
import static frc.robot.utils.RobotParameters.MotorParameters.MAX_SPEED;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.X_DEADZONE;
import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.Y_DEADZONE;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.utils.RobotParameters;
import frc.robot.utils.emu.*;
import kotlin.Pair;
import org.photonvision.PhotonCamera;

public class AlignSwerve extends Command {
  private double yaw;
  private double y;
  private double dist;
  private PhotonVision photonVision;
  private PIDController rotationalController;
  private PIDController yController;
  private PIDController disController;
  private Timer timer;
  private double
      offset; // double offset is the left/right offset from the april tag to make it properly align
  PhotonCamera camera;
    private XboxController pad;


  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignSwerve(Direction offsetSide, XboxController pad) {

    photonVision = PhotonVision.getInstance();
    this.pad = pad;
    switch (offsetSide) {
      case LEFT:
        camera = photonVision.requestCamera("RightCamera");
        offset = RobotParameters.PhotonVisionConstants.LEFT_OFFSET;
        break;
      case RIGHT:
        camera = photonVision.requestCamera("LeftCamera");
        offset = RobotParameters.PhotonVisionConstants.RIGHT_OFFSET;
        break;
      case CENTER:
        this.offset = 0;
        break;
      default:
        throw new IllegalStateException("Unexpected value: " + offsetSide);
    }

    addRequirements(Swerve.getInstance());
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  @Override
  public void initialize() {
    yaw = photonVision.getYaw();
    y = photonVision.getY();
    dist = photonVision.getDist();

    rotationalController = ROTATIONAL_PINGU.getPidController();
    rotationalController.setTolerance(0.4); // with L4 branches
    rotationalController.setSetpoint(0);

    yController = Y_PINGU.getPidController();
    yController.setTolerance(1.5);
    yController.setSetpoint(0);

    disController = DIST_PINGU.getPidController();
    disController.setTolerance(1.5);
    disController.setSetpoint(0);

    timer = new Timer();
    timer.reset();
    timer.start();
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.
   */
  @Override
  public void execute() {
    yaw = photonVision.fetchYaw(camera);

    y = photonVision.fetchY(camera);

    dist = photonVision.fetchDist(camera);

    double error = y - offset;

    double padRotation = Math.abs(pad.getRightX()) >= 0.1 ? -pad.getRightX() * MAX_ANGULAR_SPEED : 0.0;


    Swerve.getInstance()
        .setDriveSpeeds(0, yController.calculate(error), 0.0);

    if (photonVision.hasTag()) {
      timer.reset();
    }
  }

  /**
   * Sets the position based on the input from the Logitech gaming pad.
   *
   * @param pad The Logitech gaming pad.
   * @return The coordinate representing the position. The first element is the x-coordinate, and
   *     the second element is the y-coordinate.
   */
  public static Pair<Double, Double> positionSet(XboxController pad) {
      return PadDrive.positionSet(pad);
  }

  /**
   * Returns whether this command has finished. Once a command finishes -- indicated by this method
   * returning true -- the scheduler will call its {@link #end(boolean)} method.
   *
   * <p>Returning false will result in the command never ending automatically. It may still be
   * cancelled manually or interrupted by another command. Hard coding this command to always return
   * true will result in the command executing once and finishing immediately. It is recommended to
   * use * {@link InstantCommand InstantCommand} for such an operation.
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(1.0);
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally --
   * that is called when {@link #isFinished()} returns true -- or when it is interrupted/canceled.
   * This is where you may want to wrap up loose ends, like shutting off a motor that was being used
   * in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted) {
    Swerve.getInstance().stop();
  }
}
