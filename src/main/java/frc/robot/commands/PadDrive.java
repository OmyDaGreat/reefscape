package frc.robot.commands;

import static frc.robot.utils.Dash.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds;

/** Command to control the robot's swerve drive using a Logitech gaming pad. */
public class PadDrive extends Command {
  private final GamingController pad;
  private final boolean isFieldOriented;

  /**
   * Constructs a new PadDrive command.
   *
   * @param pad The Logitech gaming pad used to control the robot.
   * @param isFieldOriented Whether the drive is field-oriented.
   */
  public PadDrive(GamingController pad, boolean isFieldOriented) {
    this.pad = pad;
    this.isFieldOriented = isFieldOriented;
    addRequirements(Swerve.getInstance());
  }

  /**
   * Called every time the scheduler runs while the command is scheduled. This method retrieves the
   * current position from the gaming pad, calculates the rotation, logs the joystick values, and
   * sets the drive speeds for the swerve subsystem.
   */
  @Override
  public void execute() {
    Coordinate position = positionSet(pad);

    double rotation =
        (Math.abs(pad.getRightAnalogXAxis()) >= 0.2)
            ? -pad.getRightAnalogXAxis() * RobotParameters.MotorParameters.MAX_ANGULAR_SPEED
            : 0.0;

    log("X Joystick", position.x());
    log("Y Joystick", position.y());
    log("Rotation", rotation);

    Swerve.getInstance()
        .setDriveSpeeds(position.y(), position.x(), rotation * 0.8, isFieldOriented);
  }

  /**
   * Returns true when the command should end.
   *
   * @return Always returns false, as this command never ends on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /** Record representing a coordinate with x and y values. */
  public record Coordinate(double x, double y) {}

  /**
   * Sets the position based on the input from the Logitech gaming pad.
   *
   * @param pad The Logitech gaming pad.
   * @return The coordinate representing the position.
   */
  public static Coordinate positionSet(GamingController pad) {
    double x = -pad.getLeftAnalogXAxis() * RobotParameters.MotorParameters.MAX_SPEED;
    if (Math.abs(x) < Thresholds.X_DEADZONE) {
      x = 0.0;
    }

    double y = -pad.getLeftAnalogYAxis() * RobotParameters.MotorParameters.MAX_SPEED;
    if (Math.abs(y) < Thresholds.Y_DEADZONE) {
      y = 0.0;
    }

    return new Coordinate(x, y);
  }
}
