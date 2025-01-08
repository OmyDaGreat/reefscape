// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * AlignScoreLeft is a command group that aligns the robot with the left side of
 * the reef to score coral on the left branch.
 */
public class AlignScoreLeft extends SequentialCommandGroup {
  /** Creates a new AlignScoreLeft. */
  public AlignScoreLeft() {
    /**
     * TASKS:
     * 1. Align up with april tag
     * 2. Offset to the left of the april tag, lining up with L4 branch placement
     * 3. Move up elevator to score
     * 4. Pivot to branch
     * 5. Reverse rollers to score
     * 6. Do all that in reverse
     */

    addCommands();
  }
}