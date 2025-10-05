// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.lobbinloco;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

/** Add your docs here. */
public class FRC5010BallOnField extends GamePieceOnFieldSimulation {
  public FRC5010BallOnField() {
    super(LobbinLoco.LOBBINLOCO_BALL_INFO, new Pose2d(0, 0, new Rotation2d()));
  }

  public FRC5010BallOnField(Translation2d initialPosition) {
    super(LobbinLoco.LOBBINLOCO_BALL_INFO, new Pose2d(initialPosition, new Rotation2d()));
  }
}
