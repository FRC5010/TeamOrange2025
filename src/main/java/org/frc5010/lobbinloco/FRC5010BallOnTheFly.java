// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.lobbinloco;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;

/** Add your docs here. */
public class FRC5010BallOnTheFly extends GamePieceProjectile {
  private static Runnable hitBoxCallBack = () -> System.out.println("hit target!");

  public static void setHitBoxCallBack(Runnable callBack) {
    hitBoxCallBack = callBack;
  }

  public FRC5010BallOnTheFly(
      Translation2d robotPosition,
      Translation2d shooterPositionOnRobot,
      ChassisSpeeds chassisSpeeds,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchingSpeed,
      Angle shooterAngle) {
    super(
        LobbinLoco.LOBBINLOCO_BALL_INFO,
        robotPosition,
        shooterPositionOnRobot,
        chassisSpeeds,
        shooterFacing,
        initialHeight,
        launchingSpeed,
        shooterAngle);
    super.withTouchGroundHeight(0.8);
    super.enableBecomesGamePieceOnFieldAfterTouchGround();
    super.withTargetTolerance(
        new Translation3d(LobbinLoco.GOAL_LENGTH, LobbinLoco.GOAL_WIDTH, LobbinLoco.BALL_HEIGHT)
            .div(2.0));
    super.withHitTargetCallBack(hitBoxCallBack);
  }
}
