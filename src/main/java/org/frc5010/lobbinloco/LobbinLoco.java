// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.lobbinloco;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;

/** Add your docs here. */
public class LobbinLoco extends SimulatedArena {
  public static final double FIELD_LENGTH = Units.Feet.of(16).in(Units.Meters);
  public static final double FIELD_OVERALL_LENGTH = Units.Feet.of(50).in(Units.Meters);
  public static final double FIELD_WIDTH = Units.Feet.of(20).in(Units.Meters);
  public static final double GOAL_LENGTH = Inches.of(36).in(Meters);
  public static final double GOAL_WIDTH = Inches.of(36).in(Meters);
  public static final double GOAL_HEIGHT = Inches.of(8).in(Meters);
  public static final double BALL_HEIGHT = Inches.of(5).in(Meters);

  // Dimensions in meters (50 feet x 20 feet)
  private static final Translation2d bottomLeft = new Translation2d(0.0, 0.0);
  private static final Translation2d bottomRight = new Translation2d(FIELD_LENGTH, 0);
  private static final Translation2d bottomRightOverall =
      new Translation2d(FIELD_OVERALL_LENGTH, 0);
  private static final Translation2d topLeft = new Translation2d(0.0, FIELD_WIDTH);
  private static final Translation2d topRight = new Translation2d(FIELD_LENGTH, FIELD_WIDTH);
  private static final Translation2d topRightOverall =
      new Translation2d(FIELD_OVERALL_LENGTH, FIELD_WIDTH);

  public static final GamePieceInfo LOBBINLOCO_BALL_INFO =
      new GamePieceInfo(
          "FRC5010Ball",
          new Circle(Inches.of(5).in(Meters)),
          Inches.of(5),
          Kilograms.of(0.1),
          1.8,
          5,
          0.8);

  private static class LobbinLoboFieldMap extends FieldMap {
    public LobbinLoboFieldMap() {
      addBorderLine(bottomLeft, bottomRight); // _
      addBorderLine(bottomRight, bottomRightOverall); // __
      addBorderLine(bottomRight, topRight); // _|_
      addBorderLine(bottomRightOverall, topRightOverall); // _|_|
      addBorderLine(topRight, topRightOverall); //
      addBorderLine(topRight, topLeft);
      addBorderLine(topLeft, bottomLeft);

      defineGoal(288, 30); // 18
      defineGoal(288, 170); // 17
      defineGoal(432, 80); // 20
      defineGoal(432, 120); // 19
      defineGoal(528, 10); // 22
      defineGoal(528, 190); // 21
    }

    private void defineGoal(double x, double y) {
      double xInMeters = Inches.of(x).in(Meters);
      double yInMeters = Inches.of(y).in(Meters);
      // Goal Right Wall
      // this.addBorderLine(
      // new Translation2d(xInMeters, yInMeters),
      // new Translation2d(xInMeters + GOAL_LENGTH, yInMeters));
      this.addRectangularObstacle(
          GOAL_LENGTH, GOAL_HEIGHT, new Pose2d(xInMeters, yInMeters, new Rotation2d()));
      // Goal Front Wall
      // this.addBorderLine(
      // new Translation2d(xInMeters, yInMeters),
      // new Translation2d(xInMeters, yInMeters + GOAL_WIDTH));
      this.addRectangularObstacle(
          GOAL_LENGTH, GOAL_HEIGHT, new Pose2d(xInMeters, yInMeters, new Rotation2d(90.0)));
      // Goal Left Wall
      // this.addBorderLine(
      // new Translation2d(xInMeters, yInMeters + GOAL_WIDTH),
      // new Translation2d(xInMeters + GOAL_LENGTH, yInMeters + GOAL_WIDTH));
      this.addRectangularObstacle(
          GOAL_LENGTH,
          GOAL_HEIGHT,
          new Pose2d(xInMeters, yInMeters + GOAL_WIDTH, new Rotation2d()));
      // Goal Back Wall
      // this.addBorderLine(
      // new Translation2d(xInMeters + GOAL_LENGTH, yInMeters),
      // new Translation2d(xInMeters + GOAL_LENGTH, yInMeters + GOAL_WIDTH));
      this.addRectangularObstacle(
          GOAL_LENGTH,
          GOAL_HEIGHT,
          new Pose2d(xInMeters + GOAL_LENGTH, yInMeters + GOAL_WIDTH, new Rotation2d(-90.0)));
    }
  }

  private static final LobbinLoboFieldMap fieldMap = new LobbinLoboFieldMap();

  public LobbinLoco() {
    super(fieldMap);
  }

  /**
   * Places game pieces on the field according to the current game configuration. This method is
   * called by the SimulatedArena class when the game starts. The method is responsible for placing
   * the game pieces in the correct positions on the simulated field.
   */
  @Override
  public void placeGamePiecesOnField() {
    // no pieces on the field to start
  }
}
