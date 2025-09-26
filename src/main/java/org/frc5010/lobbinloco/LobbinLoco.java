// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.lobbinloco;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import org.ironmaple.simulation.SimulatedArena;

/** Add your docs here. */
public class LobbinLoco extends SimulatedArena {
  // Dimensions in meters (50 feet x 20 feet)
  private static final Translation2d bottomLeft = new Translation2d(0.0, 0.0);
  private static final Translation2d bottomRight =
      new Translation2d(Units.Feet.of(15).in(Units.Meters), 0);
  private static final Translation2d topLeft =
      new Translation2d(0.0, Units.Feet.of(20).in(Units.Meters));
  private static final Translation2d topRight =
      new Translation2d(Units.Feet.of(16).in(Units.Meters), Units.Feet.of(20).in(Units.Meters));

  private static class LobbinLoboFieldMap extends FieldMap {
    public LobbinLoboFieldMap() {
      this.addBorderLine(bottomLeft, bottomRight);
      this.addBorderLine(bottomRight, topRight);
      this.addBorderLine(topRight, topLeft);
      this.addBorderLine(topLeft, bottomLeft);
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
