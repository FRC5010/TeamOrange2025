// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

/** A Unit aware 2D pose JSON object */
public class Pose2dJson {
  public UnitValueJson x = new UnitValueJson(0, "m");
  public UnitValueJson y = new UnitValueJson(0, "m");
  public UnitValueJson rotation = new UnitValueJson(0, "deg");
}
