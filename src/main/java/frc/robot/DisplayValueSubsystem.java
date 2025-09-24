// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.telemetry.DisplayAngle;
import org.frc5010.common.telemetry.DisplayBoolean;
import org.frc5010.common.telemetry.DisplayDouble;
import org.frc5010.common.telemetry.DisplayFloat;
import org.frc5010.common.telemetry.DisplayLength;
import org.frc5010.common.telemetry.DisplayLong;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayTime;

/** Tests the classes in the {@link org.frc5010.common.telemetry} package that Display values */
public class DisplayValueSubsystem extends GenericSubsystem {
  DisplayAngle inputAngle;
  DisplayAngle outputAngle;
  DisplayBoolean inputBoolean;
  DisplayBoolean outputBoolean;
  DisplayDouble inputDouble;
  DisplayDouble outputDouble;
  DisplayFloat inputFloat;
  DisplayFloat outputFloat;
  DisplayLength inputLength;
  DisplayLength outputLength;
  DisplayLong inputLong;
  DisplayLong outputLong;
  DisplayString inputString;
  DisplayString outputString;
  DisplayTime inputTime;
  DisplayTime outputTime;
  DisplayAngle outAngle;

  public DisplayValueSubsystem() {
    super();
    outAngle = DashBoard.makeInfoAngle("Out Angle");
    outputAngle = DashBoard.makeDisplayAngle("OUTPUT_ANGLE");
    outputBoolean = DashBoard.makeDisplayBoolean("OUTPUT_BOOLEAN");
    outputDouble = DashBoard.makeDisplayDouble("OUTPUT_DOUBLE");
    outputFloat = DashBoard.makeDisplayFloat("OUTPUT_FLOAT");
    DashBoard.nextColumn("Config");
    inputAngle = DashBoard.makeConfigAngle("INPUT_ANGLE");
    inputBoolean = DashBoard.makeConfigBoolean("INPUT_BOOLEAN");
    inputDouble = DashBoard.makeConfigDouble("INPUT_DOUBLE");
    inputFloat = DashBoard.makeConfigFloat("INPUT_FLOAT");
    DashBoard.nextColumn("Input");
    inputLength = DashBoard.makeConfigLength("INPUT_LENGTH");
    inputLong = DashBoard.makeConfigLong("INPUT_LONG");
    inputString = DashBoard.makeConfigString("INPUT_STRING");
    inputTime = DashBoard.makeConfigTime("INPUT_TIME");
    DashBoard.nextColumn("Debug-Info");
    outputLength = DashBoard.makeInfoLength("OUTPUT_LENGTH");
    outputLong = DashBoard.makeInfoLong("OUTPUT_LONG");
    outputString = DashBoard.makeInfoString("OUTPUT_STRING");
    outputTime = DashBoard.makeInfoTime("OUTPUT_TIME");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    outputAngle.setAngle(inputAngle);
    outputBoolean.setValue(inputBoolean.getValue());
    outputDouble.setValue(inputDouble.getValue());
    outputFloat.setValue(inputFloat.getValue());
    outputLength.setLength(inputLength);
    outputLong.setValue(inputLong.getValue());
    outputString.setValue(inputString.getValue());
    outputTime.setTime(inputTime);
    outAngle.setAngle(Degrees.of(Math.random() * 360.0));
  }
}
