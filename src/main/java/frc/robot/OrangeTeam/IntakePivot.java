// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OrangeTeam;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.system.plant.DCMotor;
import org.frc5010.common.arch.GenericSubsystem;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class IntakePivot extends GenericSubsystem {

  private final ThriftyNova motor = new ThriftyNova(12);

  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withGearing(gearing(gearbox(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withControlMode(ControlMode.OPEN_LOOP);
  private final SmartMotorController motorController =
      new NovaWrapper(motor, DCMotor.getNEO(1), motorConfig);
  /** Creates a new IntakePivot. */
  public IntakePivot() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
