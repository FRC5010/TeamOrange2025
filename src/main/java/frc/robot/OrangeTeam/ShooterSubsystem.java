// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OrangeTeam;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import org.frc5010.common.arch.GenericSubsystem;
import yams.mechanisms.config.ShooterConfig;
import yams.mechanisms.velocity.Shooter;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class ShooterSubsystem extends GenericSubsystem {
  private final ThriftyNova motor = new ThriftyNova(10);
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withClosedLoopController(
              4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          .withGearing(gearing(gearbox(3, 4)))
          .withIdleMode(MotorMode.COAST)
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motorController =
      new NovaWrapper(motor, DCMotor.getNEO(1), motorConfig);

  private final ShooterConfig shooterConfig =
      new ShooterConfig(motorController)
          .withDiameter(Inches.of(4))
          .withMass(Pounds.of(1))
          .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
          .withUpperSoftLimit(RPM.of(5000));

  private final Shooter shooter = new Shooter(shooterConfig);

  /** Creates a new Shooter. */
  public ShooterSubsystem() {}

  public Command setSpeed(double speed) {
    return shooter.set(speed);
  }

  @Override
  public void periodic() {
    shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
