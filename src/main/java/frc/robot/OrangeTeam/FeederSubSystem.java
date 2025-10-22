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

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.frc5010.common.arch.GenericSubsystem;

import com.thethriftybot.ThriftyNova;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import yams.mechanisms.config.ShooterConfig;
import yams.mechanisms.velocity.Shooter;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class FeederSubSystem extends GenericSubsystem {
  private final ThriftyNova motor = new ThriftyNova(11);
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

  private final ShooterConfig feederConfig = new ShooterConfig(motorController)
    .withDiameter(Inches.of(4))
    .withMass(Pounds.of(1))
    .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
    .withUpperSoftLimit(RPM.of(5000));

  private final Shooter feeder = new Shooter(feederConfig);

  public Command setSpeed(double speed) {
     return feeder.set(speed);
   }
  /** Creates a new FeederSubSystem. */
  public FeederSubSystem() {}

  @Override
  public void periodic() {
    feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic(){
    feeder.simIterate();
  }
}

// test comment please ignore
