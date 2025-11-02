// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json.devices;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc5010.common.config.DeviceConfiguration;
import org.frc5010.common.config.UnitsParser;
import org.frc5010.common.config.json.UnitValueJson;
import org.frc5010.common.motors.GenericMotorController;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ShooterConfig;
import yams.mechanisms.velocity.Shooter;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/** Add your docs here. */
public class YamsShooterConfigurationJson implements DeviceConfiguration {
  public MotorSetupJson motorSetup = new MotorSetupJson();
  public MotorSystemIdJson motorSystemId = new MotorSystemIdJson();
  public UnitValueJson lowerSoftLimit = new UnitValueJson(0, UnitsParser.DEGPS);
  public UnitValueJson upperSoftLimit = new UnitValueJson(0, UnitsParser.DEGPS);
  public double[] gearing;
  public UnitValueJson voltageCompensation = new UnitValueJson(12, UnitsParser.VOLTS);
  public UnitValueJson mass = new UnitValueJson(0, UnitsParser.LBS);
  public UnitValueJson diameter = new UnitValueJson(0, UnitsParser.IN);
  public double moi = 0;

  /**
   * Configure the given GenericSubsystem with a shooter using the given json configuration.
   *
   * @param deviceHandler the GenericSubsystem to configure
   * @return the configured shooter
   */
  @Override
  public Shooter configure(SubsystemBase deviceHandler) {
    GenericMotorController motor =
        DeviceConfigReader.getMotor(
            motorSetup.controllerType, motorSetup.motorType, motorSetup.canId);

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(deviceHandler)
            .withClosedLoopController(
                motorSystemId.feedBack.p,
                motorSystemId.feedBack.i,
                motorSystemId.feedBack.d,
                UnitsParser.parseAngularVelocity(motorSystemId.maxVelocity),
                UnitsParser.parseAngularAcceleration(motorSystemId.maxAcceleration))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(gearing)))
            .withIdleMode(MotorMode.valueOf(motorSetup.idleMode))
            .withTelemetry(
                motorSetup.name + "Motor", TelemetryVerbosity.valueOf(motorSetup.logLevel))
            .withStatorCurrentLimit(UnitsParser.parseAmps(motorSetup.currentLimit))
            .withMotorInverted(motorSetup.inverted)
            .withClosedLoopRampRate(UnitsParser.parseTime(motorSystemId.closedLoopRamp))
            .withOpenLoopRampRate(UnitsParser.parseTime(motorSystemId.openLoopRamp))
            .withFeedforward(
                new SimpleMotorFeedforward(
                    motorSystemId.feedForward.s,
                    motorSystemId.feedForward.v,
                    motorSystemId.feedForward.a))
            .withControlMode(ControlMode.valueOf(motorSystemId.controlMode));
    MotorSetupJson.setupFollowers(motorConfig, motorSetup);
    motor.setMotorSimulationType(
        motor.getMotorConfig().getMotorSimulationType(motorSetup.numberOfMotors));

    SmartMotorController smartMotor = motor.getSmartMotorController(motorConfig);
    ShooterConfig shooterConfig =
        new ShooterConfig(smartMotor)
            // .withMechanismPositionConfig(motorSetup.getMechanismPositionConfig())
            .withDiameter(UnitsParser.parseDistance(diameter))
            .withMass(UnitsParser.parseMass(mass))
            .withUpperSoftLimit(UnitsParser.parseAngularVelocity(upperSoftLimit))
            .withLowerSoftLimit(UnitsParser.parseAngularVelocity(lowerSoftLimit))
            .withTelemetry(motorSetup.name, TelemetryVerbosity.valueOf(motorSetup.logLevel));
    if (0 != moi) {
      shooterConfig.withMOI(moi);
    }
    Shooter shooter = new Shooter(shooterConfig);
    return shooter;
  }
}
