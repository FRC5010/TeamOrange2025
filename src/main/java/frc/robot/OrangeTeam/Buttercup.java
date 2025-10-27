// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OrangeTeam;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

/** Add your docs here. */
public class Buttercup extends GenericRobot {
  private GenericDrivetrain drivetrain;
  private ShooterSubsystem shooterSubsystem;
  private StateMachine shooterStateMachine = new StateMachine("Shooter State Machine");

  public Buttercup(String directory) {
    super(directory);
    drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);
    shooterSubsystem = new ShooterSubsystem();
  }

  @Override
  public void initAutoCommands() {
    drivetrain.setAutoBuilder();
  }

  @Override
  public Command generateAutoCommand(Command autoCommand) {
    return drivetrain.generateAutoCommand(autoCommand);
  }

  @Override
  public void buildAutoCommands() {
    super.buildAutoCommands();
    selectableCommand.addOption("Do Nothing", Commands.none());
  }

  @Override
  public void setupDefaultCommands(Controller driver, Controller operator) {
    driver.setRightTrigger(driver.createRightTrigger());

    drivetrain.setDefaultCommand(drivetrain.createDefaultCommand(driver));
  }

  @Override
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createAButton().onTrue(shooterSubsystem.setSpeed(0.5));

    JoystickButton rightBumper = driver.createRightBumper();

    State idle =
        shooterStateMachine.addState("idle", Commands.print("IDLE").andThen(Commands.idle()));
    State prep =
        shooterStateMachine.addState("prep", Commands.print("PREP").andThen(Commands.idle()));
    State fire =
        shooterStateMachine.addState("fire", Commands.print("FIRE").andThen(Commands.idle()));

    shooterStateMachine.setInitialState(idle);
    idle.switchTo(prep).when(rightBumper);
    prep.switchTo(fire).when(shooterSubsystem.isNearTarget(RPM.of(3000), RPM.of(200)));
    prep.switchTo(idle).when(() -> !rightBumper.getAsBoolean());
    fire.switchTo(idle).when(() -> !rightBumper.getAsBoolean());
  }
}
