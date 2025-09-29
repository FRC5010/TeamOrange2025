// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OrangeTeam;

import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class Buttercup extends GenericRobot {
    private GenericDrivetrain drivetrain;

    public Buttercup(String directory) {
        super(directory);
        drivetrain = (GenericDrivetrain) subsystems.get(ConfigConstants.DRIVETRAIN);

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
}
}