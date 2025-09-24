// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import java.io.File;
import java.io.IOException;
import java.util.Optional;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.constants.RobotConstantsDef;
import org.frc5010.common.drive.swerve.GenericSwerveDrivetrain;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.drive.swerve.akit.GyroIOPigeon2;
import org.frc5010.common.drive.swerve.akit.GyroIOSim;
import org.frc5010.common.drive.swerve.akit.ModuleIOSpark;
import org.frc5010.common.drive.swerve.akit.ModuleIOSparkTalon;
import org.frc5010.common.drive.swerve.akit.ModuleIOTalonFXReal;
import org.frc5010.common.drive.swerve.akit.ModuleIOTalonFXSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

/** Add your docs here. */
public class AKitSwerveDrivetrainJson implements DrivetrainPropertiesJson {
  public String type = "SparkTalon";
  public double robotMassKg = 74.088;
  public double robotMOI = 6.883;
  public double wheelCOF = 1.2;
  private Optional<GamePiecesJson> gamePiecesJson = Optional.empty();

  @Override
  public void readDrivetrainConfiguration(GenericRobot robot, File directory) throws IOException {
    AkitSwerveDrive.mapleSimConfig =
        DriveTrainSimulationConfig.Default()
            .withRobotMass(Kilograms.of(robotMassKg))
            .withCustomModuleTranslations(getModuleTranslations())
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                    wheelCOF));
  }

  @Override
  public void createDriveTrain(GenericRobot robot) {
    AkitSwerveDrive drive;
    if (RobotBase.isSimulation()) {

      AkitSwerveDrive.driveSimulation =
          new SwerveDriveSimulation(
              AkitSwerveDrive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
      SimulatedArena.getInstance().addDriveTrainSimulation(AkitSwerveDrive.driveSimulation);
      drive =
          new AkitSwerveDrive(
              new GyroIOSim(AkitSwerveDrive.driveSimulation.getGyroSimulation()),
              new ModuleIOTalonFXSim(
                  TunerConstants.FrontLeft, AkitSwerveDrive.driveSimulation.getModules()[0]),
              new ModuleIOTalonFXSim(
                  TunerConstants.FrontRight, AkitSwerveDrive.driveSimulation.getModules()[1]),
              new ModuleIOTalonFXSim(
                  TunerConstants.BackLeft, AkitSwerveDrive.driveSimulation.getModules()[2]),
              new ModuleIOTalonFXSim(
                  TunerConstants.BackRight, AkitSwerveDrive.driveSimulation.getModules()[3]),
              AkitSwerveDrive.driveSimulation::setSimulationWorldPose);
    } else {
      if ("SparkTalon".equals(type)) {
        drive =
            new AkitSwerveDrive(
                new GyroIOPigeon2(),
                new ModuleIOSparkTalon(TunerConstants.FrontLeft),
                new ModuleIOSparkTalon(TunerConstants.FrontRight),
                new ModuleIOSparkTalon(TunerConstants.BackLeft),
                new ModuleIOSparkTalon(TunerConstants.BackRight),
                (pose) -> {});
      } else if ("Spark".equals(type)) {
        drive =
            new AkitSwerveDrive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3),
                (pose) -> {});
      } else if ("TalonFX".equals(type)) {
        drive =
            new AkitSwerveDrive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                new ModuleIOTalonFXReal(TunerConstants.BackRight),
                (pose) -> {});
      } else {
        throw new IllegalArgumentException("Unknown AkitSwerveDrive type: " + type);
      }
    }
    GenericSwerveDrivetrain drivetrain =
        new GenericSwerveDrivetrain(
            new LoggedMechanism2d(RobotConstantsDef.robotVisualH, RobotConstantsDef.robotVisualV),
            robot.getDrivetrainConstants(),
            drive);
    robot.addSubsystem(ConfigConstants.DRIVETRAIN, drivetrain);
    robot.setPoseSupplier(() -> drivetrain.getPoseEstimator().getCurrentPose());
    robot.setSimulatedPoseSupplier(() -> drive.getMapleSimPose());
    gamePiecesJson.ifPresent(it -> it.createGamePieces(drivetrain));
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
