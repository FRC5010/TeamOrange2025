// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.generated.SwerveConstants;
import frc.robot.generated.SwerveConstants.GyroType;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.generated.SwerveConstants;


public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final GyroSupplier gyro = GyroFactory.createGyro(SwerveConstants.GyroConstants.GYRO_TYPE, SwerveConstants.GyroConstants.GYRO_PARAMS);

  private final Drivetrain m_swerve = new Drivetrain(gyro::getRotation2d, new Pose2d());
  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(20);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(20);

  @Override
  public void robotPeriodic() {
      // This runs in all robot modes (disabled, auto, teleop, test)
      m_swerve.periodic();
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.01))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.01))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.02))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
    // m_simSwerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}

interface GyroSupplier {
  Rotation2d getRotation2d();
}

class GyroFactory {
  public static GyroSupplier createGyro(SwerveConstants.GyroType gyroType, Object... params) {
      switch (gyroType) {
          case NAVX -> {
              // Check if parameters are provided, otherwise use defaults
              AHRS.NavXComType comType = params.length > 0 && params[0] instanceof AHRS.NavXComType
                  ? (AHRS.NavXComType) params[0]
                  : AHRS.NavXComType.kMXP_SPI;

              AHRS ahrs = new AHRS(comType);
              return ahrs::getRotation2d;
          }
          case PIGEON2 -> {
              // Check if parameters are provided, otherwise use defaults
              int canID = params.length > 0 && params[0] instanceof Integer
                  ? (Integer) params[0]
                  : 1; // Default CAN ID

              Pigeon2 pigeon = new Pigeon2(canID);
              return pigeon::getRotation2d;
          }
          default -> throw new IllegalArgumentException("Unsupported gyro type: " + gyroType);
      }
  }
}