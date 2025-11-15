// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.SwerveConstants;
import frc.robot.generated.SwerveConstants.ModuleConstants;
import java.util.function.Supplier;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    private final SwerveModule m_frontLeft = new SwerveModule(
        ModuleConstants.FRONT_LEFT_DRIVE_MOTOR_ID, 
        ModuleConstants.FRONT_LEFT_AZIMUTH_MOTOR_ID, 
        0, ModuleConstants.ENCODER_TICKS_PER_ROTATION, 
        ModuleConstants.FRONT_LEFT_ENCODER_OFFSET, 
        "Front Left");
        
    private final SwerveModule m_frontRight = new SwerveModule(
        ModuleConstants.FRONT_RIGHT_DRIVE_MOTOR_ID, 
        ModuleConstants.FRONT_RIGHT_AZIMUTH_MOTOR_ID, 
        1, ModuleConstants.ENCODER_TICKS_PER_ROTATION,
        ModuleConstants.FRONT_RIGHT_ENCODER_OFFSET, 
        "Front Right");
        
    private final SwerveModule m_backLeft = new SwerveModule(
        ModuleConstants.BACK_LEFT_DRIVE_MOTOR_ID, 
        ModuleConstants.BACK_LEFT_AZIMUTH_MOTOR_ID, 
        2,  ModuleConstants.ENCODER_TICKS_PER_ROTATION,
        ModuleConstants.BACK_LEFT_ENCODER_OFFSET, 
        "Back Left");
        
    private final SwerveModule m_backRight = new SwerveModule(
        ModuleConstants.BACK_RIGHT_DRIVE_MOTOR_ID, 
        ModuleConstants.BACK_RIGHT_AZIMUTH_MOTOR_ID, 
        3, ModuleConstants.ENCODER_TICKS_PER_ROTATION,
        ModuleConstants.BACK_RIGHT_ENCODER_OFFSET, 
        "Back Right");
        
    private final Supplier<Rotation2d> m_gyroSupplier;

    private final SwerveDriveKinematics m_kinematics =
        new SwerveDriveKinematics(
            SwerveConstants.FRONT_LEFT_LOCATION, SwerveConstants.FRONT_RIGHT_LOCATION, 
            SwerveConstants.BACK_LEFT_LOCATION, SwerveConstants.BACK_RIGHT_LOCATION);

    private final SwerveDriveOdometry m_odometry;

    /**
     * Constructs the Drivetrain with a supplier that returns the current robot heading.
     *
     * @param gyroSupplier A lambda or method reference that returns current heading as Rotation2d
     * @param initialPose The initial pose of the robot
     */
    public Drivetrain(Supplier<Rotation2d> gyroSupplier, Pose2d initialPose) {
        this.m_gyroSupplier = gyroSupplier;

        m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            m_gyroSupplier.get(),
            new SwerveModulePosition[] {
                m_frontLeft.getSwervePosition(),
                m_frontRight.getSwervePosition(),
                m_backLeft.getSwervePosition(),
                m_backRight.getSwervePosition()
            },
            initialPose);
    }

    /**
     * Drives the robot using desired speeds and rotation.
     *
     * @param xSpeed        Speed in the X direction (forward).
     * @param ySpeed        Speed in the Y direction (sideways).
     * @param rot           Angular speed (radians/sec).
     * @param fieldRelative Whether movement should be relative to the field.
     * @param periodSeconds The loop period (dt).
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
        ChassisSpeeds speeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyroSupplier.get())
            : new ChassisSpeeds(xSpeed, ySpeed, rot);

        var swerveModuleStates = m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(speeds, periodSeconds));

        // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.TOP_SPEED_METERS_PER_SEC);

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(
            m_gyroSupplier.get(),
            new SwerveModulePosition[] {
                m_frontLeft.getSwervePosition(),
                m_frontRight.getSwervePosition(),
                m_backLeft.getSwervePosition(),
                m_backRight.getSwervePosition()
            });
    }

    /**
     * Gets the current robot pose
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to a specific pose
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
            m_gyroSupplier.get(),
            new SwerveModulePosition[] {
                m_frontLeft.getSwervePosition(),
                m_frontRight.getSwervePosition(),
                m_backLeft.getSwervePosition(),
                m_backRight.getSwervePosition()
            },
            pose);
    }

    /**
     * Stops all swerve modules
     */
    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Sets all modules to X formation for defense
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Gets current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(
            m_frontLeft.getSwerveState(),
            m_frontRight.getSwerveState(),
            m_backLeft.getSwerveState(),
            m_backRight.getSwerveState());
    }

    /**
     * Should be called periodically to update odometry and SmartDashboard
     */
    public void periodic() {
        // Update odometry
        updateOdometry();
        
        // Update module calibration info on SmartDashboard (no parameters needed)
        m_frontLeft.updateSmartDashboard();
        m_frontRight.updateSmartDashboard();
        m_backLeft.updateSmartDashboard();
        m_backRight.updateSmartDashboard();
        
        // Robot pose information
        Pose2d currentPose = getPose();
        SmartDashboard.putNumber("Robot X (m)", currentPose.getX());
        SmartDashboard.putNumber("Robot Y (m)", currentPose.getY());
        SmartDashboard.putNumber("Robot Rotation (deg)", currentPose.getRotation().getDegrees());
        
        // Gyro information
        SmartDashboard.putNumber("Gyro Angle (deg)", m_gyroSupplier.get().getDegrees());
        
        // Current chassis speeds
        ChassisSpeeds speeds = getChassisSpeeds();
        SmartDashboard.putNumber("Chassis X Speed (m/s)", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chassis Y Speed (m/s)", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Chassis Angular Speed (rad/s)", speeds.omegaRadiansPerSecond);
        
        // Module states for debugging
        SmartDashboard.putNumber("FL Speed (m/s)", m_frontLeft.getSwerveState().speedMetersPerSecond);
        SmartDashboard.putNumber("FL Angle (deg)", m_frontLeft.getSwerveState().angle.getDegrees());
        
        SmartDashboard.putNumber("FR Speed (m/s)", m_frontRight.getSwerveState().speedMetersPerSecond);
        SmartDashboard.putNumber("FR Angle (deg)", m_frontRight.getSwerveState().angle.getDegrees());
        
        SmartDashboard.putNumber("BL Speed (m/s)", m_backLeft.getSwerveState().speedMetersPerSecond);
        SmartDashboard.putNumber("BL Angle (deg)", m_backLeft.getSwerveState().angle.getDegrees());
        
        SmartDashboard.putNumber("BR Speed (m/s)", m_backRight.getSwerveState().speedMetersPerSecond);
        SmartDashboard.putNumber("BR Angle (deg)", m_backRight.getSwerveState().angle.getDegrees());

        // Control buttons
        handleSmartDashboardButtons();
    }

    /**
     * Handle SmartDashboard button inputs
     */
    private void handleSmartDashboardButtons() {
        // Initialize buttons if they don't exist
        if (!SmartDashboard.containsKey("Reset Odometry")) {
            SmartDashboard.putBoolean("Reset Odometry", false);
        }
        if (!SmartDashboard.containsKey("Set X Formation")) {
            SmartDashboard.putBoolean("Set X Formation", false);
        }
        if (!SmartDashboard.containsKey("Stop All Modules")) {
            SmartDashboard.putBoolean("Stop All Modules", false);
        }

        // Reset odometry button
        if (SmartDashboard.getBoolean("Reset Odometry", false)) {
            resetOdometry(new Pose2d());
            SmartDashboard.putBoolean("Reset Odometry", false);
            System.out.println("Odometry reset to origin");
        }

        // X formation button
        if (SmartDashboard.getBoolean("Set X Formation", false)) {
            setX();
            SmartDashboard.putBoolean("Set X Formation", false);
            System.out.println("Set to X formation");
        }

        // Stop all modules button
        if (SmartDashboard.getBoolean("Stop All Modules", false)) {
            stopModules();
            SmartDashboard.putBoolean("Stop All Modules", false);
            System.out.println("All modules stopped");
        }
    }
}