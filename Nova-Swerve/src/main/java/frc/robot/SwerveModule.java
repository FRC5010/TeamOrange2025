// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.ExternalEncoder;
import com.thethriftybot.ThriftyNova.MotorType;
import com.thethriftybot.ThriftyNova.PIDSlot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.SwerveConstants;
import frc.robot.generated.SwerveConstants.EncoderType;
import frc.robot.generated.SwerveConstants.ModuleConstants;

public class SwerveModule {
    // Motors
    private final ThriftyNova m_driveMotor;
    private final ThriftyNova m_azimuthMotor;

    // Encoder (only used for Thrifty absolute encoder)
    private final AnalogEncoder m_thriftyEncoder;

    // Encoder configuration
    private final double m_encoderTicksPerRevolution;
    private double m_encoderOffsetTicks; // Used only for Thrifty encoder (RoboRIO reading), Nova just stores for
                                         // persistence
    private final String m_moduleName;

    // PID controller for Thrifty encoder (RIO-side control)
    private final PIDController m_turningPID = new PIDController(0.5, 0.0, 0.05);

    // Flag to track if we've checked for saved offsets yet (avoids blocking during
    // init)
    private boolean m_hasCheckedSavedOffset = false;

    // Drive motor conversion factors
    private static final double DRIVE_MOTOR_RPM_TO_MPS = (5676.0 / 60.0) / SwerveConstants.DRIVE_GEAR_RATIO
            * (SwerveConstants.WHEEL_DIAMETER_METERS * Math.PI);
    private static final double DRIVE_MOTOR_ROTATIONS_TO_METERS = (1.0 / SwerveConstants.DRIVE_GEAR_RATIO)
            * (SwerveConstants.WHEEL_DIAMETER_METERS * Math.PI);

    public SwerveModule(int driveMotorId, int azimuthMotorId, int encoderPort,
            double encoderTicksPerRevolution, double encoderOffsetTicks, String moduleName) {

        m_moduleName = moduleName;
        m_encoderTicksPerRevolution = encoderTicksPerRevolution;

        // Initialize motors
        m_driveMotor = new ThriftyNova(driveMotorId, MotorType.NEO);
        m_azimuthMotor = new ThriftyNova(azimuthMotorId, MotorType.NEO);
        m_thriftyEncoder = new AnalogEncoder(encoderPort);

        configureDriveMotor();
        configureAzimuthMotor();
        initializeOffset(encoderOffsetTicks);

        // Configure turning PID for continuous input (-180 to 180 degrees)
        m_turningPID.enableContinuousInput(-Math.PI, Math.PI);

        System.out.println(m_moduleName + " module initialized successfully");
    }

    /**
     * Configure the drive motor with PID and feedforward
     */
    private void configureDriveMotor() {
        m_driveMotor.factoryReset();

        // Configure drive PID and feedforward
        m_driveMotor.pid0.setPID(new PIDController(0.0004, 0.0, 0.0));
        m_driveMotor.pid0.setFF(1.0 / DRIVE_MOTOR_RPM_TO_MPS * 4);
        m_driveMotor.usePIDSlot(PIDSlot.SLOT0);
    }

    /**
     * Configure the azimuth motor based on encoder type
     */
    private void configureAzimuthMotor() {
        m_azimuthMotor.factoryReset();

        switch (ModuleConstants.ENCODER_SELECTED) {
            case REDUX_ENCODER:
            case THRIFTY_10PIN_ENCODER:
                m_azimuthMotor.setExternalEncoder(ExternalEncoder.REDUX_ENCODER);
                m_azimuthMotor.pid0.setP(0.0008).setD(0.0001);
                m_azimuthMotor.usePIDSlot(PIDSlot.SLOT0);
                m_azimuthMotor.setAbsoluteWrapping(true);
                break;

            case SRX_MAG_ENCODER:
                m_azimuthMotor.setExternalEncoder(ExternalEncoder.SRX_MAG_ENCODER);
                m_azimuthMotor.pid0.setP(0.0008).setD(0.0003);
                m_azimuthMotor.usePIDSlot(PIDSlot.SLOT0);
                break;

            case REV_ENCODER:
                m_azimuthMotor.setExternalEncoder(ExternalEncoder.REV_ENCODER);
                m_azimuthMotor.pid0.setP(0.0008).setD(0.00003);
                m_azimuthMotor.usePIDSlot(PIDSlot.SLOT0);
                break;

            case THRIFTY_ABSOLUTE_ENCODER:
                // No motor controller configuration needed - using RIO PID
                break;

            default:
                System.err.println("Unknown encoder type for " + m_moduleName);
                break;
        }
    }

    /**
     * Initialize the encoder offset, prioritizing saved values over constants
     */
    private void initializeOffset(double constantsOffsetTicks) {
        if (ModuleConstants.ENCODER_SELECTED == SwerveConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
            // Thrifty encoder: Use constants for initial setup, will check saved values
            // later in periodic
            m_encoderOffsetTicks = constantsOffsetTicks;
            System.out.println(m_moduleName + " Thrifty encoder initialized with constants: " +
                    ticksToDegrees(constantsOffsetTicks) + " degrees");
        } else {
            // Redux/SRX/REV: Set constants to Nova if provided, otherwise Nova will use
            // whatever is saved
            if (constantsOffsetTicks != 0) {
                m_azimuthMotor.setAbsOffset((int) constantsOffsetTicks);
                System.out.println(m_moduleName + " Nova encoder set to constants: " +
                        ticksToDegrees(constantsOffsetTicks) + " degrees");
            } else {
                System.out.println(m_moduleName + " Nova encoder using saved offset");
            }

            // Nova handles offset automatically, Java doesn't need to track it
            m_encoderOffsetTicks = 0;
        }
    }

    /**
     * Get raw encoder reading in ticks
     */
    private double getRawEncoderTicks() {
        if (ModuleConstants.ENCODER_SELECTED == SwerveConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
            return m_thriftyEncoder.get();
        } else {
            return m_azimuthMotor.getPositionAbs();
        }
    }

    /**
     * Get current encoder position in radians, accounting for offset
     */
    public double getEncoderPosition() {
        double rawTicks = getRawEncoderTicks();

        if (ModuleConstants.ENCODER_SELECTED == SwerveConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
            // Thrifty encoder: motor controller doesn't apply offset, so we do it in
            // software
            double adjustedTicks = rawTicks - m_encoderOffsetTicks;
            return ticksToRadians(adjustedTicks);
        } else if (ModuleConstants.ENCODER_SELECTED == SwerveConstants.EncoderType.THRIFTY_10PIN_ENCODER) {
            // Temp fix: Redux encoder type has wrong divisor, so reported angle is 2x
            // actual.
            // We divide by 2 to get the correct angle.
            return ticksToRadians(rawTicks / 2.0);
        } else {
            // Redux/SRX/REV: motor controller already applied offset, so use raw reading
            return ticksToRadians(rawTicks);
        }
    }

    /**
     * Get current swerve module state
     */
    public SwerveModuleState getSwerveState() {
        double velocityMPS = m_driveMotor.getVelocity() * DRIVE_MOTOR_RPM_TO_MPS;
        return new SwerveModuleState(velocityMPS, new Rotation2d(getEncoderPosition()));
    }

    /**
     * Get current swerve module position
     */
    public SwerveModulePosition getSwervePosition() {
        double positionMeters = m_driveMotor.getPosition() * DRIVE_MOTOR_ROTATIONS_TO_METERS;
        return new SwerveModulePosition(positionMeters, new Rotation2d(getEncoderPosition()));
    }

    /**
     * Set the desired state for this swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Get current angle and optimize the desired state
        Rotation2d currentAngle = new Rotation2d(getEncoderPosition());
        desiredState.optimize(currentAngle);

        // Set drive motor velocity
        double targetVelocityRPM = desiredState.speedMetersPerSecond / DRIVE_MOTOR_RPM_TO_MPS;
        m_driveMotor.setVelocity(targetVelocityRPM);

        // Set azimuth motor position
        setAzimuthPosition(desiredState.angle.getRadians());
    }

    /**
     * Set azimuth motor to target angle in radians
     */
    private void setAzimuthPosition(double targetAngleRadians) {
        if (ModuleConstants.ENCODER_SELECTED == SwerveConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
            // Use RIO PID controller for Thrifty encoder
            double currentAngle = getEncoderPosition();
            double output = m_turningPID.calculate(currentAngle, targetAngleRadians);
            m_azimuthMotor.set(output);
        } else {
            // Use motor controller position control for other encoders
            // Motor controller handles offset automatically, so just convert angle to ticks
            double targetTicks = radiansToTicks(targetAngleRadians);
            m_azimuthMotor.setPositionAbs(targetTicks);
        }
    }

    /**
     * Set current position as the new zero offset
     */
    public void setZeroOffset() {
        double currentRawTicks = getRawEncoderTicks();

        if (ModuleConstants.ENCODER_SELECTED == SwerveConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
            // Thrifty encoder: Read through RoboRIO AnalogEncoder, not Nova
            // Java does "position = raw - offset"
            // To make current position = 0: 0 = raw - offset, so offset = raw
            m_encoderOffsetTicks = currentRawTicks;

            // Save to Nova for persistence, but Nova doesn't use it for Thrifty encoder
            // control
            // (This is just for storing the value between code deploys)
            m_azimuthMotor.setAbsOffset((int) currentRawTicks);

            System.out.println(m_moduleName + " Thrifty encoder zeroed. Java offset: " + currentRawTicks +
                    " ticks (" + ticksToDegrees(currentRawTicks) + " degrees)");
        } else {
            // Redux/SRX/REV: Nova automatically applies offset to readings
            // Nova does "position = raw + offset"
            // To make current position = 0: 0 = raw + offset, so offset = -raw
            int newOffset = -(int) currentRawTicks;
            m_azimuthMotor.setAbsOffset(newOffset);

            System.out.println(m_moduleName + " Nova encoder zeroed. Raw: " + (int) currentRawTicks +
                    ", Nova offset: " + newOffset + " ticks");
        }

        // Verify the zero worked by checking position
        System.out.println(m_moduleName + " Position after zero: " +
                Math.toDegrees(getEncoderPosition()) + " degrees");
    }

    /**
     * Stop both motors
     */
    public void stop() {
        m_driveMotor.set(0.0);
        m_azimuthMotor.set(0.0);
    }

    /**
     * Update SmartDashboard with module information and handle zero button
     */
    public void updateSmartDashboard() {
        // Display encoder values
        SmartDashboard.putNumber(m_moduleName + " Raw Encoder (ticks)", getRawEncoderTicks());
        SmartDashboard.putNumber(m_moduleName + " Position (deg)", Math.toDegrees(getEncoderPosition()));
        SmartDashboard.putNumber(m_moduleName + " Position (rad)", getEncoderPosition());

        // Display offset info based on encoder type
        if (ModuleConstants.ENCODER_SELECTED == SwerveConstants.EncoderType.THRIFTY_ABSOLUTE_ENCODER) {
            SmartDashboard.putNumber(m_moduleName + " Java Offset (ticks)", m_encoderOffsetTicks);
            SmartDashboard.putNumber(m_moduleName + " Java Offset (deg)", ticksToDegrees(m_encoderOffsetTicks));
        } else {
            int motorControllerOffset = m_azimuthMotor.getAbsOffset();
            SmartDashboard.putNumber(m_moduleName + " MC Offset (ticks)", motorControllerOffset);
            SmartDashboard.putNumber(m_moduleName + " MC Offset (deg)", ticksToDegrees(motorControllerOffset));
        }

        // Display drive motor info
        SmartDashboard.putNumber(m_moduleName + " Drive Velocity", getSwerveState().speedMetersPerSecond);
        SmartDashboard.putNumber(m_moduleName + " Drive Position", getSwervePosition().distanceMeters);

        // Handle zero offset button
        String buttonKey = m_moduleName + " Set Zero";
        if (!SmartDashboard.containsKey(buttonKey)) {
            SmartDashboard.putBoolean(buttonKey, false);
        }

        if (SmartDashboard.getBoolean(buttonKey, false)) {
            setZeroOffset();
            SmartDashboard.putBoolean(buttonKey, false);
            SmartDashboard.putString("Last Zeroed", m_moduleName + " at " +
                    String.format("%.2f", Math.toDegrees(getEncoderPosition())) + "°");
        }
    }

    // Utility methods for unit conversions
    private double ticksToRadians(double ticks) {
        return (ticks / m_encoderTicksPerRevolution) * (2 * Math.PI);
    }

    private double radiansToTicks(double radians) {
        return (radians / (2 * Math.PI)) * m_encoderTicksPerRevolution;
    }

    private double ticksToDegrees(double ticks) {
        return (ticks / m_encoderTicksPerRevolution) * 360.0;
    }
}