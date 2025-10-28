// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import org.frc5010.common.sensors.camera.GenericCamera;

public class VisibleTargetSystem extends CameraSystem {
  protected boolean hasTargets = false;
  protected double targetHeight = 0;
  protected double targetPitch = 0;
  protected double targetYaw = 0;
  protected String TARGET_PITCH = "targetPitch";
  protected String TARGET_YAW = "targetYaw";
  protected String TARGET_DISTANCE = "targetDistance";
  protected Pose2d targetPose = new Pose2d();

  public VisibleTargetSystem(GenericCamera camera, double targetHeight) {
    super(camera);
    this.targetHeight = targetHeight;
    values.declare(TARGET_PITCH, 0.0);
    values.declare(TARGET_YAW, 0.0);
    values.declare(TARGET_DISTANCE, 0.0);

    camera.registerUpdater(
        () -> {
          hasTargets = camera.hasValidTarget();
          values.set(HAS_VALID_TARGET, hasTargets);
        });
    camera.registerUpdater(
        () -> {
          targetYaw = camera.getTargetYaw();
          values.set(TARGET_YAW, targetYaw);
        });
    camera.registerUpdater(
        () -> {
          targetPitch = camera.getTargetPitch();
          values.set(TARGET_PITCH, targetPitch);
        });
  }

  /**
   * A method to get the distance to the target.
   *
   * @return the distance to the target, or Double.MAX_VALUE if no valid target
   */
  @Override
  public double getDistanceToTarget() {
    return camera.getDistanceToTarget(targetHeight);
  }

  /**
   * Does the camera have a valid target?
   *
   * @return true if the camera has a valid target
   */
  @Override
  public boolean hasValidTarget() {
    return hasTargets;
  }

  /**
   * Get the yaw of the target
   *
   * @return the yaw of the target
   */
  public double getTargetYaw() {
    return targetYaw;
  }

  /**
   * Get the pitch of the target
   *
   * @return the pitch of the target
   */
  public double getTargetPitch() {
    return targetPitch;
  }

  @Override
  public void periodic() {
    super.periodic();
    values.set(TARGET_DISTANCE, getDistanceToTarget());
  }
}
