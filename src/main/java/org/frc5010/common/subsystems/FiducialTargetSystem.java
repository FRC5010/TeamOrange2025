// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.frc5010.common.sensors.camera.FiducialTargetCamera;
import org.frc5010.common.sensors.camera.GenericCamera;

public class FiducialTargetSystem extends CameraSystem {
  protected boolean hasTargets = false;
  protected double targetHeight = 0;
  protected double targetPitch = 0;
  protected double targetYaw = 0;
  protected String TARGET_PITCH = "targetPitch";
  protected String TARGET_YAW = "targetYaw";
  protected String TARGET_DISTANCE = "targetDistance";
  protected Pose2d targetPose = new Pose2d();

  public FiducialTargetSystem(FiducialTargetCamera camera) {
    super((GenericCamera) camera);
    values.declare(TARGET_PITCH, 0.0);
    values.declare(TARGET_YAW, 0.0);
    values.declare(TARGET_DISTANCE, 0.0);

    this.camera.registerUpdater(
        () -> {
          hasTargets = this.camera.hasValidTarget();
          values.set(HAS_VALID_TARGET, hasTargets);
        });
    this.camera.registerUpdater(
        () -> {
          targetYaw = this.camera.getTargetYaw();
          values.set(TARGET_YAW, targetYaw);
        });
    this.camera.registerUpdater(
        () -> {
          targetPitch = this.camera.getTargetPitch();
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

  /**
   * Gets the current list of fiducial IDs for this camera.
   *
   * @return the current list of fiducial IDs
   */
  public List<Integer> getFiducialIds() {
    return ((FiducialTargetCamera) camera).getFiducialIds();
  }

  /**
   * Sets the list of fiducial IDs for this camera. The camera will only consider targets with IDs
   * in this list when locating targets. This does not change the list spefified at construction
   * time to the pose camera so that the camera can be both a pose and target camera.
   *
   * @param fiducialIds the list of fiducial IDs to consider
   */
  public void setFiducialIds(List<Integer> fiducialIds) {
    ((FiducialTargetCamera) camera).setFiducialIds(fiducialIds);
  }
}
