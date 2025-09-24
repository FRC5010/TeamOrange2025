// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.config.json;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import org.frc5010.common.arch.GenericRobot;
import org.frc5010.common.vision.AprilTags;

/** JSON class with an array of cameras to configure */
public class VisionPropertiesJson {
  /** An array of camera names */
  public String[] cameras;

  public String aprilTagLayout = "default";

  /**
   * Creates cameras for a given robot using the provided map of camera configurations.
   *
   * @param robot the robot to add the cameras to
   * @param map a map of camera configurations, where the key is the camera name and the value is
   *     the configuration object
   */
  public void createCameraSystem(GenericRobot robot, Map<String, CameraConfigurationJson> map) {
    map.keySet()
        .forEach(
            it -> {
              map.get(it).configureCamera(robot);
            });
  }

  /**
   * Reads in cameras from the provided directory.
   *
   * @param directory the directory to read from
   * @return the map of camera configurations
   */
  public Map<String, CameraConfigurationJson> readCameraSystem(File directory) throws IOException {
    if (aprilTagLayout.equals("5010")) {
      AprilTags.setAprilTagFieldLayout(AprilTags.aprilTagRoomLayout);
    } else if (aprilTagLayout.equals("default")) {
      AprilTagFieldLayout layout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
      AprilTags.setAprilTagFieldLayout(layout);
    } else {
      try {
        AprilTagFieldLayout layout =
            AprilTagFieldLayout.loadFromResource(
                AprilTagFields.valueOf(aprilTagLayout).m_resourceFile);
        AprilTags.setAprilTagFieldLayout(layout);
      } catch (IllegalArgumentException e) {
        AprilTagFieldLayout layout = AprilTagFieldLayout.loadFromResource(aprilTagLayout);
        AprilTags.setAprilTagFieldLayout(layout);
      }
    }

    Map<String, CameraConfigurationJson> camerasMap = new HashMap<>();
    for (String cameraString : cameras) {
      File cameraFile = new File(directory, "cameras/" + cameraString);
      assert cameraFile.exists();
      CameraConfigurationJson camera =
          new ObjectMapper().readValue(cameraFile, CameraConfigurationJson.class);
      camerasMap.put(camera.name, camera);
    }
    return camerasMap;
  }
}
