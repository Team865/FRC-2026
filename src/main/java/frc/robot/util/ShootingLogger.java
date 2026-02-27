package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ShootingLogger {
  private record Measurement(
      double distanceMeters, double hoodAngleRads, double flywheelVelocityRadsPerSec) {}

  private final List<Measurement> measurements = new ArrayList<>();

  public void addMeasurement(
      double distanceMeters, double hoodAngleRads, double flywheelVelocityRadsPerSec) {
    measurements.add(new Measurement(distanceMeters, hoodAngleRads, flywheelVelocityRadsPerSec));
  }

  public void writeToFile(String filePath) {
    filePath = Filesystem.getOperatingDirectory().toPath().resolve(filePath).toString();

    try {
      File file = new File(filePath);

      if (!file.exists()) file.createNewFile();
      BufferedWriter writer = new BufferedWriter(new FileWriter(filePath));
      StringBuilder csvBuilder = new StringBuilder();

      csvBuilder.append(
          "DISTANCE (METERS),HOOD ANGLE (RADIANS),FLYWHEEL VELOCITY (RADIANS PER SECOND)\n");

      for (Measurement measurement : measurements) {
        csvBuilder.append(
            String.format(
                "%s,%s,%s\n",
                measurement.distanceMeters(),
                measurement.hoodAngleRads(),
                measurement.flywheelVelocityRadsPerSec()));
      }

      writer.write(csvBuilder.toString());
      writer.close();
    } catch (IOException e) {
      System.out.printf("Failed to log measurements to %s.\n", filePath);
      e.printStackTrace();
    }
  }
}
