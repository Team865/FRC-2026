package frc.robot.util;

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
    // filePath = Filesystem.getOperatingDirectory().toPath().resolve(filePath).toString();
    System.out.printf("Trying to write to file %s\n", filePath);
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

    System.out.println("FILE CONTENT:");
    System.out.println(csvBuilder.toString());

    try {
      File file = new File(filePath);

      if (!file.exists()) file.createNewFile();
      BufferedWriter writer = new BufferedWriter(new FileWriter(filePath));

      writer.write(csvBuilder.toString());
      writer.close();
      System.out.println("FILE SUCCESSFULLY SAVED");
    } catch (IOException e) {
      System.err.printf("Failed to log measurements to %s.\n", filePath);
      e.printStackTrace();
    }
  }
}
