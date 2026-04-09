package frc.robot.util.Shooting;

import edu.wpi.first.math.util.Units;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

public class ShootingLogger {
  private record Measurement(
      double distanceMeters, double hoodAngleDeg, double flywheelVelocityRadsPerSec) {}

  private final List<Measurement> measurements = new ArrayList<>();
  private static final SimpleDateFormat dateFormat = new SimpleDateFormat("HH-mm-ss");

  public void addMeasurement(
      double distanceMeters, double hoodAngleDeg, double flywheelVelocityRadsPerSec) {
    measurements.add(new Measurement(distanceMeters, hoodAngleDeg, flywheelVelocityRadsPerSec));
  }

  public void save() {
    // filePath = Filesystem.getOperatingDirectory().toPath().resolve(filePath).toString();
    String filePath = "/U/logs/measurements_" + dateFormat.format(new Date()) + ".txt";

    System.out.printf("Trying to write to file %s\n", filePath);
    StringBuilder csvBuilder = new StringBuilder();

    csvBuilder.append(
        "DISTANCE (METERS),HOOD ANGLE (DEGREES),FLYWHEEL VELOCITY (RADIANS PER SECOND)\n");

    // Sort measurements
    measurements.sort(
        (m1, m2) ->
            (int)
                ((m1.flywheelVelocityRadsPerSec() - m2.flywheelVelocityRadsPerSec()) * 1000000000
                    + (m1.distanceMeters() - m2.distanceMeters()) * 1000));

    for (Measurement measurement : measurements) {
      csvBuilder.append(
          String.format(
              "%s,%s,%s\n",
              measurement.distanceMeters(),
              Units.radiansToDegrees(measurement.hoodAngleDeg()),
              measurement.flywheelVelocityRadsPerSec()));
    }

    String csv = csvBuilder.toString();

    System.out.println("FILE CONTENT:");
    System.out.println(csv);

    try {
      File file = new File(filePath);

      if (!file.exists()) file.createNewFile();
      BufferedWriter writer = new BufferedWriter(new FileWriter(filePath));

      writer.write(csv);
      writer.close();
      System.out.println("FILE SUCCESSFULLY SAVED");
    } catch (IOException e) {
      System.err.printf("Failed to log measurements to %s.\n", filePath);
      e.printStackTrace();
    }
  }
}
