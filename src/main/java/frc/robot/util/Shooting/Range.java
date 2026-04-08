package frc.robot.util.Shooting;

public enum Range {
  SHORT(3.8),
  MEDIUM(5.5),
  LONG(10000.0);

  public final double MAX_THESHOLD_METERS;

  private Range(double maxThresholdMeters) {
    this.MAX_THESHOLD_METERS = maxThresholdMeters;
  }
}
