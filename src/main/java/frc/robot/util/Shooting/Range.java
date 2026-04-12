package frc.robot.util.Shooting;

public enum Range {
  SHORT(2.8),
  MEDIUM(100.0),
  LONG(10000.0);

  public final double MAX_THESHOLD_METERS;

  private Range(double maxThresholdMeters) {
    this.MAX_THESHOLD_METERS = maxThresholdMeters;
  }
}
