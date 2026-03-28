package frc.robot.subsystems.pivot;

public record CANcoderSpecifications(
    double gearRatio, boolean clockwisePositive, double magnetOffsetRots) {}
