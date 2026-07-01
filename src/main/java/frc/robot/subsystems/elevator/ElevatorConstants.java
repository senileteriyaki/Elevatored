package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final double minHeight = 0.0;
    public static final double maxHeight = 2.0;
    public static final double ZERO = ElevatorConstants.minHeight;
    public static final double STOW = 0;

    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 1.0;

    public static final double tolerance = 0.01;

    public static final double[] levelHeights = {0.2, 0.5, 0.7, 1.5};

    public static double drumRadius = 0.015;
}
