package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final double minHeight = 0.0;
    public static final double maxHeight = 2.0;
    public static final double ZERO = ElevatorConstants.minHeight;
    public static final double STOW = 0.0;

    // --- Physical Sim Parameters ---
    public static final double GEAR_RATIO = 10.0;
    public static final double CARRIAGE_MASS_KG = 5.0;
    public static final double drumRadius = 0.015;
    public static final double currentLimit = 50;

    // --- PID / Feedforward Constants ---
    public static final double kP = 10.0;
    public static final double kI = 0.0;
    public static final double kD = 0.25;
    
    public static final double kS = 0.0;
    public static final double kG = 0.125;
    public static final double kV = 14.7;

    // --- Motion Profile Constraints ---
    public static final double maxVelocity = 0.8;
    public static final double maxAcceleration = 3.0;
    public static final double maxJerk = 0; // Using trapezoid so jerk should be zero
    
    public static final double tolerance = 0.025;

    public static final double[] levelHeights = {0.2, 0.5, 0.7, 1.5};
}