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
    public static final double kP = 1; 
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    
    public static final double kS = 0.07;
    public static final double kG = 0.1;
    public static final double kV = 0.1;

    // --- Motion Profile Constraints ---
    public static final double maxVelocity = 2;
    public static final double maxAcceleration = 3.0;
    public static final double maxJerk = 0; // Using trapezoid so jerk should be zero
    
    public static final double tolerance = 0.05;

    public static final double[] levelHeights = {0.2, 0.5, 0.7, 1.5};
}