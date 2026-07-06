package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final double minHeight = 0.0;
    public static final double maxHeight = 2.0;
    public static final double ZERO = ElevatorConstants.minHeight;
    public static final double STOW = 0.0;

    // --- Physical Sim Parameters ---
    public static final double GEAR_RATIO = 10.0;        // Reasonable standard for a single Kraken elevator
    public static final double CARRIAGE_MASS_KG = 5.0;   // ~11 lbs carriage
    public static final double drumRadius = 0.015;       // Meters (1.5cm radius)

    // --- PID / Feedforward Constants ---
    public static final double kP = 15.0;  // 15 Volts per 1 meter of error (e.g. 0.1m error = 1.5V)
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    
    public static final double kS = 0.0;
    // FIX 5: Recalculated kG based on physical values
    public static final double kG = 0.2;   
    public static final double kV = 0.05;

    // --- Motion Profile Constraints ---
    public static final double maxVelocity = 2.0;     // m/s (Bumped up for realism)
    public static final double maxAcceleration = 3.0; // m/s^2 (0.5 takes too long to get up to speed)
    
    public static final double tolerance = 0.01; // 1 cm tolerance is perfectly fine for elevators

    public static final double[] levelHeights = {0.2, 0.5, 0.7, 1.5};
}