package frc.robot.subsystems.climb;

public final class ClimberConstants {

    public static final double minAngle = -15;
    public static final double maxAngle = 150;

    public static final double stowAngle = 90;
    public static final double stretchAngle = 0;
    public static final double climbAngle = 45;

    // Converted to Degrees/sec and Degrees/sec^2 to match ProfiledPID units
    public static final double MAX_VELOCITY = 180;       // Max 180 degrees per second
    public static final double MAX_ACCELERATION = 360;   // Reach max speed in 0.5 seconds

    public static final double GEAR_RATIO = 50.0; 

    public static double tolerance = 0.5; // 0.5 degree tolerance is more realistic than 0.01

    // TUNED SIMULATION GAINS
    public static final double kP = 0.4;   // Lowered to prevent massive over-voltage spikes
    public static final double kI = 0.0;
    public static final double kD = 0.05;  // Slight damping to stop oscillations
    public static final double kS = 0.0;   // Keep friction at 0 for clean simulation testing
    public static final double kG = 0.5;   // Calculated gravity hold voltage for 50:1 ratio
    public static final double kV = 0.05;
}