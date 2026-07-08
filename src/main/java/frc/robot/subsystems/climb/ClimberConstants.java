package frc.robot.subsystems.climb;

public final class ClimberConstants {

    public static final double minAngle = -15;
    public static final double maxAngle = 150;

    public static final double stowAngle = 90;
    public static final double stretchAngle = 0;
    public static final double climbAngle = 45;

    // Physical constants
    public static final double GEAR_RATIO = 50.0;
    public static final double MOI = 1.5;
    public static final double LENGTH = 0.5;
    public static final double currentLimit = 50;

    // Sim + motor configs
    public static final double MAX_VELOCITY = 180;
    public static final double MAX_ACCELERATION = 360;
    public static final double MAX_JERK = 0; // Using trapezoidal profile so jerk must be 0

    public static double tolerance = 0.5;

    // TUNED SIMULATION GAINS
    public static final double kP = 0.4;
    public static final double kI = 0.0;
    public static final double kD = 0.05;

    public static final double kS = 0.0;
    public static final double kG = 0.65;
    public static final double kV = 0.96;
}