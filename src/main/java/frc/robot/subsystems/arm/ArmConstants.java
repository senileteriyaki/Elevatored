package frc.robot.subsystems.arm;

// TODO: implement new minAngle, elbow + shoulder level angles
public final class ArmConstants {
    public static final int ELBOW_MOTOR_ID = 2;
    public static final int SHOULDER_MOTOR_ID = 3;

    public static final double sMin = -90; // prolly fake but whatever
    public static final double sMax = 90;

    public static final double eMin = -135;
    public static final double eMax = 135;

    public static final double ELBOW_ZERO = ArmConstants.eMin;
    public static final double ELBOW_STOW = 0;

    public static final double SHOULDER_ZERO = ArmConstants.sMin;
    public static final double SHOULDER_STOW = 90;

    // --- PID Constants (Tuned for Degrees) ---
    public static final double elbowKP = 0.15;
    public static final double elbowKI = 0.8;
    public static final double elbowKD = 0.05;

    public static final double shoulderKP = 0.4;
    public static final double shoulderKI = 0.0;
    public static final double shoulderKD = 0.05;

    // --- Feedforward Constants (Required for Sim) ---
    public static final double elbowKS = 0.05;
    public static final double elbowKG = 0.0;   
    public static final double elbowKV = 0.05;

    public static final double shoulderKS = 0.0;
    public static final double shoulderKG = 0.34;  
    public static final double shoulderKV = 0.05;

    // --- Motion Profile Constraints ---
    public static final double ELBOW_MAX_VELOCITY_DPS = 180;
    public static final double ELBOW_MAX_ACCELERATION_DPS2 = 360;

    public static final double SHOULDER_MAX_VELOCITY_DPS = 180;
    public static final double SHOULDER_MAX_ACCELERATION_DPS2 = 360;

    // --- Tolerances ---
    public static final double elbowTolerance = 0.5;
    public static final double shoulderTolerance = 0.5;

    public static final double[] elbowLevelAngles = {42, 60, 70, 67};
    public static final double[] shoulderLevelAngles = {80, 45, 32, 56};
}