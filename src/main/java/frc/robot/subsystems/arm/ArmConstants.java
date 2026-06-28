package frc.robot.subsystems.arm;

public final class ArmConstants {
    public static final int ELBOW_MOTOR_ID = 2;
    public static final int SHOULDER_MOTOR_ID = 3;

    public static final double minAngle = -90; //prolly fake but whatever
    public static final double maxAngle = 90;

    // Constants for PIDController sim
    public static final double elbowKP = 17;
    public static final double elbowKI = 0;
    public static final double elbowKD = 0;

    public static final double shoulderKP = 25;
    public static final double shoulderKI = 0;
    public static final double shoulderKD = 0;

    public static final double ELBOW_MAX_VELOCITY_DPS = 720;
    public static final double ELBOW_MAX_ACCELERATION_DPS2 = 1380;

    public static final double SHOULDER_MAX_VELOCITY_DPS = 480;
    public static final double SHOULDER_MAX_ACCELERATION_DPS2 = 1380;

    public static final double elbowTolerance = 0.05;
    public static final double shoulderTolerance = 0.05;

    public static final double[] elbowLevelAngles = {42, 60, 70, 67}; //entirely random numbers but wtv
    public static final double[] shoulderLevelAngles = {80, 45, 32, 56};
}
