package frc.robot;

import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.IPeriodic;
import frc.robot.util.Util;

public class ControlScheme implements IPeriodic {

    private Drive drive;
    private Elevator elevator;
    private Arm arm;
    private Climb climb;

    private String[] modes = { "disabled", "elevator", "arm", "climb", "drive" };
    private int modeCounter;
    private boolean modeInit;

    private int elevatorLevel;
    private int elbowLevel;
    private int shoulderLevel;

    public ControlScheme() {
        this.drive = Drive.getInstance();
        this.elevator = Elevator.getInstance();
        this.arm = Arm.getInstance();
        this.climb = Climb.getInstance();

        this.modeInit = true;
    }

    public void init() {
        drive.queueState(PathingMode.FIELD_RELATIVE);
        drive.setPathingOverride(PathingOverride.NONE);
        System.out.println("Control Scheme Initialized");
    }

    // FIXME: for some reason my controller bindings are all wrong (fedora linux, xbox one controller) & no auto mapping options! (sorry for the weird mappings)
    public void periodic() {
        if (OI.DR.getAButtonPressed()) {
            modeCounter++;
            System.out.println("\n\n\nCurrent mode is now: " + modes[modeCounter % modes.length] + "\n\n\n");
            modeInit = true;
            return;
        }

        switch (modes[modeCounter % modes.length]) {
            case "disabled":
                if (modeInit) {
                    System.out.println("\n\n\nCurrent mode is now: " + modes[modeCounter % modes.length] + "\n\n\n");
                    modeInit = false;
                }

                break;
            case "elevator":
                if (OI.DR.getLeftBumperButtonPressed()) { // as Y
                    elevatorLevel = Math.max(0, elevatorLevel - 1);
                    System.out.println("decreased, now: " + elevatorLevel);
                }

                if (OI.DR.getYButtonPressed()) { // as X
                    elevatorLevel = Math.min(ElevatorConstants.levelHeights.length - 1, elevatorLevel + 1);
                    System.out.println("increased, now: " + elevatorLevel);
                }

                if (OI.DR.getBButtonPressed()) { // as B
                    elevatorLevel = 0;
                    System.out.println("zeroed");
                }

                elevator.setHeight(ElevatorConstants.levelHeights[elevatorLevel]);
                break;
            case "arm":
                if (OI.DR.getLeftBumperButtonPressed()) { // as Y
                    elbowLevel = (elbowLevel + 1) % ArmConstants.elbowLevelAngles.length;
                    System.out.println("set elbow level: " + elbowLevel);
                }

                if (OI.DR.getYButtonPressed()) { // as X
                    shoulderLevel = (shoulderLevel + 1) % ArmConstants.shoulderLevelAngles.length;
                    System.out.println("set shoulder level: " + shoulderLevel);
                }

                if (OI.DR.getBButtonPressed()) { // as B
                    elbowLevel = shoulderLevel = 0;
                    System.out.println("zeroed all");
                }

                arm.trackToPosition(ArmConstants.elbowLevelAngles[elbowLevel], ArmConstants.shoulderLevelAngles[shoulderLevel]);
                break;
            case "climb":
                if (OI.DR.getLeftBumperButtonPressed()) { // as Y
                    climb.stretch();
                    System.out.println("stretching");
                }

                if (OI.DR.getYButtonPressed()) { // as X
                    climb.climb();
                    System.out.println("climbing");
                }

                if (OI.DR.getBButtonPressed()) { // as B
                    climb.stow();
                    System.out.println("stowing/idle");
                }
                break;
            case "drive":
                double rotMult = 0.5;
                double mult = 0;
                if (OI.DR.getLeftTriggerAxis() >= 0.8) {
                    mult = -0.4;
                } else {
                    mult = -0.7;
                }

                if (OI.DR.getRightTriggerAxis() >= 0.8) {
                    // drive.queueState(PathingMode.TRACKING);
                    drive.setPathingOverride(PathingOverride.SHOOTING);
                } else {
                    drive.queueState(PathingMode.FIELD_RELATIVE);
                    drive.setPathingOverride(PathingOverride.NONE);
                }
                double x_ = -OI.deadband(OI.DR.getLeftY() * mult);
                double y_ = -OI.deadband(OI.DR.getLeftX() * mult);
                double w_ = rotMult * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
                double throttle = Util
                        .sqInput(1.0 - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

                SwerveInput input = new SwerveInput(x_, y_, w_, throttle);
                drive.setInput(input);
                break;
            default:
                System.err.println("\n\n\nERROR: Current mode \"" + modes[modeCounter % modes.length] + "\" not implemented!\n\n\n");
                break;
        }
    }
}
