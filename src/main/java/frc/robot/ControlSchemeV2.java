package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.superstructure.Intention;
import frc.robot.superstructure.SS;
import frc.robot.util.IPeriodic;
import frc.robot.util.Util;

public class ControlSchemeV2 implements IPeriodic {

    protected Drive drive;
    protected Elevator elevator;
    private SS ss;

    public ControlSchemeV2() {
        this.drive = Drive.getInstance();
        this.elevator = Elevator.getInstance();

        this.ss = SS.getInstance();
    }

    public void init() {
        drive.queueState(PathingMode.FIELD_RELATIVE);
        drive.setPathingOverride(PathingOverride.NONE);
        System.out.println("Control Scheme Initialized");
    }

    public void periodic() {
        double rotMult = 0.5;
        double mult = 0;
        if (OI.DR.getLeftTriggerAxis() >= 0.8) {
            mult = -0.4;
        } else {
            mult = -0.7;
        }

        if (OI.DR.getRightTriggerAxis() >= 0.8) {
            // drive.queueState(PathingMode.TRACKING);

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

        if (OI.DR.getRightBumperButtonPressed()){ //increment reef level
            ss.setReef((ss.getReef() + 1) % 4);
        }

        if (OI.DR.getLeftBumperButtonPressed()){ //decrement reef level
            ss.setReef((ss.getReef() - 1 + 4) % 4);
        }

        if (OI.DR.getXButton()){
            ss.intend(Intention.SCORE);
        }

        if (OI.DR.getAButton()){ //first stage of climbing
            ss.intend(Intention.CLIMB1);
        }

        if (OI.DR.getBButton()){
            ss.intend(Intention.CLIMB2);
        }

    }
}
