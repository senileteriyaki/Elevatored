package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.superstructure.Intention;
import frc.robot.superstructure.InternalState;
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
            drive.queueState(PathingMode.TRACKING);

        } else {
            drive.queueState(PathingMode.FIELD_RELATIVE);
            if (ss.getState() != InternalState.PRESCORE){
             drive.setPathingOverride(PathingOverride.NONE);
            }
        }
        double x_ = -OI.deadband(OI.DR.getLeftY() * mult);
        double y_ = -OI.deadband(OI.DR.getLeftX() * mult);
        double w_ = rotMult * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
        double throttle = Util
                .sqInput(1.0 - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

        SwerveInput input = new SwerveInput(x_, y_, w_, throttle);
        drive.setInput(input);

        boolean leftBumper = OI.DR.getRawButtonPressed(7);
        boolean rightBumper = OI.DR.getRawButtonPressed(8);
        boolean aButton = OI.DR.getRawButtonPressed(1);
        boolean bButton = OI.DR.getRawButtonPressed(2);
        boolean xButton = OI.DR.getRawButtonPressed(4);
        boolean yButton = OI.DR.getRawButtonPressed(5);
        boolean menuButton = OI.DR.getRawButtonPressed(12);
        
        if (rightBumper) { // increment reef level
            ss.setReef(MathUtil.clamp(ss.getReef() + 1, 0, SS.REEF_LEVELS));
        }

        if (leftBumper) { // decrement reef level
            ss.setReef(MathUtil.clamp(ss.getReef() - 1, 0, SS.REEF_LEVELS));
        }

        if (menuButton) {
            ss.intend(Intention.REJECT);
        }

        if (yButton) {
            ss.intend(Intention.IDLE);
        }

        if (xButton){
            ss.intend(Intention.SCORE);
        }

        if (aButton){
            ss.intend(Intention.PRECLIMB);
        }

        if (bButton){
            ss.intend(Intention.CLIMB);
        }
    }
}
