package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.util.IPeriodic;
import frc.robot.util.Util;

public class ControlScheme implements IPeriodic {

    protected Drive drive;

    public ControlScheme() {
        super();
        drive = Drive.getInstance();
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

        // if (OI.DR.getRightTriggerAxis() >= 0.8) {
        //     // drive.queueState(PathingMode.TRACKING);
        //     drive.setPathingOverride(PathingOverride.SHOOTING);

        // } else {
        //     drive.queueState(PathingMode.FIELD_RELATIVE);
        //     drive.setPathingOverride(PathingOverride.NONE);

        // }
        // double x_ = -OI.deadband(OI.DR.getLeftY() * mult);
        // double y_ = -OI.deadband(OI.DR.getLeftX() * mult);
        // double w_ = rotMult * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
        // double throttle = Util
        //         .sqInput(1.0 - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

        // SwerveInput input = new SwerveInput(x_, y_, w_, throttle);
        // drive.setInput(input);

        if (OI.DR.getRightTriggerAxis() > 0.8) {
        }
        else {
        }

        if (OI.DR.getAButtonReleased()) {
        }
    }
}
