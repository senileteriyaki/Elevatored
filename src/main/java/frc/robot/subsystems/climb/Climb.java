package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Climb extends StateMachineSubsystemBase<ClimbStates> {
    private ClimbIO io;
    private static Climb instance;
    private double target;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
    public final Climb2D climb2d;

    public Climb(ClimbIO io) {
        super("climb");
        this.io = io;
        target = 0;
        queueState(ClimbStates.IDLE);
        climb2d = new Climb2D("climb", new Color8Bit(Color.kLavender), new Color8Bit(Color.kDarkOrange));
    }

    public static Climb getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case SIM:
                    instance = new Climb(new ClimbIOSim());
                    break;
                case REAL:
                    instance = new Climb(new ClimbIOReal());
                    break;
                case REPLAY:
                    instance = new Climb(new ClimbIOReal());
                    break;
            }
        }
        return instance;
    }

    public void handleStateMachine() {
        switch (getState()) {
            case DISABLED:
                io.stop();
                break;
            case IDLE:
                target = ClimberConstants.stowAngle;
                io.hold(ClimberConstants.stowAngle);
                break;
            case HOLDING:
                target = ClimberConstants.climbAngle;
                io.hold(ClimberConstants.climbAngle);
                break;
            case STRETCHING:
                target = ClimberConstants.stretchAngle;
                io.goToPos(ClimberConstants.stretchAngle);
                break;
            case CLIMBING:
                target = ClimberConstants.climbAngle;
                io.goToPos(ClimberConstants.climbAngle);
                break;
            default:
                break;
        }
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);
    }

    @Override
    public void outputPeriodic() {
        Logger.recordOutput("Climb/target", target);
        climb2d.set(inputs.pos);
        climb2d.periodic();
    }

    public void setTarget(double p) {
        target = p;
    }

}
