package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
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
        this.target = ClimberConstants.stowAngle;
        queueState(ClimbStates.IDLE);
        
        climb2d = new Climb2D("climb", new Color8Bit(Color.kMediumSpringGreen));
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
            case HOLDING:
                io.hold(target);
                break;
            case STRETCHING:
            case CLIMBING:
                io.goToPos(target);
                
                if (reachedTarget()) { // ethan - should handle state switching elsewhere
                    queueState(ClimbStates.HOLDING);
                }
                break;
            default:
                break;
        }
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        climb2d.set(inputs.pos_deg);
        Logger.processInputs("Climb", inputs);
    }

    @Override
    public void outputPeriodic() {
        Logger.recordOutput("Climb/target", target);
        Logger.recordOutput("Climb/state", getState());
        climb2d.set(inputs.pos_deg);
        climb2d.periodic();
    }

    private void setTarget(double target) { // ethan - satyaki queued travelling here. best to do so here too.
        this.target = target;
    }

    public void stretch() {
        setTarget(ClimberConstants.stretchAngle);
        queueState(ClimbStates.STRETCHING);
    }

    public void climb() {
        setTarget(ClimberConstants.climbAngle);
        queueState(ClimbStates.CLIMBING);
    }

    public void hold() {
        queueState(ClimbStates.HOLDING);
    }

    public void disable() {
        queueState(ClimbStates.DISABLED);
    }

    public void idle() {
        setTarget(ClimberConstants.stowAngle);
        queueState(ClimbStates.IDLE);
    }

    public void abortAndHold() {
        target = inputs.pos_deg;
        hold();
    }

    public boolean reachedTarget(){
        return Math.abs(inputs.pos_deg - target) < ClimberConstants.tolerance;
    }
}
