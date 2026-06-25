package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
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
                if (Math.abs(inputs.pos - ClimberConstants.stretchAngle) < ClimberConstants.tolerance){ // Raymond: This logic doesn't work because like you don't want it to start climping the moment it reaches the stretch angle. You want to wait for a button press or something like if its at the location.
                    queueState(ClimbStates.CLIMBING);
                }
                break;
            case CLIMBING:
                target = ClimberConstants.climbAngle;
                io.goToPos(ClimberConstants.climbAngle);
                if (Math.abs(inputs.pos - ClimberConstants.climbAngle) < ClimberConstants.tolerance){
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
        // Raymond: Not updating climb2d here. You need to update it with the current position of the climb.
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

    // Raymond: YOU HAVE NO HANDLER METHODS> YOU DONT DO SHIT WITH THIS THEN.
}
