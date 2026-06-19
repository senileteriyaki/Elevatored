package frc.robot.superstructure;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.MTimer;


public class SS extends StateMachineSubsystemBase<InternalState> {

    private static SS instance;

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private Intention intention;

    public boolean booted;

    private Alert unimplementedStateAlert = new Alert("SS InternalState unimplemented", AlertType.kError);

    public static final double PULLBACK_TIME_s = 0.45;

    private MTimer scoringTimer;

    private boolean homedYet = false;

    private static Drive drive;

    private static Elevator elevator;
    private static Arm arm;
    
    private SS() {
        super("SS");
        intention = Intention.IDLE;
        queueState(InternalState.IDLE);
        booted = false;
        scoringTimer = new MTimer();
        homedYet = false;

        drive = Drive.getInstance();
        elevator = Elevator.getInstance();
        arm = Arm.getInstance();
    }

    @Override
    public void inputPeriodic() {}

    public InternalState defaultIntentionHandling() {
        // Intention only contains IDLE per request; default to IDLE state
        return InternalState.IDLE;
    }

    private void handleIntention() {
        // Only IDLE/DISABLED are supported in InternalState, map all intentions to IDLE
        if (isState(InternalState.IDLE) || isState(InternalState.DISABLED) || isState(InternalState.BOOT)) {
            queueState(defaultIntentionHandling());
        }
    }

    @Override
    public void handleStateMachine() {
        if (!booted && !isState(InternalState.DISABLED)) {
            queueState(InternalState.BOOT);
        }

        handleIntention();

        switch (getState()) {
            case DISABLED:
                break;
            case BOOT:
                booted = true;
                queueState(InternalState.IDLE);
                break;
            case IDLE:
                if (stateInit()) {
                    homedYet = false;
                }
                // ensure drive is in a safe state
                if (drive != null) {
                    drive.setPathingOverride(PathingOverride.NONE);
                }
                break;
            case SCORE1:
                elevator.setCoralLevel(0);
            case SCORE2:
                elevator.setCoralLevel(1);
            case SCORE3:
                elevator.setCoralLevel(2);
            case SCORE4:
                elevator.setCoralLevel(3);
            default:
                unimplementedStateAlert.set(true);
                break;
        }
    }

    @Override
    public final void outputPeriodic() {
        Logger.recordOutput("SS/Booted?", booted);
        Logger.recordOutput("SS/Intention", intention);
        elevator.setArmLigament(0); //hella sus design pattern but whatever
    }

}

