package frc.robot.superstructure;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.util.MTimer;
import frc.robot.superstructure.InternalState;


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
    private static Climb climb;
    
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
        climb = Climb.getInstance();
    }

    @Override
    public void inputPeriodic() {}

    public InternalState defaultIntentionHandling() {
        return switch (intention){
            case IDLE -> InternalState.IDLE;
            case CLIMB -> InternalState.CLIMBING;
            case SCORE -> InternalState.SCORE1;
        };
    }

    private void handleIntention() {
        switch (getState()){
            case BOOT: break;
            case DISABLED: break;
            case IDLE: break;
            case SCORE1:

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
                arm.setCoralLevel(0);
                break;
            case SCORE2:
                elevator.setCoralLevel(1);
                arm.setCoralLevel(1);
                break;
            case SCORE3:
                elevator.setCoralLevel(2);
                arm.setCoralLevel(2);
                break;
            case SCORE4:
                elevator.setCoralLevel(3);
                arm.setCoralLevel(3);
            case CLIMBING:
                climb.queueState(ClimbStates.STRETCHING);
                break;
            default:
                unimplementedStateAlert.set(true);
                break;
        }
    }

    @Override
    public final void outputPeriodic() {
        Logger.recordOutput("SS/Booted?", booted);
        Logger.recordOutput("SS/Intention", intention);
        elevator.setArmLigament(arm.getElbowPos()); //hella sus design pattern but whatever
    }

}

