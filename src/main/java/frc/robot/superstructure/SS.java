package frc.robot.superstructure;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.tracking.Tracking;
import frc.robot.subsystems.vision.Vision;


public class SS extends StateMachineSubsystemBase<InternalState> {

    private static SS instance;

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private Intention intention;

    private boolean resetIntention = false;
    private boolean ready1 = false;
    private boolean ready2 = false;

    public boolean booted;

    private Alert unimplementedStateAlert = new Alert("SS InternalState unimplemented", AlertType.kError);

    public static final double SCORE_s = 2; //ts elevator is not that fast
    public static final double PULLBACK_TIME_s = 0.45;
    public static final double POSTSCORE_s = 0.5;
    public static final double REJECT_TIMEOUT_s = 2;

    private Timer timer;

    private boolean homedYet = false;

    private static Drive drive;

    private static Elevator elevator;
    private static Arm arm;
    private static Climb climb;
    private static Tracking tracking;
    private static Vision vision;

    private int coralLevel;

    private SS() {
        super("SS");

        this.intention = Intention.IDLE;
        queueState(InternalState.BOOT);
        
        this.booted = false;
        this.homedYet = false;
        this.coralLevel = 3;

        timer = new Timer();
        timer.start();

        drive = Drive.getInstance();
        elevator = Elevator.getInstance();
        arm = Arm.getInstance();
        climb = Climb.getInstance();
        tracking = Tracking.getInstance();
        vision = Vision.getInstance();

        tracking.setValidIds(new double[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21}); //reef only
    }

    @Override
    public void inputPeriodic() {}

    public InternalState defaultIntentionHandling() {
        return switch (intention){
            case IDLE -> InternalState.IDLE;
            case REJECT -> InternalState.IDLE;
            case SCORE -> InternalState.PRESCORE;
            case CLIMB1 -> InternalState.PRECLIMB;
            case CLIMB2 -> InternalState.CLIMB;
            default -> InternalState.IDLE;
        };
    }

    /**
     * Controls the switching between states by handling the intention. No subsystem
     * actions are called since that behavior is manged by handleStateMachine().
     */
    private void handleIntention() {
        switch (getState()) {
            case BOOT:
            case DISABLED:
                break; // Wait for handleStateMachine() to finish booting / disable state switching
            case REJECT:
                if (timer.hasElapsed(REJECT_TIMEOUT_s)){
                    queueState(InternalState.IDLE); 
                }
                break;
            case IDLE:

                queueState(defaultIntentionHandling());
                break;
            case PRESCORE:
                queueState(switch (intention) {
                    case REJECT:
                        timer.reset();
                        yield InternalState.REJECT;
                    case SCORE: 
                        if (tracking.finishedTracking()){
                            timer.restart();
                        }
                        yield tracking.finishedTracking() ? InternalState.SCORESTAGE1 : InternalState.PRESCORE;
                    default: 
                        yield defaultIntentionHandling();
                });
                break;
            case SCORESTAGE1:
                queueState(switch (intention) {
                    case REJECT:
                        timer.reset();
                        yield InternalState.REJECT;
                    case SCORE:
                        ready1 = okToScore1();
                        if (ready1){
                            timer.restart();
                        }
                        yield ready1 ? InternalState.SCORESTAGE2 : InternalState.SCORESTAGE1;
                    default:
                        yield defaultIntentionHandling();
                });
                break;
            case SCORESTAGE2:
                queueState(switch (intention) {
                    case REJECT:
                        timer.reset();
                        yield InternalState.REJECT;
                    case SCORE:
                        ready2 = okToScore2();
                        if (ready2){
                            timer.restart();
                        }
                        yield ready2 ? InternalState.POSTSCORE : InternalState.SCORESTAGE2;
                    default:
                        yield defaultIntentionHandling();
                });
                break;
            case POSTSCORE:
                if (arm.reachedTarget() && elevator.reachedTarget() && timer.hasElapsed(POSTSCORE_s)){
                    intend(Intention.IDLE);
                    timer.reset();
                    queueState(InternalState.IDLE);
                }
                break;
            case PRECLIMB:
                queueState(switch (intention) {
                    case IDLE:
                        yield InternalState.IDLE;
                    case CLIMB1:
                        yield InternalState.PRECLIMB;
                    case CLIMB2:
                        yield climb.reachedTarget() ? InternalState.CLIMB : InternalState.PRECLIMB;
                    default:
                        yield defaultIntentionHandling();

                });
                break;
            case CLIMB:
                queueState(switch (intention) {
                    case IDLE: 
                        intend(Intention.IDLE);
                        yield InternalState.IDLE;
                    default: 
                        yield defaultIntentionHandling();
                });
                break;
            default:
                queueState(defaultIntentionHandling());
                break;
        }
    }

    /**
     * Calls subsystem methods to achive the current state. Does not manage switching
     * the current state - use handleIntention().
     */
    @Override
    public void handleStateMachine() { 
        if (!booted && !isState(InternalState.DISABLED)) {
            queueState(InternalState.BOOT);
        }

        handleIntention();

        switch (getState()){
            case DISABLED:
            case REJECT:
                break;
            case BOOT:
                if (!booted) {
                    elevator.zero();
                    arm.zero();
                    booted = true;
                }

                if (elevator.reachedTarget() && arm.reachedTarget()) {
                    queueState(InternalState.IDLE);
                }
                break;
            case IDLE:

                elevator.stow();
                arm.stow();
                break; // Wait for new intention - changes in handleIntention()      
            case PRESCORE:
                drive.setPathingOverride(PathingOverride.TRACKING);
                break;
            case SCORESTAGE1:
                elevator.setHeight(ElevatorConstants.levelHeights[coralLevel]);
                arm.setShoulderPosition(ArmConstants.shoulderLevelAngles[coralLevel]);
                arm.setElbowPosition(ArmConstants.elbowLevelAngles[coralLevel]);
                break;
            case SCORESTAGE2:
                if (timer.hasElapsed(PULLBACK_TIME_s)) {
                    arm.setShoulderPosition(ArmConstants.shoulderLevelAngles[coralLevel]);
                }
                break;
            case POSTSCORE:
                if (timer.hasElapsed(POSTSCORE_s)) {
                    elevator.stow();
                    arm.stow();
                }
                break;
            case PRECLIMB:
                climb.queueState(ClimbStates.STRETCHING);
                break;
            case CLIMB:
                climb.queueState(ClimbStates.CLIMBING);
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
        Logger.recordOutput("SS/timer", timer.get());
    }

    public void intend(Intention i) {
        intention = i;
    }

    public void setReef(int coralLevel) { 
        this.coralLevel = coralLevel;
    }

    public int getReef() {
        return coralLevel;
    }

    public boolean okToScore1() {
        return arm.reachedTarget() && timer.hasElapsed(SCORE_s);
    }

    public boolean okToScore2() {
        return arm.reachedTarget() && timer.hasElapsed(PULLBACK_TIME_s + 0.05);
    }
}

