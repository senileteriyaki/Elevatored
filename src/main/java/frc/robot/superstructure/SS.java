package frc.robot.superstructure;

import java.util.Arrays;

import org.littletonrobotics.junction.Logger;

import com.fasterxml.jackson.databind.ser.BeanSerializer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbStates;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.tracking.Tracking;
import frc.robot.subsystems.vision.Vision;
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
    private static Climb climb;
    private static Tracking tracking;
    private static Vision vision;

    private int coralLevel = 3;

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
        tracking = Tracking.getInstance();
        vision = Vision.getInstance();


        tracking.setValidIds(new double[] {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21}); //reef only
    }

    @Override
    public void inputPeriodic() {}

    public InternalState defaultIntentionHandling() {
        return switch (intention){
            case IDLE -> InternalState.IDLE;
            case PRESCORE -> InternalState.PRESCORE;
            case SCORE -> InternalState.SCORE1;
            case CLIMB -> InternalState.CLIMBING;
            default -> InternalState.IDLE;
        };
    }

    private void handleIntention() {

    }

    // you don't handle actual internal states, you mostly only queue the score states. You basically never go back to idle when intended. 
    @Override
    public void handleStateMachine() {
        if (!booted && !isState(InternalState.DISABLED)) {
            queueState(InternalState.BOOT);
        }


    }
    @Override
    public final void outputPeriodic() {
        Logger.recordOutput("SS/Booted?", booted);
        Logger.recordOutput("SS/Intention", intention);
    }

    public void intend(Intention i) {
        intention = i;
    }

    public void setReef(int l) { 
        coralLevel = l;
    }

}

