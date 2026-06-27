package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private ElevatorIO io;
    private static Elevator instance;
    private double target;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    public final Elevator2D elevator2d;

    public Elevator(ElevatorIO io){
        super("elevator");
        this.io = io;
        target = ElevatorConstants.minHeight;
        queueState(ElevatorStates.IDLE);
        elevator2d = new Elevator2D("elevator", new Color8Bit(Color.kLavender), new Color8Bit(Color.kDarkOrange));
    }

    public static Elevator getInstance(){
        if (instance == null){
          switch (Constants.currentMode){
            case SIM:
              instance = new Elevator(new ElevatorIOSim());
              break;
            case REAL:
              instance = new Elevator(new ElevatorIOReal());
              break;
            case REPLAY:
              instance = new Elevator(new ElevatorIOReal());
              break;
          }
        }
        return instance;
    }

    public void handleStateMachine() {
      switch (getState()) {
        case DISABLED:
        case IDLE:
          io.stop();
          break;
        case HOLDING:
          io.hold(target);
          break;
        case TRAVELLING:
          io.goToPos(target);
          if (Math.abs(inputs.pos_m - target) < ElevatorConstants.tolerance){
            queueState(ElevatorStates.HOLDING);
          }
          break;
        default:
          break;
      }
    }

    @Override
    public void inputPeriodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void outputPeriodic(){
      Logger.recordOutput("Elevator/target", target);
      elevator2d.set(inputs.pos_m);
      elevator2d.periodic();
    }

    public void setTarget(double p){
      target = p;
    }

    public void setHeight(double height) {
      setTarget(height);
      queueState(ElevatorStates.TRAVELLING);
    }

}
