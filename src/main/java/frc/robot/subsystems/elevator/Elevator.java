package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private ElevatorIO io;
    private static Elevator instance;

    public Elevator(ElevatorIO io){
        super("elevator");
        this.io = io;
        
        queueState(ElevatorStates.IDLE);
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

    // TODO: implement
    public void handleStateMachine() {
      switch (getState()) {
        case DISABLED:
          break;
        case IDLE:
          break;
        case HOLDING:
          break;
        case HOMING:
          break;
        default:
          break;
      }
    }

    @Override
    public void inputPeriodic(){

    }

    @Override
    public void outputPeriodic(){

    }
}
