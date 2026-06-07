package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private ElevatorIO io;
    private static Elevator instance;

    public Elevator(ElevatorIO io){
        super("elevator");
        this.io = io;
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

    public void handleStateMachine(){

    }

    public void inputPeriodic(){

    }

    public void outputPeriodic(){

    }
}
