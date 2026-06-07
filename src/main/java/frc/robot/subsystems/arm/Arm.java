package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Arm extends StateMachineSubsystemBase<ArmStates> {
    private ArmIO io;
    private static Arm instance;

    public Arm(ArmIO io){
        super("arm");
        this.io = io;

    }
    public static Arm getInstance(){
        if (instance == null){
        switch (Constants.currentMode){
          case SIM:
            instance = new Arm(new ArmIOSim());
            break;
          case REAL:
            instance = new Arm(new ArmIOReal());
            break;
          case REPLAY:
            instance = new Arm(new ArmIOReal());
            break;
        }
      }
        return instance;
    }

    public void handleStateMachine(){

    }

    protected void inputPeriodic(){

    }

    protected void outputPeriodic(){

    }
}
