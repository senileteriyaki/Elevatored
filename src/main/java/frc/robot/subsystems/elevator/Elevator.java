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
          io.stop();
          break;
        case IDLE:
          io.hold(ElevatorConstants.minHeight); // bruh whats with u and the hold methods
          break;
        case HOLDING:
          io.hold(target);
          break;
        case TRAVELLING:
          io.goToPos(target);
          if (Math.abs(inputs.pos - target) < ElevatorConstants.tolerance){
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
      elevator2d.set(inputs.pos); // nice u remembered
      elevator2d.periodic();
    }

    public void setTarget(double p){
        target = p;
    }

    public void setCoralLevel(int level){ // just use a method to set the specific height instead of the level thats bad. Too specific lacks polymorphism.
      setTarget(ElevatorConstants.levelHeights[level]);
      queueState(ElevatorStates.TRAVELLING);
    }

    public void setArmLigament(double deg){ // Raymond: This is a bad method. You should not be setting the arm ligament from the elevator subsystem. The elevator should not know about the arm. This is a violation of the single responsibility principle. You should have a method in the arm subsystem to set the arm ligament and call that from the command that controls both subsystems. Do it in ss or smt.
      elevator2d.setArm(deg);
    }

}
