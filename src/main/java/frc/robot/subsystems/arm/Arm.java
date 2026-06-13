package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.StateMachineSubsystemBase;

public class Arm extends StateMachineSubsystemBase<ArmStates> {
    private ArmIO io;
    private static Arm instance;

    private final Arm2d arm2d;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private double elbowTarget;
    private double shoulderTarget;

    public Arm(ArmIO io){
        super("arm");
        this.io = io;
        this.elbowTarget = this.shoulderTarget = ArmConstants.minAngle;
        this.arm2d = new Arm2d("arm", new Color8Bit(Color.kBlanchedAlmond));
        queueState(ArmStates.IDLE);
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
      switch (getState()) {
        case DISABLED:
          io.stopElbow();
          io.stopShoulder();
          break;
        case IDLE:
          io.holdElbow(ArmConstants.minAngle);
          io.holdShoulder(ArmConstants.minAngle);
          break;
        case HOLDING:
          io.holdElbow(elbowTarget);
          io.holdShoulder(shoulderTarget);
        case TRAVELLING:
          io.goToElbowPos(elbowTarget);
          io.goToShoulderPos(shoulderTarget);

          if (Math.abs(inputs.elbowPos - elbowTarget) < ArmConstants.tolerance &&
              Math.abs(inputs.shoulderPos - shoulderTarget) < ArmConstants.tolerance) {
            queueState(ArmStates.HOLDING);
          }
          break;
        default:
          break;
      }
    }

    @Override
    protected void inputPeriodic(){
      io.updateInputs(inputs);
      Logger.processInputs("Arm", inputs);
    }

    @Override
    protected void outputPeriodic(){
      Logger.recordOutput("Arm/Elbow/targetAngleDegrees", elbowTarget);
      Logger.recordOutput("Arm/Shoulder/targetAngleDegrees", shoulderTarget);
      arm2d.setElbow(elbowTarget);
      arm2d.setShoulder(shoulderTarget);
      arm2d.periodic();
    }

    public void setElbowTarget(double target) {
      this.elbowTarget = target;
    }

    public void setShoulderTarget(double target) {
      this.shoulderTarget = target;
    }
}
