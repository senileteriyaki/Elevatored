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
          io.holdShoulder(ArmConstants.minAngle); // Raymond: BRO hold elbow/shoulder just calls goToElbowPos/goToShoulderPos with the current position, so like why??????? just use the same method. Also, like why are we holding the minAngle when idle? Just stopElbow() and stopShoulder() here.
          break;
        case HOLDING:
          io.holdElbow(elbowTarget);
          io.holdShoulder(shoulderTarget); // Raymond: Just use goToPos methods here. 
        case TRAVELLING:
          io.goToElbowPos(elbowTarget);
          io.goToShoulderPos(shoulderTarget);

          if (Math.abs(inputs.elbowPos - elbowTarget) < ArmConstants.tolerance &&
              Math.abs(inputs.shoulderPos - shoulderTarget) < ArmConstants.tolerance) { // Raymond prob want different tolerances for shoulder and elbow
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
      arm2d.set(inputs.shoulderPos);
      arm2d.periodic();
    }

    public void setElbowTarget(double target) {
      this.elbowTarget = target;
    }

    public void setShoulderTarget(double target) {
      this.shoulderTarget = target;
    }

    public double getElbowPos(){
      return inputs.elbowPos;
    }

    public void setCoralLevel(int level) { // Raymond: ok sure but like in here you don't want full logic for whole bot. Cuz Arm doesn't like determine coral levles ykwim, its more elevator? also you should just have a general method for like trackToPosition that sets target Angle and queues States. Your states should also check here or else its gonna fluctuate between holding and travelling. Your elbow and shoulder should have seperate methods too. 
      elbowTarget = ArmConstants.elbowLevelAngles[level];
      shoulderTarget = ArmConstants.shoulderLevelAngles[level];
      queueState(ArmStates.TRAVELLING);
    }
}
