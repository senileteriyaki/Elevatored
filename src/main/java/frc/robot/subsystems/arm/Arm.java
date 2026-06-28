package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
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

    /*
     * bhlcusd: keeping seperate hold methods for semantic reasons, especially since HOLDING/TRAVELING would behave the same except for the name
     */
    public void handleStateMachine(){
      switch (getState()) {
        case DISABLED:
          io.stopElbow();
          io.stopShoulder();
          break;
        case IDLE:
          io.stopElbow();
          io.stopShoulder();
          break;
        case HOLDING:
          io.holdElbow(elbowTarget); 
          io.holdShoulder(shoulderTarget);
        case TRAVELLING:
          io.goToElbowPos(elbowTarget);
          io.goToShoulderPos(shoulderTarget);

          if (Math.abs(inputs.elbowPos_deg - elbowTarget) < ArmConstants.elbowTolerance &&
              Math.abs(inputs.shoulderPos_deg - shoulderTarget) < ArmConstants.shoulderTolerance) {
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
      arm2d.set(inputs.shoulderPos_deg);
      arm2d.periodic();
    }

    public void setElbowTarget(double target) {
      this.elbowTarget = target;
    }

    public void setShoulderTarget(double target) {
      this.shoulderTarget = target;
    }

    public double getElbowPos(){
      return inputs.elbowPos_deg;
    }

    public void trackToPosition(double elbowAngle, double shoulderAngle) {
      setElbowTarget(elbowAngle);
      setShoulderTarget(shoulderAngle);
      queueState(ArmStates.TRAVELLING);
    }

    /*
     * FEEDBACK:
     * 
     * Raymond: ok sure but like in here you don't want full logic for whole bot. Cuz Arm doesn't like
     * determine coral levles ykwim, its more elevator? also you should just have a general method for 
     * like trackToPosition that sets target Angle and queues States. Your states should also check here 
     * or else its gonna fluctuate between holding and travelling. Your elbow and shoulder should have 
     * seperate methods too. 
     */
    public void setCoralLevel(int level) {
      elbowTarget = ArmConstants.elbowLevelAngles[level];
      shoulderTarget = ArmConstants.shoulderLevelAngles[level];
      queueState(ArmStates.TRAVELLING);
    }
}
