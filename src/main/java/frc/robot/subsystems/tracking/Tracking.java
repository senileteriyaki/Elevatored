package frc.robot.subsystems.tracking;

import frc.robot.Constants;
import frc.robot.subsystems.StateMachineSubsystemBase;
import frc.robot.util.Util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class Tracking extends StateMachineSubsystemBase<TrackingStates> {

  public static final int PIPELINE = 0;

  private final double ROT_KP = 4;

  private static Tracking instance;

  private final TrackingIO io;
  private final TrackingIOInputsAutoLogged inputs = new TrackingIOInputsAutoLogged();

  private boolean enabled = true;

  private double currTxTarget = 0.0;
  private double currTzTarget = 0.0;

  private Tracking(TrackingIO io) {
    super("Tracking");
    this.io = io;
    queueState(TrackingStates.IDLE);
  }

  public static Tracking getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case REAL:
          instance = new Tracking(new TrackingIOLimelight("limelight-mvrt"));
          break;

        case SIM:
        case REPLAY:
          instance = new Tracking(new TrackingIO() {});
          break;

        default:
          break;
      }
    }
    return instance;
  }


  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Tracking", inputs);
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {

      case DISABLED:
        break;

      case IDLE:
        if (enabled && hasTarget()) {
          queueState(TrackingStates.TRACKING);
        }
        break;

      case TRACKING:
        if (!enabled) {
          queueState(TrackingStates.DISABLED);
        } else if (!hasTarget()) {
          queueState(TrackingStates.IDLE);
        }
        break;

      default:
        break;
    }
  }


  @Override
  public void outputPeriodic() {
    Logger.recordOutput("Tracking/Enabled", enabled);
    Logger.recordOutput("Tracking/State", getState().toString());
    Logger.recordOutput("Tracking/TxTarget", currTxTarget);
    Logger.recordOutput("Tracking/TzTarget", currTzTarget);
  }

  public ChassisSpeeds getTrackingSpeeds(double currAngle, double maxVel) {
    double rotSpeed = 0;
    double sideSpeed = 0;
    double forwardSpeed = 0;
    
    double txTol = 0;
    double tzTol = 0;

    double fMultiplier = -0.5; 
    double sMultiplier = 0.5; 
    double error = 0;
    double targetAngle = 180; //Could set to a tid switch case, seeing which ID and which angle corresponds
    
    if (currAngle < 0) { // This means that its closer to the left
      error = -(targetAngle - Math.abs(currAngle));
    } else { // this means that its closer to the right
      error = (targetAngle - Math.abs(currAngle));
    }

    rotSpeed = Math.abs(error) > 0.5 ? (int)(1.5 * error) : 0;

    if (error < 30 && hasTarget()) {
        TrackingLocation current = getLocation();
        double tx_error = txTol - (current.getTx());
        System.out.println(current.getTz());
        double tz_error = tzTol - (current.getTz()-1);
        
        double calculatedSpeedSide = 0;
        double calculatedSpeedForward = 0;
        double maxSpeed = maxVel;

        calculatedSpeedForward = SharkFinConfigs.BASE_SHARKFIN.calculate(tz_error, tx_error);
        calculatedSpeedSide = SharkFinConfigs.BASE_SHARKFIN.calculate(tx_error, tz_error);

        // clamp
        if (Math.abs(tz_error) > 0.02 || Math.abs(tx_error) > 0.02) {
            sideSpeed = Util.limit(calculatedSpeedSide, maxSpeed);
            forwardSpeed = Util.limit(calculatedSpeedForward, maxSpeed);
        }
    }
    // Cosine compensation
    double comp = Math.cos(Units.degreesToRadians(error));
    // System.out.println(fMultiplier * forwardSpeed * comp);
    return new ChassisSpeeds(fMultiplier * forwardSpeed * comp, sMultiplier * sideSpeed * comp, rotSpeed);
}

    public ChassisSpeeds getTrackingSpeeds(double targetAngle, double currAngle, double maxVel, double tZ_offset, double tX_offset) {
        double rotSpeed = 0;
        double sideSpeed = 0;
        double forwardSpeed = 0;
        
        if (!hasTarget()) {
            return new ChassisSpeeds();
        }
        double txTol = 0;
        double tzTol = 0;

        double fMultiplier = 1; 
        double sMultiplier = -1; 

        double error = targetAngle - currAngle;
        rotSpeed = Math.abs(error) > 0.5 ? ROT_KP * error : 0;

        if (error < 10) {
            TrackingLocation current = getLocation();
            double tx_error = txTol - (current.getTx() - tZ_offset);
            double tz_error = tzTol - (current.getTz() - tX_offset);
            
            double calculatedSpeedSide = 0;
            double calculatedSpeedForward = 0;
            double maxSpeed = maxVel;
    
            calculatedSpeedForward = SharkFinConfigs.BASE_SHARKFIN.calculate(tz_error, tx_error);
            calculatedSpeedSide = SharkFinConfigs.BASE_SHARKFIN.calculate(tx_error, tz_error);

            // clamp
            if (Math.abs(tz_error) > 0.02 || Math.abs(tx_error) > 0.02) {
                sideSpeed = Util.limit(calculatedSpeedSide, maxSpeed);
                forwardSpeed = Util.limit(calculatedSpeedForward, maxSpeed);
            }
        }
        // Cosine compensation
        double comp = Math.cos(Units.degreesToRadians(error));
        return new ChassisSpeeds(fMultiplier * forwardSpeed * comp, sMultiplier * sideSpeed * comp, rotSpeed);
    }

    public ChassisSpeeds getTrackingSpeeds(double targetAngle, double currAngle, double maxVel) {
        double rotSpeed = 0;
        double sideSpeed = 0;
        double forwardSpeed = 0;
        
        if (!hasTarget()) {
            return new ChassisSpeeds();
        }
        double txTol = 0;
        double tzTol = 0;

        double fMultiplier = 1; 
        double sMultiplier = -1; 

        double error = targetAngle - currAngle;
        rotSpeed = Math.abs(error) > 0.5 ? ROT_KP * error : 0;

        if (error < 10) {
            TrackingLocation current = getLocation();
            double tx_error = txTol - (current.getTx());
            double tz_error = tzTol - (current.getTz());
            
            double calculatedSpeedSide = 0;
            double calculatedSpeedForward = 0;
            double maxSpeed = maxVel;
    
            calculatedSpeedForward = SharkFinConfigs.BASE_SHARKFIN.calculate(tz_error, tx_error);
            calculatedSpeedSide = SharkFinConfigs.BASE_SHARKFIN.calculate(tx_error, tz_error);

            // clamp
            if (Math.abs(tz_error) > 0.02 || Math.abs(tx_error) > 0.02) {
                sideSpeed = Util.limit(calculatedSpeedSide, maxSpeed);
                forwardSpeed = Util.limit(calculatedSpeedForward, maxSpeed);
            }
        }
        // Cosine compensation
        double comp = Math.cos(Units.degreesToRadians(error));
        System.out.println(fMultiplier * forwardSpeed * comp);

        return new ChassisSpeeds(fMultiplier * forwardSpeed * comp, sMultiplier * sideSpeed * comp, rotSpeed);
    }


    public ChassisSpeeds getTrackingSpeeds(double currAngle) {
      double rotSpeed = 0;
      double targetAngle = 0;
      double error = 0;
    
      double targetStartAngleRight = -5;
      double targetStartAngleLeft = 5;

      if (hasTarget()) {
        error = targetAngle - getTx();

        rotSpeed = Math.abs(error) > 0.5 ?(int) (ROT_KP * error) : 0;
      }
      // } else {
      //   if (currAngle >= targetStartAngleLeft) { // This means that its closer to the left
      //     System.out.println(currAngle);
      //     rotSpeed = -(currAngle - targetStartAngleLeft) * ROT_KP;
      //   } else if (currAngle <= targetStartAngleRight) { // this means that its closer to the right
      //     rotSpeed = -(currAngle - targetStartAngleRight) * ROT_KP;
      //   }
      // }
      return new ChassisSpeeds(0,0,rotSpeed);
  }

  public boolean hasTarget() {
    return inputs.tv == 1;
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public double getTx() {
    return inputs.tx;
  }

  public double getTa() {
    return inputs.ta;
  }

  public double getTaLinear() {
    return Math.sqrt(inputs.ta);
  }

  public double getTz() {
    return inputs.tz;
  }

  public double get3dTx() {
    return inputs.tx_3d;
  }

  public void setTxTarget(double tx) {
    currTxTarget = tx;
  }

  public void setTzTarget(double tz) {
    currTzTarget = tz;
  }

  public double getTxTarget() {
    return currTxTarget;
  }

  public double getTzTarget() {
    return currTzTarget;
  }

  public TrackingLocation getLocation() {
    return new TrackingLocation(get3dTx(), getTz());
  }


  public void enable() {
    enabled = true;
    queueState(TrackingStates.IDLE);
  }

  public void disable() {
    enabled = false;
    queueState(TrackingStates.DISABLED);
  }

  public void toggleEnabled() {
    enabled = !enabled;
    queueState(enabled ? TrackingStates.IDLE : TrackingStates.DISABLED);
  }

  public boolean getEnabled() {
    return enabled;
  }

  public void setPipeline(int pipeline) {
    io.setpl(pipeline);
  }

  public void setValidIds(double[] validIds) {
    io.setValidIds(validIds);
  }
}
