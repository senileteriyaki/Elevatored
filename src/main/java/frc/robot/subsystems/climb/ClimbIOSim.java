package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ClimbIOSim implements ClimbIO{

    private SingleJointedArmSim sim;

    private Constraints constraints;
    private ProfiledPIDController controller;
    private State goal;

    private double targetPos;
    private double volts;

    public ClimbIOSim(){
        this.sim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), 3, 
                                   5, 0.5, Units.degreesToRadians(ClimberConstants.minAngle),
                                   Units.degreesToRadians(ClimberConstants.maxAngle), true, Units.degreesToRadians(90), 0.1, 0);

        this.constraints = new Constraints(ClimberConstants.MAX_VELOCITY, ClimberConstants.MAX_ACCELERATION);
        this.controller = new ProfiledPIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, constraints);
        controller.setTolerance(ClimberConstants.tolerance);

        this.targetPos = 90;
    }
    
    @Override
    public void updateInputs(ClimbIOInputs inputs){
        sim.update(Constants.globalDelta_s);

        inputs.voltage_v = volts;
        inputs.current_a = sim.getCurrentDrawAmps();
        inputs.pos_deg = Units.radiansToDegrees(sim.getAngleRads());
        inputs.vel_dps = Units.radiansToDegrees(sim.getVelocityRadPerSec());
    }

    @Override
    public void setVoltage(double voltage){
        volts = MathUtil.clamp(voltage, -12, 12);
        sim.setInputVoltage(volts);
    }

    @Override
    public void goToPos(double pos){
       if (pos != targetPos) {
        targetPos = pos;
        goal = new State(targetPos, 0);
        controller.setGoal(goal);
       }

       setVoltage(controller.calculate(Units.radiansToDegrees(sim.getAngleRads()), targetPos));
    }

    @Override
    public void hold(double pos){
        goToPos(pos);
    }

    @Override
    public void stop(){
        setVoltage(0);
    }

}
