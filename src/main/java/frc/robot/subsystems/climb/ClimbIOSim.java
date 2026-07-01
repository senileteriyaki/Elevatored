package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ClimbIOSim implements ClimbIO{

    private ElevatorSim sim;

    private Constraints constraints;
    private ProfiledPIDController controller;
    private State goal;

    private double targetPos;
    private double volts;

    public ClimbIOSim(){
        this.sim = new ElevatorSim(DCMotor.getKrakenX60(1), 3, 
                                   5, 0.02, ElevatorConstants.minHeight, 
                                   ElevatorConstants.maxHeight, true, 0, 0.1, 0);

        this.constraints = new Constraints(ClimberConstants.MAX_VELOCITY, ClimberConstants.MAX_ACCELERATION);
        this.controller = new ProfiledPIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, constraints);
        controller.setTolerance(ClimberConstants.tolerance);

        this.targetPos = 0;
    }
    
    @Override
    public void updateInputs(ClimbIOInputs inputs){
        sim.update(Constants.globalDelta_s);

        inputs.voltage_v = volts;
        inputs.current_a = sim.getCurrentDrawAmps();
        inputs.pos_deg = sim.getPositionMeters();
        inputs.vel_dps = sim.getVelocityMetersPerSecond();
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

       setVoltage(controller.calculate(sim.getPositionMeters()));
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
