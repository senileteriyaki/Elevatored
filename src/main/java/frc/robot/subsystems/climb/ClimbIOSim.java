package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ClimbIOSim implements ClimbIO{

    private ElevatorSim sim;
    private PIDController controller;

    private double targetPos;
    private double volts;

    public ClimbIOSim(){
        this.sim = new ElevatorSim(DCMotor.getKrakenX60(1), 3, 
                                   5, 0.02, ElevatorConstants.minHeight, 
                                   ElevatorConstants.maxHeight, true, 0, 0.1, 0);

        this.controller = new PIDController(1.0, 0, 0);
        this.targetPos = 0;
    }
    
    @Override
    public void updateInputs(ClimbIOInputs inputs){
        sim.update(Constants.globalDelta_s);

        inputs.volts = volts;
        inputs.amps = sim.getCurrentDrawAmps();
        inputs.pos = sim.getPositionMeters();
        inputs.vel = sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setVoltage(double voltage){
        volts = MathUtil.clamp(voltage, -12, 12);
        sim.setInputVoltage(volts);
    }

    @Override
    public void goToPos(double pos){
       targetPos = pos;
       setVoltage(controller.calculate(sim.getPositionMeters(), targetPos)); // Use a trapezoidal profile here as well. Try to integrate that.
    }

    @Override
    public void hold(double pos){
        targetPos = pos;
        setVoltage(controller.calculate(sim.getPositionMeters(), targetPos)); // Raymond: Redundant. Just use goToPos method here.
    }

    @Override
    public void stop(){
        setVoltage(0);
    }

}
