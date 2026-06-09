package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim sim;
    private PIDController controller;

    private double targetHeight;
    private double volts;

    public ElevatorIOSim(){
        this.sim = new ElevatorSim(DCMotor.getKrakenX60(1), 2, 
                                   5, 0.02, ElevatorConstants.minHeight, 
                                   ElevatorConstants.maxHeight, true, 0, 0.1, 0);

        this.controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        this.targetHeight = ElevatorConstants.minHeight;
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
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
       targetHeight = pos;
       setVoltage(controller.calculate(sim.getPositionMeters(), targetHeight));
    }

    @Override
    public void hold(double pos){
        targetHeight = pos;

        setVoltage(controller.calculate(sim.getPositionMeters(), targetHeight));
    }

    @Override
    public void stop(){
        setVoltage(0);
    }

}
