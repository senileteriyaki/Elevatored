package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim sim;
    private ProfiledPIDController controller;
    private ElevatorFeedforward ff;
    private Constraints constraints;

    private double volts;

    public ElevatorIOSim(){
        this.sim = new ElevatorSim(DCMotor.getKrakenX60(1), ElevatorConstants.GEAR_RATIO, 
                                   ElevatorConstants.CARRIAGE_MASS_KG, ElevatorConstants.drumRadius, ElevatorConstants.minHeight, 
                                   ElevatorConstants.maxHeight, true, ElevatorConstants.minHeight, 0.1, 0);
        this.constraints = new Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
        this.controller = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, constraints);
        controller.setTolerance(ElevatorConstants.tolerance);

        this.ff = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV);   
        controller.reset(ElevatorConstants.minHeight);
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        sim.update(Constants.globalDelta_s);

        inputs.voltage_v = volts;
        inputs.current_a = sim.getCurrentDrawAmps();
        inputs.pos_m = sim.getPositionMeters();
        inputs.vel_mps = sim.getVelocityMetersPerSecond();
    }

    @Override
    public void setVoltage(double voltage){
        volts = MathUtil.clamp(voltage, -12, 12);
        sim.setInputVoltage(volts);
    }

    @Override
    public void goToPos(double pos){
       double conVoltage = controller.calculate(sim.getPositionMeters(), pos);
       setVoltage(conVoltage + ff.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));
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
