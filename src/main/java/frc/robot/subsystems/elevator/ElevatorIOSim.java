package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim sim;
    private ProfiledPIDController controller;
    private State setpoint;
    private Constraints constraints;

    private double targetHeight;
    private double volts;

    private ElevatorFeedforward ff;

    public ElevatorIOSim(){
        this.sim = new ElevatorSim(DCMotor.getKrakenX60(1), 3, 
                                   5, ElevatorConstants.drumRadius, ElevatorConstants.minHeight, 
                                   ElevatorConstants.maxHeight, true, ElevatorConstants.minHeight, 0.1, 0);
        this.constraints = new Constraints(ElevatorConstants.maxVelocity, ElevatorConstants.maxAcceleration);
        this.controller = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, constraints);
        controller.setTolerance(0.05);
        this.targetHeight = ElevatorConstants.minHeight;

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
       if (pos != targetHeight) {
        targetHeight = pos;
       }

       double conVoltage = controller.calculate(sim.getPositionMeters(), pos);

       setpoint = controller.getSetpoint();
       setVoltage(conVoltage + ff.calculate(setpoint.velocity));

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
