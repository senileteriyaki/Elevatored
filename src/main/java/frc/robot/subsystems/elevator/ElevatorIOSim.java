package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim sim;
    private PIDController controller;
    private TrapezoidProfile profile;
    private Constraints constraints;
    private State setPoint;
    private State startPoint;
    private State goal;
    private Timer timer;

    private double targetHeight;
    private double volts;
    private ElevatorIOInputs inputs;

    public ElevatorIOSim(){
        this.sim = new ElevatorSim(DCMotor.getKrakenX60(1), 3, 
                                   5, 0.02, ElevatorConstants.minHeight, 
                                   ElevatorConstants.maxHeight, true, 0, 0.1, 0);

        this.controller = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        controller.setTolerance(0.05);
        this.targetHeight = ElevatorConstants.minHeight;

        this.constraints = new Constraints(23.4, 3.1);
        this.profile = new TrapezoidProfile(constraints);
        this.setPoint = this.startPoint = this.goal = new State(targetHeight, 0);
        this.timer = new Timer();
        timer.start();
    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        sim.update(Constants.globalDelta_s);

        inputs.voltage_v = volts;
        inputs.current_a = sim.getCurrentDrawAmps();
        inputs.pos_m = sim.getPositionMeters();
        inputs.vel_mps = sim.getVelocityMetersPerSecond();

        this.inputs = inputs;
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

       if (pos != targetHeight) {
        targetHeight = pos;
        goal = new State(targetHeight, 0);
        startPoint = new State(inputs.pos_m, inputs.vel_mps);
        timer.reset();
       }

       setPoint = profile.calculate(timer.get(), startPoint, goal);
       controller.setSetpoint(setPoint.position);
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
