package frc.robot.subsystems.climb;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.Constants;

public class ClimbIOSim implements ClimbIO{

    private SingleJointedArmSim sim;

    private Constraints constraints;
    private ProfiledPIDController controller;
    private State goal;

    private ArmFeedforward ff;
    private double targetPos;
    private double volts;

    public ClimbIOSim(){
        this.sim = new SingleJointedArmSim(DCMotor.getKrakenX60(2), 50, 
                                   1.5, 0.5, Units.degreesToRadians(ClimberConstants.minAngle),
                                   Units.degreesToRadians(ClimberConstants.maxAngle), true, Units.degreesToRadians(ClimberConstants.stowAngle));

        this.constraints = new Constraints(ClimberConstants.MAX_VELOCITY, ClimberConstants.MAX_ACCELERATION);
        this.controller = new ProfiledPIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, constraints);
        controller.setTolerance(ClimberConstants.tolerance);

        ff = new ArmFeedforward(ClimberConstants.kS, ClimberConstants.kG, ClimberConstants.kV); 
        this.targetPos = ClimberConstants.stowAngle;

        this.controller.reset(ClimberConstants.stowAngle);

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
        double conVoltage = controller.calculate(Units.radiansToDegrees(sim.getAngleRads()), pos);
        setVoltage(conVoltage + ff.calculate(Units.degreesToRadians(controller.getSetpoint().position), Units.degreesToRadians(controller.getSetpoint().velocity)));
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
