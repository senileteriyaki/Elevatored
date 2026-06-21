package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ClimberConstants;
import frc.robot.util.PhoenixUtil;

public class ClimbIOReal implements ClimbIO{
    private TalonFX left = new TalonFX(7); //as lead
    private TalonFX right = new TalonFX(8);

    private final MotionMagicVoltage request;

    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Angle> pos;
    private final StatusSignal<Current> amps;
    private final StatusSignal<AngularVelocity> vel;
    private final ElevatorFeedforward ff;
    private double ffVoltage;

    public ClimbIOReal(){
        TalonFXConfiguration config = new TalonFXConfiguration();
        request = new MotionMagicVoltage(ClimberConstants.stowAngle);
        ff = new ElevatorFeedforward(0, 0, 0, 0);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        volts = left.getMotorVoltage();
        pos = left.getPosition();
        amps = left.getStatorCurrent();
        vel = left.getVelocity();
        
        // [imagine I actually had values to config the motor]

        PhoenixUtil.tryUntilOk(5, () -> left.getConfigurator().apply(config));
        PhoenixUtil.tryUntilOk(5, () -> right.getConfigurator().apply(config));
        right.setControl(new Follower(left.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    public void updateInputs(ClimbIOInputs inputs){
        BaseStatusSignal.refreshAll(volts, amps, pos, vel);
        
        inputs.volts = volts.getValueAsDouble();
        inputs.amps = amps.getValueAsDouble();
        inputs.pos = pos.getValueAsDouble() * 360;
        inputs.vel = vel.getValueAsDouble() * 360;
    }

    @Override
    public void setVoltage(double voltage){
        ffVoltage = ff.calculate(0);
        left.setVoltage(voltage + ffVoltage);
    }

    @Override
    public void goToPos(double pos){
        ffVoltage = ff.calculate(0);
        left.setControl(request
            .withPosition(pos)
            .withFeedForward(ffVoltage));
    }

    @Override
    public void hold(double pos){
        ffVoltage = ff.calculate(0);
        left.setControl(request
            .withPosition(pos)
            .withFeedForward(ffVoltage));
    }

    @Override
    public void stop(){
        left.stopMotor();
        right.stopMotor();
    }

}
