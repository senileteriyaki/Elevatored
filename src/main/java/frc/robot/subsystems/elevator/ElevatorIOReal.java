package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOReal implements ElevatorIO{

    private TalonFX elevatorMotor = new TalonFX(01);
    private final MotionMagicVoltage request;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Angle> pos;
    private final StatusSignal<Current> amps;
    private final StatusSignal<AngularVelocity> vel;
    private final ElevatorFeedforward ff;
    private double ffVoltage;

    public ElevatorIOReal(){

        TalonFXConfiguration config = new TalonFXConfiguration();
        request = new MotionMagicVoltage(ElevatorConstants.startPos);
        ff = new ElevatorFeedforward(0, 0, 0, 0);

        volts = elevatorMotor.getMotorVoltage();
        pos = elevatorMotor.getPosition();
        amps = elevatorMotor.getStatorCurrent();
        vel = elevatorMotor.getVelocity();
        
        // [imagine I actually had values to config the motor]

        elevatorMotor.getConfigurator().apply(config);


    }

    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(volts, amps, pos, vel);
        
        inputs.volts = volts.getValueAsDouble();
        inputs.amps = amps.getValueAsDouble();
        inputs.pos = pos.getValueAsDouble() * 360;
        inputs.vel = vel.getValueAsDouble() * 360;
    }
    
    @Override
    public void setVoltage(double voltage){
        ffVoltage = ff.calculate(0);
        elevatorMotor.setVoltage(voltage + ffVoltage);
    }

    @Override
    public void goToPos(double pos){
        ffVoltage = ff.calculate(0);
        elevatorMotor.setControl(request
            .withPosition(pos)
            .withFeedForward(ffVoltage));
    }

    @Override
    public void hold(double pos){
        ffVoltage = ff.calculate(0);
        elevatorMotor.setControl(request
            .withPosition(pos)
            .withFeedForward(ffVoltage));
    }

    @Override
    public void stop(){
        elevatorMotor.stopMotor();
    }
    
}
