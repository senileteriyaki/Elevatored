package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorIOReal implements ElevatorIO{

    private TalonFX elevatorMotor = new TalonFX(01);

    public ElevatorIOReal(){

    }

    public void updateInputs(ElevatorIOInputs inputs){
        // TODO: read sensors and populate inputs (position, velocity, etc.)
        // Example:
        // inputs.positionMeters = elevatorMotor.getPosition();
    }
    
    @Override
    public void setVoltage(double voltage) {
        // TODO: set motor output in volts
        // elevatorMotor.setVoltage(voltage);
    }

    @Override
    public void goToPos(double pos) {
        // TODO: implement closed-loop position control
    }

    @Override
    public void hold(double pos) {
        // TODO: hold position (maybe PID)
    }

    @Override
    public void stop() {
        // elevatorMotor.setVoltage(0);
    }
    
}
