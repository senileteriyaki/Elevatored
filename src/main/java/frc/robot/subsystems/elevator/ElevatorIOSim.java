package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO{

    public ElevatorIOSim(){

    }
    
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        // Populate simulated inputs with default/sim values
        // e.g., inputs.positionMeters = 0.0;
    }

    @Override
    public void setVoltage(double voltage){
        // Simulate setting voltage
    }

    @Override
    public void goToPos(double pos){
        // Simulate position command
    }

    @Override
    public void hold(double pos){
        // Simulate hold
    }

    @Override
    public void stop(){
        // Simulate stop
    }

}
