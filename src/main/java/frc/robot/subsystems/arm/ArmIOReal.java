package frc.robot.subsystems.arm;

public class ArmIOReal implements ArmIO{
    
    public ArmIOReal(){

    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        // TODO: read real hardware sensors and populate inputs
    }

    @Override
    public void setVoltage(double voltage){
        // TODO: apply voltage to arm motor
    }

    @Override
    public void goToPos(double pos){
        // TODO: closed-loop position
    }

    @Override
    public void hold(double pos){
        // TODO: hold position
    }

    @Override
    public void stop(){
        // TODO: stop motor
    }
}
