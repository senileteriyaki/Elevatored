package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO{

    public ArmIOSim(){

    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs){
        // populate simulated inputs
    }

    @Override
    public void setVoltage(double voltage){
        // simulate voltage
    }

    @Override
    public void goToPos(double pos){
        // simulate go to position
    }

    @Override
    public void hold(double pos){
        // simulate hold
    }

    @Override
    public void stop(){
        // simulate stop
    }
}
