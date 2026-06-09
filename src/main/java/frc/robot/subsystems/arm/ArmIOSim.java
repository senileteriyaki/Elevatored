package frc.robot.subsystems.arm;

public class ArmIOSim implements ArmIO{

    public ArmIOSim(){

    }
    
    @Override
    public void updateInputs(ArmIOInputs inputs){
        // populate simulated inputs
    }

   @Override
    public void setElbowVoltage(double voltage){
    }

    @Override
    public void setShoulderVoltage(double voltage) {

    }

    @Override
    public void goToElbowPos(double pos){
    }

    @Override
    public void goToShoulderPos(double pos) {

    }

    @Override
    public void holdElbow(double pos){
    }

    @Override
    public void holdShoulder(double pos) {

    }

    @Override
    public void stopElbow(){
    }

    @Override
    public void stopShoulder() {
        
    }
}
