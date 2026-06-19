package frc.robot.subsystems.climb;

public class ClimbIOReal implements ClimbIO{

    public ClimbIOReal(){
        // Real hardware initialization would go here
    }

    public void updateInputs(ClimbIOInputs inputs){
        // populate real signals
    }

    @Override
    public void setVoltage(double voltage){
        // set motor voltage
    }

    @Override
    public void goToPos(double pos){
        // motion magic or closed-loop control
    }

    @Override
    public void hold(double pos){
        // hold position
    }

    @Override
    public void stop(){
        // stop motors
    }

}
