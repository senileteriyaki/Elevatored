package frc.robot.subsystems.tracking;

import frc.robot.subsystems.vision.Vision;

public class TrackingIOSim implements TrackingIO {
    private Vision vision;
    public TrackingIOSim(){
        vision = Vision.getInstance();
    }

    public void updateInputs(TrackingIOInputs inputs) {
        inputs.tx = vision.getTargetX();
        inputs.ty = vision.getTargetY();
        inputs.tz = 0;
        inputs.ta = vision.getTargetArea();
        inputs.tv = vision.hasTarget() ? 1 : 0;
    }
    

    @Override
    public void setValidIds(double[] valid){

    }
}
