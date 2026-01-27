package frc.robot.subsystems.tracking;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TrackingIOLimelight implements TrackingIO {

    private final NetworkTable table;
    private double prevLat = 0;

    Debouncer commsDebouncer = new Debouncer(1.0,DebounceType.kRising);

    public TrackingIOLimelight(String name) {
        table = NetworkTableInstance.getDefault().getTable(name);
    }
    
    public void updateInputs(TrackingIOInputs inputs) {
        
        double[] t2d = table.getEntry("t2d").getDoubleArray(new double[17]);
        
        inputs.ty = t2d[5];
        inputs.tx = t2d[4];
        inputs.ta = t2d[8];
        inputs.tv = t2d[0];

        inputs.hb = table.getEntry("hb").getDouble(0);

        double[] targetPose_cameraSpace = table.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
        inputs.tz = targetPose_cameraSpace[2];
        inputs.tx_3d = targetPose_cameraSpace[0];
        
        double lat = table.getEntry("tl").getDouble(0);
        inputs.connected = !commsDebouncer.calculate(lat == prevLat);
        prevLat = lat;
    }

    @Override
    public void setpl(int pl) {
        table.getEntry("pipeline").setNumber(pl);
    }

    @Override
    public void setValidIds(double[] valid) {
        System.out.println("setting valid array");
        table.getEntry("fiducial_id_filters_set").setDoubleArray(valid);
    }
}