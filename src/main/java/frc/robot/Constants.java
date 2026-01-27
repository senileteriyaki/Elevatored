// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    public static final double globalDelta_s = 0.02; // 0.02 = 50Hz
    public static final double globalDelta_Hz = 1.0 / globalDelta_s; // 0.02 = 50Hz

    private static boolean allianceFound = false;
    private static boolean isRed = true;

    public static boolean isRedAlliance() {
        if (!allianceFound && DriverStation.getAlliance().isPresent()) {
            isRed = DriverStation.getAlliance().get() == Alliance.Red;
            allianceFound = false;
        }
        return isRed;
    }

    public static final boolean verboseLogging = true;

    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static class IndexConstants {
        // motors
        
    }

    public static class ShooterConstants {
        // motors
        public static final int bottomFlyID = 67; // not inverted
        public static final int topFlyID = 67; // identical to bottomfly
        public static final int indexMotorID = 1;
        
        // canrange + debouncer stsuff
        public static final int canRangeID = 2;
        public static final double debounceTime = 0.01; // CHANGE LATER
        

        public static final double timeNeededElapsed = 5.0; // in seconds

        // Motion Magic/PID constants for fly wheels. Might get deprecated.
        public static final double kP = 1.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kA = 0.0;

        // Motion Magic/PID constants for index motor. Might get deprecated.
        public static final double indexKP = 0;
        public static final double indexKI = 0;
        public static final double indexKD = 0;

        public static final double indexKS = 0;
        public static final double indexKV = 0;
        public static final double indexKA = 0;

        // Motor velocities that we want constant. Nishka will give?
        public static final double targetIndexVel = 0.0;
        public static final double targetFlyVel = 10.0;
    }
}
