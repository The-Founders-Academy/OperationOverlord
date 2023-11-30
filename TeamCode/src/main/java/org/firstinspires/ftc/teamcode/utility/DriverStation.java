package org.firstinspires.ftc.teamcode.utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriverStation {
    private static DriverStation driverStation;
    public Telemetry telemetry;
    public Alliance alliance;

    public enum Alliance {
        BLUE, RED, NONE
    }

    public synchronized static DriverStation getInstance() {
        if(driverStation == null) {
            driverStation = new DriverStation();
        }
        return driverStation;
    }

}
