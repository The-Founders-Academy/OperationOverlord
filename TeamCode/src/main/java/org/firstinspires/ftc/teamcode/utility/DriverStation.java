package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriverStation {
    private static DriverStation driverStation;
    public Telemetry telemetry;
    public Alliance alliance;
    private HardwareMap hardwareMap;

    public enum Alliance {
        BLUE, RED, NONE
    }

    public synchronized static DriverStation getInstance() {
        if(driverStation == null) {
            driverStation = new DriverStation();
        }
        return driverStation;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void setHardwareMap(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
}
