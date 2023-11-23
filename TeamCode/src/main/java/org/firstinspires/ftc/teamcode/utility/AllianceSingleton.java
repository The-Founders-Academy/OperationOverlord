package org.firstinspires.ftc.teamcode.utility;

public class AllianceSingleton {
    public enum Alliance {
        RED, BLUE
    }
    public Alliance m_alliance;

    private static AllianceSingleton allianceSingletonInstance = null;

    // This function is synchronized because it may be called by different classes at unpredictable times
    public static synchronized AllianceSingleton getInstance() {
        if(allianceSingletonInstance == null) {
            allianceSingletonInstance = new AllianceSingleton();
        }
        return allianceSingletonInstance;
    }

    private AllianceSingleton() {
        m_alliance = Alliance.BLUE;
    }

    public Alliance getAlliance() {
        return allianceSingletonInstance.m_alliance;
    }

    public void setAlliance(Alliance alliance) {
        allianceSingletonInstance.m_alliance = alliance;
    }

}
