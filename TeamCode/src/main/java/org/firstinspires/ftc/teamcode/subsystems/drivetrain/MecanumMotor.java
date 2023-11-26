package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

/**
 * The MecanumMotor class uses a PID loop to ensure the physical mecanum wheel maintains a certain velocity
 * This class is redundant because MotorEx automatically uses the associated encoder port to control velocity using both open and closed loop control. Remove this class at some point.
 */
public class MecanumMotor {
    private MotorEx m_motor;

    public MecanumMotor(MotorEx motor) {
        m_motor = motor;
        m_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public double getVelocity() {
        return m_motor.getVelocity();
    }

    public void setTargetVelocity(double targetVelocity) {
        m_motor.setVelocity(targetVelocity);
    }


}