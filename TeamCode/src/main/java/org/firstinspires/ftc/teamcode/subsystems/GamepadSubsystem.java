package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class GamepadSubsystem extends SubsystemBase {
    private GamepadEx m_gamepad;

    public GamepadSubsystem(GamepadEx gamepad) {
        m_gamepad = gamepad;
    }

    public GamepadButton buttonX() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.X);
    }

    public GamepadButton buttonY() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.Y);
    }

    public GamepadButton buttonA() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.A);
    }

    public GamepadButton buttonB() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.B);
    }

    public GamepadButton dpadDown() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN);
    }

    public GamepadButton dpadUp() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP);
    }

    public GamepadButton dpadLeft() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT);
    }

    public GamepadButton dpadRight() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT);
    }

    public double getLeftX() {
        return m_gamepad.getLeftX();
    }

    public double getLeftY() {
        return m_gamepad.getLeftY();
    }

    public double getRightX() {
        return m_gamepad.getRightX();
    }

    public double getleftAngleRadians() {
        return Math.atan2(getLeftY(), getLeftX());
    }

    public double getLeftMagnitude() {
        return Math.sqrt(Math.pow(getLeftX(), 2) + Math.pow(getLeftY(), 2));
    }

    public double leftTrigger() {
        return m_gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    public double rightTrigger() {
        return m_gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }

    public GamepadButton bumperLeft() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER);
    }

    public GamepadButton bumperRight() {
        return m_gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER);
    }
}
