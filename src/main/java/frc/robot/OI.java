package frc.robot;

import frc.robot.commands.ToggleArm;
import harkerrobolib.wrappers.XboxGamepad;

public class OI {
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    private static XboxGamepad driverGamepad;
    private static XboxGamepad operatorGamepad;

    private static OI instance;

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);
    }

    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    public static void initBindings() {
        driverGamepad.getButtonA().whenPressed(new ToggleArm());
    }

    public static OI getInstance() {
        if (instance == null) {
           instance = new OI();
        }

        initBindings();

        return instance;
    }
}