package frc.robot;

import harkerrobolib.wrappers.XboxGamepad;

public class OI {
    private XboxGamepad driverGamepad;

    private OI()
    {
        driverGamepad = new XboxGamepad(port);
    }
}