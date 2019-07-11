package frc.robot;

import frc.robot.auton.StraightLinePath;
import frc.robot.commands.arm.ToggleArm;
import frc.robot.commands.extender.ToggleExtender;
import frc.robot.commands.flower.ToggleFlower;
import frc.robot.commands.wrist.ZeroWrist;
import frc.robot.commands.drivetrain.DriveWithMotionProfile;
import frc.robot.commands.elevator.MoveElevatorMotionMagic;
import frc.robot.commands.elevator.ZeroElevator;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/14/19
 */
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

        initBindings();
    }

    public void initBindings() {
        driverGamepad.getButtonA().whenPressed(new ZeroElevator());
        driverGamepad.getButtonB().whenPressed(new MoveElevatorMotionMagic(15000, 3));
        driverGamepad.getButtonY().whenPressed(new MoveElevatorMotionMagic(2000, 3));
        driverGamepad.getButtonX().whenPressed(new ZeroWrist());
        //driverGamepad.getButtonY().whenPressed(new DriveWithMotionProfile(StraightLinePath.leftTrajectory, StraightLinePath.rightTrajectory, 0.01));
    }

    public XboxGamepad getDriverGamepad() {
        return driverGamepad;
    }

    public XboxGamepad getOperatorGamepad() {
        return operatorGamepad;
    }

    public static OI getInstance() {
        if (instance == null) {
           instance = new OI();
        }
        return instance;
    }
}