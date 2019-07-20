package frc.robot;

import frc.robot.commands.wrist.MoveWristMotionMagic;
import frc.robot.commands.wrist.ZeroWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.MoveElevatorAndWrist;
import frc.robot.commands.arm.ToggleArm;
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
        //MoveElevatorAndWrist backHatch = new MoveElevatorAndWrist(7320, Wrist.HORIZONTAL_BACK);
        
        //MoveElevatorAndWrist backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
        //MoveElevatorAndWrist frontShipAndLoading = new MoveElevatorAndWrist(17600, 120);
        
        //MoveElevatorAndWrist backRocketFirstCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketSecondCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketFirstHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketSecondHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist frontRocketSecondCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist frontRocketSecondHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);

        driverGamepad.getButtonA().whenPressed(new ZeroElevator());
        driverGamepad.getButtonX().whenPressed(new ZeroWrist());
        driverGamepad.getButtonB().whenPressed(new ToggleArm());
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