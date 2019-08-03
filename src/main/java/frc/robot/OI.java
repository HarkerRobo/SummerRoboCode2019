package frc.robot;

import frc.robot.commands.wrist.ZeroWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.MoveElevatorAndWrist;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.arm.ToggleArm;
import frc.robot.commands.drivetrain.DriveWithLimelight;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.extender.ToggleExtender;
import frc.robot.commands.flower.ToggleFlower;
import harkerrobolib.commands.ConditionalCommand;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/14/19
 */
public class OI {
    public static final DemoMode mode = DemoMode.NORMAL;
    
    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    private static XboxGamepad driverGamepad;
    private static XboxGamepad operatorGamepad;

    private static OI instance;

    public enum DemoMode {
        /**
         * Normal Controls and Speeds for Testing or Competitions
         */
        NORMAL, 
        /**
         * Safe Controls and Speeds for Demos where others will be driving
         */
        SAFE
    }

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);

        initBindings();
    }

    public void initBindings() {
        MoveElevatorAndWrist groundCargo = new MoveElevatorAndWrist(100, -150);
        
        MoveElevatorAndWrist backHatch = new MoveElevatorAndWrist(7320, Wrist.HORIZONTAL_BACK);
        
        MoveElevatorAndWrist backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
        MoveElevatorAndWrist frontShipAndLoading = new MoveElevatorAndWrist(17600, 120);
        
        //MoveElevatorAndWrist backRocketFirstCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketSecondCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketFirstHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketSecondHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist frontRocketSecondCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist frontRocketSecondHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);

        if (mode == DemoMode.NORMAL) {
            driverGamepad.getButtonA().whenPressed(new ZeroElevator());
            driverGamepad.getButtonX().whenPressed(new ZeroWrist());
            driverGamepad.getButtonB().whenPressed(new ToggleArm());

            driverGamepad.getUpDPadButton().whenPressed(backShipAndLoading);
            driverGamepad.getDownDPadButton().whenPressed(groundCargo);
            driverGamepad.getRightDPadButton().whenPressed(frontShipAndLoading);
            //driverGamepad.getButtonY().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight, 10));
        }
        else {
            //Operator Controller (For guest) Left Joystick controls Drivetrain (10% speed)
            //Driver Controller (For Robotics Member) Left Joystick Y Controls Elevator (30% speed), and Driver Right Joystick X Controls Wrist (30% Speed)
            driverGamepad.getButtonBumperLeft().whenPressed(new ZeroElevator());
            driverGamepad.getButtonBumperRight().whenPressed(new ZeroWrist());
            driverGamepad.getButtonX().whenPressed(new ToggleFlower());
            driverGamepad.getButtonB().whenPressed(new ToggleExtender());
            driverGamepad.getButtonA().whenPressed(new ToggleArm());

            operatorGamepad.getButtonA().whenPressed(groundCargo);
            operatorGamepad.getButtonX().whenPressed(frontShipAndLoading);
            operatorGamepad.getButtonB().whenPressed(backShipAndLoading);
            operatorGamepad.getButtonY().whenPressed(backHatch);
        }
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