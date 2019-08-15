package frc.robot;

import frc.robot.commands.wrist.MoveWristMotionMagic;
import frc.robot.commands.wrist.ZeroWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchExtender;
import frc.robot.subsystems.HatchFlower;
import frc.robot.subsystems.Wrist;
import frc.robot.commands.MoveElevatorAndWrist;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.arm.ToggleArm;
import frc.robot.commands.drivetrain.DriveWithLimelight;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.extender.SetExtender;
import frc.robot.commands.extender.ToggleExtender;
import frc.robot.commands.flower.SetFlower;
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
    public static final DemoMode mode = DemoMode.SAFE;
    
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
        SAFE,

        GROUPA,

        GROUPB
    }

    private OI() {
        driverGamepad = new XboxGamepad(DRIVER_PORT);
        operatorGamepad = new XboxGamepad(OPERATOR_PORT);

        initBindings();
    }

    public void initBindings() {
        MoveElevatorAndWrist groundCargo = new MoveElevatorAndWrist(100, -170);
        
        MoveElevatorAndWrist backHatch = new MoveElevatorAndWrist(7320, 2000);
        
        MoveElevatorAndWrist backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
        MoveElevatorAndWrist frontShipAndLoading = new MoveElevatorAndWrist(17600, 120);

        MoveElevatorAndWrist 
        defenseMode = new MoveElevatorAndWrist(0, Wrist.DEFENSE_POSITION);
        
        //MoveElevatorAndWrist backRocketFirstCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketSecondCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketFirstHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist backRocketSecondHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist frontRocketSecondCargo = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);
        //MoveElevatorAndWrist frontRocketSecondHatch = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);

        //MoveElevatorAndWrist defenseMode = new MoveElevatorAndWrist(elevatorSetpoint, wristSetpoint);

        if (mode == DemoMode.NORMAL) {
            driverGamepad.getButtonA().whenPressed(new ZeroElevator());
            driverGamepad.getButtonX().whenPressed(new ZeroWrist());
            driverGamepad.getButtonB().whenPressed(new ToggleArm());
            driverGamepad.getButtonY().whileHeld(new DriveWithLimelight());

            driverGamepad.getButtonBumperRight().whenPressed(defenseMode);

            driverGamepad.getButtonStart().whenPressed(new ToggleFlower());
            driverGamepad.getButtonSelect().whenPressed(new ToggleExtender());

            driverGamepad.getUpDPadButton().whenPressed(backShipAndLoading);
            driverGamepad.getDownDPadButton().whenPressed(groundCargo);
            driverGamepad.getRightDPadButton().whenPressed(frontShipAndLoading);
            driverGamepad.getLeftDPadButton().whenPressed(backHatch);

            //driverGamepad.getButtonY().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight, 10));
        }
        else if (mode == DemoMode.SAFE) {
            //Driver Controller (For guest) Left Joystick controls Drivetrain (30% speed)
            //Driver Controller (For Robotics Member) Left Joystick Y Controls Elevator (30% speed), and Driver Right Joystick X Controls Wrist (30% Speed)
            driverGamepad.getButtonBumperLeft().whenPressed(new ZeroElevator());
            driverGamepad.getButtonBumperRight().whenPressed(new ZeroWrist());
            driverGamepad.getButtonY().whileHeld(new DriveWithLimelight());

            operatorGamepad.getButtonX().whenPressed(new ToggleFlower());
            operatorGamepad.getButtonB().whenPressed(new ToggleExtender());
            operatorGamepad.getButtonA().whenPressed(new ToggleArm());
            
            operatorGamepad.getUpDPadButton().whenPressed(backShipAndLoading);
            operatorGamepad.getDownDPadButton().whenPressed(groundCargo);
            operatorGamepad.getRightDPadButton().whenPressed(frontShipAndLoading);
            operatorGamepad.getLeftDPadButton().whenPressed(backHatch);

            
        } 
        else if (mode == DemoMode.GROUPA) {

            operatorGamepad.getButtonY().whenPressed(new SetFlower(HatchFlower.OPEN));
            operatorGamepad.getButtonX().whenPressed(new SetFlower(HatchFlower.CLOSED));
            operatorGamepad.getButtonB().whenPressed(new SetExtender(HatchExtender.OUT));
            operatorGamepad.getButtonA().whenPressed(new SetExtender(HatchExtender.IN));
            operatorGamepad.getButtonBumperLeft().whenPressed(new ToggleFlower());
            operatorGamepad.getButtonBumperRight().whenPressed(new ToggleExtender());
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