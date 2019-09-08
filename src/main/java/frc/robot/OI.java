package frc.robot;

import frc.robot.commands.wrist.MoveWristMotionMagic;
import frc.robot.commands.wrist.ZeroWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchExtender;
import frc.robot.subsystems.HatchFlower;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.auton.CurveRightEndStraight;
import frc.robot.auton.CurveRightTest;
import frc.robot.auton.SmoothCurveRightEndStraight;
import frc.robot.auton.StraightLinePath5Ft;
import frc.robot.commands.MoveElevatorAndWrist;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.arm.ToggleArm;
import frc.robot.commands.drivetrain.AlignWithLimelight;
import frc.robot.commands.drivetrain.DriveWithLimelight;
import frc.robot.commands.drivetrain.DriveWithMotionProfile;
import frc.robot.commands.elevator.ZeroElevator;
import frc.robot.commands.extender.SetExtender;
import frc.robot.commands.extender.ToggleExtender;
import frc.robot.commands.flower.SetFlower;
import frc.robot.commands.flower.ToggleFlower;
import harkerrobolib.auto.SequentialCommandGroup;
import harkerrobolib.commands.CallMethodCommand;
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

    private boolean cargoShipMode;

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
        cargoShipMode = true;
    }

    public boolean getCargoShipMode() {
        return cargoShipMode;
    }

    public void initBindings() {
        MoveElevatorAndWrist groundCargo = new MoveElevatorAndWrist(100, -170);
        
        MoveElevatorAndWrist backHatch = new MoveElevatorAndWrist(7320, 2000);
        
        MoveElevatorAndWrist backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
        MoveElevatorAndWrist frontShipAndLoading = new MoveElevatorAndWrist(17600, 100);

        MoveElevatorAndWrist defenseMode = new MoveElevatorAndWrist(0, Wrist.DEFENSE_POSITION);
        
        MoveElevatorAndWrist backRocketFirstCargo = new MoveElevatorAndWrist(6000, Wrist.HORIZONTAL_BACK);
        MoveElevatorAndWrist backRocketSecondCargo = new MoveElevatorAndWrist(19600, 1750);
        MoveElevatorAndWrist backRocketSecondHatch = new MoveElevatorAndWrist(19600, 1750);
        // MoveElevatorAndWrist frontRocketSecondHatch = new MoveElevatorAndWrist(14500, Wrist.HORIZONTAL_FRONT);

        // SequentialCommandGroup testAuton = new SequentialCommandGroup(
        //         new SetFlower(HatchFlower.OPEN),
        //         /*new DriveWithMotionProfile(StraightLinePath5Ft.pathLeft, StraightLinePath5Ft.pathRight, 10),*/
        //         new AlignWithLimelight(), 
        //         backHatch,
        //         new SetFlower(HatchFlower.CLOSED)
        // );

        if (mode == DemoMode.NORMAL) {
            operatorGamepad.getButtonA().whenPressed(new ToggleExtender());
            operatorGamepad.getButtonX().whenPressed(new ToggleFlower());
            operatorGamepad.getButtonB().whenPressed(new ToggleArm());

            driverGamepad.getUpDPadButton().whenPressed(defenseMode);

            driverGamepad.getButtonStart().whenPressed(new ZeroElevator());
            driverGamepad.getButtonSelect().whenPressed(new ZeroWrist());  
            operatorGamepad.getButtonStart().whenPressed(new ZeroElevator());
            operatorGamepad.getButtonSelect().whenPressed(new ZeroWrist()); 
            
            driverGamepad.getButtonStickLeft().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));
            operatorGamepad.getButtonBumperRight().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));

            // operatorGamepad.getLeftDPadButton().whenPressed(
            //         new ConditionalCommand(backHatch, //If Has hatch
            //             new ConditionalCommand(backShipAndLoading, backRocketFirstCargo, () -> cargoShipMode), //If has cargo
            //             () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
            //         )
            // );
            // operatorGamepad.getRightDPadButton().whenPressed(
            //         new ConditionalCommand(
            //             new ConditionalCommand(backHatch, backRocketSecondHatch, () -> cargoShipMode), //If has hatch
            //             new ConditionalCommand(backShipAndLoading, backRocketSecondCargo, () -> cargoShipMode), //If has cargo
            //             () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
            //     )
            // );
            // operatorGamepad.getUpDPadButton().whenPressed(
            //     new ConditionalCommand(
            //         frontRocketSecondHatch, //If has hatch
            //         new ConditionalCommand(frontShipAndLoading, frontRocketSecondCargo, () -> cargoShipMode), //If has cargo
            //         () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
            //     )
            // );
            operatorGamepad.getDownDPadButton().whenPressed(groundCargo);

            //driverGamepad.getButtonY().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight, 10));
        }
        else  {
            //Driver Controller (For guest) Left Joystick controls Drivetrain (30% speed)
            //Driver Controller (For Robotics Member) Left Joystick Y Controls Elevator (30% speed), and Driver Right Joystick X Controls Wrist (30% Speed)
            driverGamepad.getButtonBumperLeft().whenPressed(new ZeroElevator());
            driverGamepad.getButtonBumperRight().whenPressed(new ZeroWrist());
            //driverGamepad.getButtonY().toggleWhenPressed(new DriveWithLimelight());
            //driverGamepad.getButtonX().whenPressed(new AlignWithLimelight());

            operatorGamepad.getButtonX().whenPressed(new ToggleFlower());
            operatorGamepad.getButtonB().whenPressed(new ToggleExtender());
            operatorGamepad.getButtonA().whenPressed(new ToggleArm());
            
            operatorGamepad.getUpDPadButton().whenPressed(backShipAndLoading);
            operatorGamepad.getDownDPadButton().whenPressed(groundCargo);
            operatorGamepad.getRightDPadButton().whenPressed(frontShipAndLoading);
            operatorGamepad.getLeftDPadButton().whenPressed(backHatch);

            driverGamepad.getButtonStickLeft().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));
            operatorGamepad.getButtonBumperRight().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));

            //operatorGamepad.getButtonBumperLeft();
            //driverGamepad.getLeftDPadButton().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight, 10));
            //driverGamepad.getUpDPadButton().whenPressed(new DriveWithMotionProfile(StraightLinePath5Ft.pathLeft, StraightLinePath5Ft.pathRight, 10));
            //driverGamepad.getButtonStart().whenPressed(new DriveWithMotionProfile(SmoothCurveRightEndStraight.SmoothCurveRightEndStraightLeft, SmoothCurveRightEndStraight.SmoothCurveRightEndStraightRight, 10));
            //driverGamepad.getButtonSelect().whenPressed(new DriveWithMotionProfile(SmoothCurveRightEndStraight.SmoothCurveRightEndStraightLeftReversed, SmoothCurveRightEndStraight.SmoothCurveRightEndStraightRightReversed, 10));
            //driverGamepad.getDownDPadButton().whenPressed(new DriveWithMotionProfile(CurveRightTest.pathLeft, CurveRightTest.pathRight,10));
            //driverGamepad.getLeftDPadButton().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight,10));
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