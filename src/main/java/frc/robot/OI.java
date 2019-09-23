package frc.robot;

import frc.robot.commands.wrist.MoveWristMotionMagic;
import frc.robot.commands.wrist.ZeroWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.HatchExtender;
import frc.robot.subsystems.HatchFlower;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auton.test.CurveRightEndStraight;
import frc.robot.auton.test.CurveRightTest;
import frc.robot.auton.comp.LeftCargoBayToLeftLoadingDock;
import frc.robot.auton.comp.LeftLoadingDockToLeftRocket;
import frc.robot.auton.comp.LevelOneLeftHabToLeftCargoBay;
import frc.robot.auton.comp.LevelOneMiddleHabToRightCargoBay;
import frc.robot.auton.test.SmoothCurveRightEndStraight;
import frc.robot.auton.test.StraightLinePath5Ft;
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
import harkerrobolib.auto.ParallelCommandGroup;
import harkerrobolib.auto.SequentialCommandGroup;
import harkerrobolib.commands.CallMethodCommand;
import harkerrobolib.commands.ConditionalCommand;
import harkerrobolib.commands.PrintCommand;
import harkerrobolib.wrappers.XboxGamepad;

/**
 * @author Jatin Kohli
 * @author Chirag Kaushik
 * 
 * @since 6/14/19
 */
public class OI {

    static {
        if (RobotMap.PRACTICE_BOT) {
            groundCargo = new MoveElevatorAndWrist(100, -170);
            backHatch = new MoveElevatorAndWrist(6400, 1989);
            backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
            frontShipAndLoading = new MoveElevatorAndWrist(17600, 100);
            defenseMode = new MoveElevatorAndWrist(0, Wrist.DEFENSE_POSITION);
            backRocketFirstCargo = new MoveElevatorAndWrist(6300, Wrist.HORIZONTAL_BACK);
            backRocketSecondCargo = new MoveElevatorAndWrist(19600, 1750);
            backRocketSecondHatch = new MoveElevatorAndWrist(19600, 1750);
            frontRocketSecondHatch = new MoveElevatorAndWrist(14500, Wrist.HORIZONTAL_FRONT);
            frontRocketFirstCargo = new MoveElevatorAndWrist(10600, Wrist.HORIZONTAL_FRONT);
        } else {
            groundCargo = new MoveElevatorAndWrist(100, -70);
            backHatch = new MoveElevatorAndWrist(6860, 1989);
            backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
            frontShipAndLoading = new MoveElevatorAndWrist(17600, 100);
            defenseMode = new MoveElevatorAndWrist(0, Wrist.DEFENSE_POSITION);
            backRocketFirstCargo = new MoveElevatorAndWrist(6000, Wrist.HORIZONTAL_BACK);
            backRocketSecondCargo = new MoveElevatorAndWrist(19600, 1750);
            backRocketSecondHatch = new MoveElevatorAndWrist(19600, 1750);
            frontRocketSecondHatch = new MoveElevatorAndWrist(14500, Wrist.HORIZONTAL_FRONT);
            frontRocketFirstCargo = new MoveElevatorAndWrist(10600, Wrist.HORIZONTAL_FRONT);
        }
    }

    public static final DemoMode mode = DemoMode.NORMAL;

    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    private static XboxGamepad driverGamepad;
    private static XboxGamepad operatorGamepad;

    private static OI instance;

    private boolean cargoShipMode;

    private static final MoveElevatorAndWrist groundCargo;
    private static final MoveElevatorAndWrist backHatch;
    private static final MoveElevatorAndWrist backShipAndLoading;
    private static final MoveElevatorAndWrist frontShipAndLoading;
    private static final MoveElevatorAndWrist defenseMode;
    private static final MoveElevatorAndWrist backRocketFirstCargo;
    private static final MoveElevatorAndWrist backRocketSecondCargo;
    private static final MoveElevatorAndWrist backRocketSecondHatch;
    private static final MoveElevatorAndWrist frontRocketSecondHatch;
    private static final MoveElevatorAndWrist frontRocketFirstCargo;

    public static int state;
    private static DriveWithMotionProfile firstPath; // Path from Left Hab to Left Cargo Ship
    private static DriveWithMotionProfile secondPath; // Path from the Left Front cargo bay to the Left loading dock
    private static DriveWithMotionProfile thirdPath; // Path from Left loading dock to Left rocket
 
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

        state = -1;

        chooseAutonPaths();
        initBindings();
        cargoShipMode = true;
    }

    public boolean getCargoShipMode() {
        return cargoShipMode;
    }

    public void chooseAutonPaths() {
        //Starting on the Left of the Hab:
        DriveWithMotionProfile leftHabToLeftFrontCargoBay = new DriveWithMotionProfile(
                LevelOneLeftHabToLeftCargoBay.pathLeft, LevelOneLeftHabToLeftCargoBay.pathRight, 10
        );
        DriveWithMotionProfile frontLeftCargoBayToLeftLoadingDock = new DriveWithMotionProfile(
                LeftCargoBayToLeftLoadingDock.pathLeft, LeftCargoBayToLeftLoadingDock.pathRight, 10
        );
        DriveWithMotionProfile leftLoadingDockToLeftRocket = new DriveWithMotionProfile(
                LeftLoadingDockToLeftRocket.pathLeft, LeftLoadingDockToLeftRocket.pathRight, 10
        );
        
        //Starting on the Middle of the Hab(can go either way to loading station):
        DriveWithMotionProfile middleHabToLeftFrontCargoBay = new DriveWithMotionProfile(
                LevelOneMiddleHabToRightCargoBay.pathRight, LevelOneMiddleHabToRightCargoBay.pathLeft, 10
        );
        DriveWithMotionProfile middleHabToRightFrontCargoBay = new DriveWithMotionProfile(
                LevelOneMiddleHabToRightCargoBay.pathLeft, LevelOneMiddleHabToRightCargoBay.pathRight, 10
        );

        //Starting on the Right of the Hab:
        DriveWithMotionProfile rightHabToRightFrontCargoBay = new DriveWithMotionProfile(
            LevelOneLeftHabToLeftCargoBay.pathRight, LevelOneLeftHabToLeftCargoBay.pathLeft, 10
        );
        DriveWithMotionProfile frontRightCargoBayToRightLoadingDock = new DriveWithMotionProfile(
                LeftCargoBayToLeftLoadingDock.pathRight, LeftCargoBayToLeftLoadingDock.pathLeft, 10
        );
        DriveWithMotionProfile rightLoadingDockToRightRocket = new DriveWithMotionProfile(
                LeftLoadingDockToLeftRocket.pathRight, LeftLoadingDockToLeftRocket.pathLeft, 10
        );
        
        firstPath = leftHabToLeftFrontCargoBay;
        secondPath = frontLeftCargoBayToLeftLoadingDock;
        thirdPath = leftLoadingDockToLeftRocket;

        /*
        firstPathM = 
        secondPathM = 
        thirdPathM = 

        firstPathR = rightHabToRightFrontCargoBay;
        secondPathR = frontRightCargoBayToRightLoadingDock;
        thirdPathR = rightLoadingDockToRightRocket;
        */
    }

    public void initBindings() {
        // MoveElevatorAndWrist groundCargo = new MoveElevatorAndWrist(100, -170);

        // Go from Hab to cargo ship/rocket ship
        // Have driver align manually
        // Go to loading station
        // Go to rocket/cargo ship

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

            driverGamepad.getButtonBumperRight().whenPressed(new AlignWithLimelight());

            driverGamepad.getButtonStart().whenPressed(new ZeroElevator());
            driverGamepad.getButtonSelect().whenPressed(new ZeroWrist());  
            
            operatorGamepad.getButtonStart().whenPressed(new ZeroElevator());
            operatorGamepad.getButtonSelect().whenPressed(new ZeroWrist()); 
            
            driverGamepad.getButtonStickLeft().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));
            operatorGamepad.getButtonBumperRight().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));

            operatorGamepad.getLeftDPadButton().whenPressed(
                    new ConditionalCommand(backHatch, //If Has hatch
                        new ConditionalCommand(backShipAndLoading, backRocketFirstCargo, () -> cargoShipMode), //If has cargo
                        () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
                    )
            );
            operatorGamepad.getRightDPadButton().whenPressed(
                    new ConditionalCommand(
                        new ConditionalCommand(backHatch, backRocketSecondHatch, () -> cargoShipMode), //If has hatch
                        new ConditionalCommand(backShipAndLoading, backRocketSecondCargo, () -> cargoShipMode), //If has cargo
                        () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
                )
            );
            
            operatorGamepad.getUpDPadButton().whenPressed(
                new ConditionalCommand(
                    frontRocketSecondHatch, //If has hatch
                    new ConditionalCommand(frontShipAndLoading, frontRocketFirstCargo, () -> cargoShipMode), //If has cargo
                    () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
                )
            );
            operatorGamepad.getDownDPadButton().whenPressed(groundCargo);  
            
            driverGamepad.getButtonY().whenPressed(
                new SequentialCommandGroup(
                    new CallMethodCommand(() -> state = state+1),
                    new ConditionalCommand( 
                        new ConditionalCommand(
                            new CallMethodCommand(() -> SmartDashboard.putString("Path", "firstPath")), 
                            new CallMethodCommand(() -> SmartDashboard.putString("Path", "secondPath")), 
                            () -> state == 0
                        ),  
                        new CallMethodCommand(() -> SmartDashboard.putString("Path", "thirdPath")),
                        () -> state == 2
                    )
                )
            );
        }
        else  {
            //Driver Controller (For guest) Left Joystick controls Drivetrain (30% speed)
            //Driver Controller (For Robotics Member) Left Joystick Y Controls Elevator (30% speed), and Driver Right Joystick X Controls Wrist (30% Speed)
            driverGamepad.getButtonBumperLeft().whenPressed(new ZeroElevator());
            driverGamepad.getButtonBumperRight().whenPressed(new ZeroWrist());
            //driverGamepad.getButtonY().toggleWhenPressed(new DriveWithLimelight());
            //driverGamepad.getButtonX().whenPressed(new AlignWithLimelight());
            //driverGamepad.getButtonY().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight, 10));
            
            driverGamepad.getButtonSelect().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight,10));

            operatorGamepad.getButtonX().whenPressed(new ToggleFlower());
            operatorGamepad.getButtonB().whenPressed(new ToggleExtender());
            operatorGamepad.getButtonA().whenPressed(new ToggleArm());

            //D-Pad bindings
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