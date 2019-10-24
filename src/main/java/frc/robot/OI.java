package frc.robot;

import frc.robot.commands.wrist.MoveWristMotionMagic;
import frc.robot.commands.wrist.ZeroWrist;
import frc.robot.commands.wristrollers.SpinWristRollersManual;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchExtender;
import frc.robot.subsystems.HatchFlower;
import frc.robot.subsystems.Wrist;
import frc.robot.util.Limelight;
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
import frc.robot.auton.test.StraightLinePath8Ft;
import frc.robot.commands.MoveElevatorAndWrist;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.arm.ToggleArm;
import frc.robot.commands.climber.ExtendClimbers;
import frc.robot.commands.drivetrain.AlignWithLimelight;
import frc.robot.commands.drivetrain.DriveToPosition;
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
    // front hatch 1 0, 0
    // front hatch 2: 19600 0
    static {
        if (RobotMap.PRACTICE_BOT) {
            groundCargo = new MoveElevatorAndWrist(0, 0);
            
            
            frontHatch = new MoveElevatorAndWrist(0, 0);
            backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
            frontShipAndLoading = new MoveElevatorAndWrist(13648, 458);//13648458
            defenseMode = new MoveElevatorAndWrist(0, Wrist.DEFENSE_POSITION);
            
            backRocketFirstCargo = new MoveElevatorAndWrist(6300, Wrist.HORIZONTAL_BACK);
            backRocketSecondCargo = new MoveElevatorAndWrist(19600, 1750);
            backHatch = new MoveElevatorAndWrist(0, Wrist.HORIZONTAL_BACK);
            backRocketSecondHatch = new MoveElevatorAndWrist(21000, 1920);

            frontRocketFirstCargo = new MoveElevatorAndWrist(0, 632);//700);
            frontRocketSecondCargo = new MoveElevatorAndWrist(15300, 679);//700);
            //frontRocketFirstHatch = groundCargo;
            frontRocketSecondHatch = new MoveElevatorAndWrist(19600, 0);
            
            //private static final MoveElevatorAndWrist frontRocketFirstHatch = new MoveElevatorAndWrist(Elevator.LOWER_SOFT_LIMIT, Wrist.HORIZONTAL_FRONT);
        } else {
            groundCargo = new MoveElevatorAndWrist(100, -70);
            backHatch = new MoveElevatorAndWrist(6860, 1989);
            frontHatch = new MoveElevatorAndWrist(0, 0);
            backShipAndLoading = new MoveElevatorAndWrist(18350, Wrist.HORIZONTAL_BACK);
            frontShipAndLoading = new MoveElevatorAndWrist(17600, 100);
            defenseMode = new MoveElevatorAndWrist(0, Wrist.DEFENSE_POSITION);
            backRocketFirstCargo = new MoveElevatorAndWrist(6000, Wrist.HORIZONTAL_BACK);
            backRocketSecondCargo = new MoveElevatorAndWrist(21009, 1723);
            backRocketSecondHatch = new MoveElevatorAndWrist(20423, 1815);
            frontRocketSecondHatch = new MoveElevatorAndWrist(14500, Wrist.HORIZONTAL_FRONT);
            frontRocketFirstCargo = new MoveElevatorAndWrist(10600, Wrist.HORIZONTAL_FRONT);
            frontRocketSecondCargo = new MoveElevatorAndWrist(10600, Wrist.HORIZONTAL_FRONT);;
        }
    }

    

    public static final double XBOX_JOYSTICK_DEADBAND = 0.1;
    public static final double XBOX_TRIGGER_DEADBAND = 0.1;

    private static final int DRIVER_PORT = 0;
    private static final int OPERATOR_PORT = 1;

    private static XboxGamepad driverGamepad;
    private static XboxGamepad operatorGamepad;

    private static OI instance;

    private boolean cargoShipMode;

    private static final MoveElevatorAndWrist groundCargo;
    // private static final MoveElevatorAndWrist frontCargo2Rocket;
    private static final MoveElevatorAndWrist backHatch;
    private static final MoveElevatorAndWrist backShipAndLoading;
    private static final MoveElevatorAndWrist frontShipAndLoading;
    private static final MoveElevatorAndWrist defenseMode;
    private static final MoveElevatorAndWrist frontHatch;
    private static final MoveElevatorAndWrist backRocketFirstCargo;
    private static final MoveElevatorAndWrist backRocketSecondCargo;
    private static final MoveElevatorAndWrist backRocketSecondHatch;
    private static final MoveElevatorAndWrist frontRocketSecondHatch;
    private static final MoveElevatorAndWrist frontRocketFirstCargo;
    private static final MoveElevatorAndWrist frontRocketSecondCargo;

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
        
        firstPath = new DriveWithMotionProfile(StraightLinePath8Ft.pathLeft, StraightLinePath8Ft.pathRight, 10);
        secondPath = new DriveWithMotionProfile(StraightLinePath8Ft.pathLeft, StraightLinePath8Ft.pathRight, 10);
        thirdPath = new DriveWithMotionProfile(StraightLinePath8Ft.pathLeft, StraightLinePath8Ft.pathRight, 10);
    }

    public static final DemoMode mode = DemoMode.NORMAL;

    public void initBindings() {

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


        //Make y front cargo
        //Make down d-pad lower rocket front
        //
        if (mode == DemoMode.NORMAL) {
            operatorGamepad.getButtonA().whenPressed(new ToggleExtender());
            operatorGamepad.getButtonX().whenPressed(new ToggleFlower());
            operatorGamepad.getButtonB().whenPressed(new ToggleArm());
            operatorGamepad.getButtonY().whenPressed(groundCargo);

            driverGamepad.getUpDPadButton().whenPressed(defenseMode);

            driverGamepad.getButtonBumperRight().whilePressed(new DriveWithLimelight());

            driverGamepad.getButtonStart().whenPressed(new ZeroElevator());
            driverGamepad.getButtonSelect().whenPressed(new ZeroWrist());  
            
            operatorGamepad.getButtonStart().whenPressed(new ZeroElevator());
            operatorGamepad.getButtonSelect().whenPressed(new ZeroWrist()); 
            
            driverGamepad.getButtonStickLeft().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));
            operatorGamepad.getButtonBumperRight().whenPressed(new CallMethodCommand(() -> cargoShipMode = !cargoShipMode));

            //back hatch 1 or front cargo 1
            operatorGamepad.getLeftDPadButton().whenPressed(
                    new ConditionalCommand(
                        backHatch, //If Has hatch
                        new ConditionalCommand(frontShipAndLoading, frontRocketSecondCargo, () -> cargoShipMode), //If has cargo
                        () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
                    )
            );

            //back hatch 2 or front cargo 2
            operatorGamepad.getRightDPadButton().whenPressed(
                    new ConditionalCommand(
                        new ConditionalCommand(backHatch, backRocketSecondHatch, () -> cargoShipMode), //If has hatch
                        new ConditionalCommand(frontShipAndLoading, frontRocketSecondCargo, () -> cargoShipMode), //If has cargo
                        () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
                )
            );
            
            //front rocket level 2
            operatorGamepad.getUpDPadButton().whenPressed(
                new ConditionalCommand(
                    new ConditionalCommand(groundCargo, frontRocketSecondHatch, ()-> cargoShipMode),
                    new ConditionalCommand(frontShipAndLoading, frontRocketSecondCargo, ()-> cargoShipMode), 
                    () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
                )
            );  

            //front rocket level 1
            operatorGamepad.getDownDPadButton().whenPressed(
                new ConditionalCommand(
                    groundCargo, 
                    new ConditionalCommand(frontShipAndLoading, frontRocketFirstCargo, ()-> cargoShipMode), 
                    () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
                )
            );  

            operatorGamepad.getButtonBumperLeft().whenPressed(new CallMethodCommand(()->{Limelight.toggleLEDs();}));

            // operatorGamepad.getUpDPadButton().whenPressed(
            //     new ConditionalCommand(
            //         new ConditionalCommand(
            //             new ConditionalCommand(frontHatch, frontRocketSecondHatch, () -> RobotMap.PRACTICE_BOT), 
            //             frontRocketSecondHatch, () -> cargoShipMode), //If has hatch
            //         new ConditionalCommand(frontShipAndLoading, frontRocketFirstCargo, () -> cargoShipMode), //If has cargo
            //         () -> HatchFlower.getInstance().getSolenoid().get() == HatchFlower.OPEN
            //     )
            // );
            
            /**driverGamepad.getButtonY().whenPressed(
                new SequentialCommandGroup(
                    new CallMethodCommand(() -> state = state+1),
                    new ConditionalCommand(
                        new ConditionalCommand(
                            firstPath,
                            secondPath,
                            () -> state == 0
                        ),  
                        new ConditionalCommand(
                            thirdPath,
                            ()-> state == 2
                        ),
                        () -> state < 2
                    )
                )
            ); 
            */
        }
        else  {
            //Driver Controller (For guest) Left Joystick controls Drivetrain (30% speed)
            //Driver Controller (For Robotics Member) Left Joystick Y Controls Elevator (30% speed), and Driver Right Joystick X Controls Wrist (30% Speed)
            driverGamepad.getButtonStart().whenPressed(new ZeroWrist());
            driverGamepad.getButtonStart().whenPressed(new ZeroElevator());
            // operatorGamepad.getButtonStart().whenPressed(new ZeroWrist());
            // operatorGamepad.getButtonStart().whenPressed(new ZeroElevator());
            //driverGamepad.getButtonY().toggleWhenPressed(new DriveWithLimelight());
            //driverGamepad.getButtonX().whenPressed(new AlignWithLimelight());
            //driverGamepad.getButtonY().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight, 10));
            
            //driverGamepad.getButtonSelect().whenPressed(new DriveWithMotionProfile(CurveRightEndStraight.pathLeft, CurveRightEndStraight.pathRight,10));
        
            operatorGamepad.getButtonX().whenPressed(new ToggleFlower());
            operatorGamepad.getButtonA().whenPressed(new ToggleExtender());
            operatorGamepad.getButtonB().whenPressed(new ToggleArm());
                
            //D-Pad bindings
            // operatorGamepad.getUpDPadButton().whenPressed(backShipAndLoading);
            // operatorGamepad.getDownDPadButton().whenPressed(groundCargo);
            // operatorGamepad.getRightDPadButton().whenPressed(frontShipAndLoading);
            // operatorGamepad.getLeftDPadButton().whenPressed(backHatch);
            driverGamepad.getButtonX().whilePressed(new SpinWristRollersManual());
            driverGamepad.getButtonA().whilePressed(new SpinWristRollersManual());

            // driverGamepad.getLeftTrigger().whenPressed(new CallMethodCommand(() -> {Drivetrain.IS_SLOW_MODE = Drivetrain.IS_SLOW_MODE == false; }));
            operatorGamepad.getButtonY().whenPressed(new MoveWristMotionMagic(Wrist.HORIZONTAL_BACK));
            operatorGamepad.getRightDPadButton().whenPressed(new MoveWristMotionMagic(Wrist.HORIZONTAL_FRONT));
            operatorGamepad.getUpDPadButton().whenPressed(new MoveWristMotionMagic(Wrist.MIDDLE_POSITION));

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