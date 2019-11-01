package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.elevator.MoveElevatorMotionMagic;
import frc.robot.commands.extender.SetExtender;
import frc.robot.commands.wrist.MoveWristMotionMagic;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.HatchExtender;
import frc.robot.subsystems.HatchFlower;
import frc.robot.subsystems.Wrist;

/**
 * Moves the Wrist and the Elevator to the desired positions, passing through
 * the elevator if needed
 * 
 * @author Jatin Kohli
 * 
 * @since 7/17/19
 */
public class MoveElevatorAndWrist extends Command {

    private static final int ELEVATOR_ALLOWABLE_ERROR = Elevator.ALLOWABLE_ERROR;
    private static final int WRIST_ALLOWABLE_ERROR = Wrist.ALLOWABLE_ERROR;
    
    private int elevatorSetpoint;
    private int wristSetpoint;
    private CommandGroup group;

    // private static final double INVALID_TIME = 0.06;
    // private double startTime;

    /**
     * @param elevatorSetpoint The desired Elevator Postion, in Encoder Ticks
     * @param wristSetpoint    The desired Wrist Postion, in Encoder Ticks
     */
    public MoveElevatorAndWrist(int elevatorSetpoint, int wristSetpoint) {
        this.elevatorSetpoint = elevatorSetpoint;
        this.wristSetpoint = wristSetpoint;
    }

    @Override
    protected void initialize() {
        group = new CommandGroup();
        
        group.addSequential(new SetExtender(HatchExtender.IN));

        int currentWristPos = Wrist.getInstance().getMaster().getSelectedSensorPosition();
        boolean isWristInDefense = Math.abs(currentWristPos - Wrist.DEFENSE_POSITION) < Wrist.MIDDLE_VARIANCE;
        // SmartDashboard.putBoolean("isWristInMiddle", isWristInDefense);

        // If we need to do passthrough checks, differs for comp and practice bots
        // because of arm
        boolean checkPassthrough = !(currentWristPos > Wrist.MIDDLE_POSITION && wristSetpoint > Wrist.MIDDLE_POSITION);

        if (checkPassthrough) {
            group.addSequential(new SetArm(Arm.OUT));
            group.addSequential(new WaitCommand(0.3));

            if (!isWristInDefense) 
            {
                if (currentWristPos >= Wrist.MIDDLE_POSITION && wristSetpoint <= Wrist.MIDDLE_POSITION) // Passthrough back to front
                { 
                    group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_BACK, WRIST_ALLOWABLE_ERROR));
                    group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));

                } 
                else if ((currentWristPos <= Wrist.MIDDLE_POSITION && wristSetpoint >= Wrist.MIDDLE_POSITION)
                        || wristSetpoint == Wrist.DEFENSE_POSITION) // Pasthrough front to back
                { 
                        group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_FRONT, WRIST_ALLOWABLE_ERROR));
                        group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));
                }
            }
            else 
            {
                group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_BACK));
            }
        }

        //Old passthrough logic
        // if (checkPassthrough && !isWristInDefense) { // If not in defense mode, check for
        //     if (currentWristPos >= Wrist.MIDDLE_POSITION && wristSetpoint <= Wrist.MIDDLE_POSITION) { // Passthrough back to front
                
        //         group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_BACK, WRIST_ALLOWABLE_ERROR));
        //         group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));

        //     } else if ((currentWristPos <= Wrist.MIDDLE_POSITION && wristSetpoint >= Wrist.MIDDLE_POSITION)
        //             || wristSetpoint == Wrist.DEFENSE_POSITION) { // Pasthrough front to back

        //         group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_FRONT, WRIST_ALLOWABLE_ERROR));
        //         group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));
        //     }
        // } else if (checkPassthrough) {
        //     group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_BACK));
        // }

        group.addSequential(new MoveWristMotionMagic(wristSetpoint, WRIST_ALLOWABLE_ERROR));
        group.addSequential(new MoveElevatorMotionMagic(elevatorSetpoint, ELEVATOR_ALLOWABLE_ERROR));
        
        if (HatchFlower.getInstance().getSolenoid().get() == HatchFlower.CLOSED) 
            group.addSequential(new SetArm(Arm.OUT));
        
        group.start();
        // startTime = Timer.getFPGATimestamp();
    }

    @Override
    protected boolean isFinished() {
        return group.isCompleted();
        // (Timer.getFPGATimestamp() - startTime) > INVALID_TIME
    }

    @Override
    protected void interrupted() {
        end();
    }

    @Override
    protected void end() {
        group.cancel();
    }
}