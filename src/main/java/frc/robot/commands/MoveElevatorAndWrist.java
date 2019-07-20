package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.MoveElevatorMotionMagic;
import frc.robot.commands.wrist.MoveWristMotionMagic;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/**
 * Moves the Wrist and the Elevator to the desired positions, passing through the elevator if needed
 * 
 * @author Jatin Kohli
 * 
 * @since 7/17/19
 */
public class MoveElevatorAndWrist extends Command {

    private static final int ELEVATOR_ALLOWABLE_ERROR = 1000;
    private static final int WRIST_ALLOWABLE_ERROR = 240;
    private int elevatorSetpoint;
    private int wristSetpoint;
    private CommandGroup group;

    /**
     * @param elevatorSetpoint The desired Elevator Postion, in Encoder Ticks
     * @param wristSetpoint The desired Wrist Postion, in Encoder Ticks
     */
    public MoveElevatorAndWrist(int elevatorSetpoint, int wristSetpoint) {
        this.elevatorSetpoint = elevatorSetpoint;
        this.wristSetpoint = wristSetpoint;
        //group.setRunWhenDisabled(false);
    }

    @Override
    protected void initialize() {
        group = new CommandGroup();
        int currentWristPos = Wrist.getInstance().getMaster().getSelectedSensorPosition();
        if (currentWristPos >= Wrist.MIDDLE_POSITION && wristSetpoint <= Wrist.MIDDLE_POSITION) { //Passthrough back to front
            group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_BACK, WRIST_ALLOWABLE_ERROR));
            group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));
        }
        else if (currentWristPos <= Wrist.MIDDLE_POSITION && wristSetpoint >= Wrist.MIDDLE_POSITION) { //Pasthrough front to back
            group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_FRONT, WRIST_ALLOWABLE_ERROR));
            group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));
        }
        group.addSequential(new MoveWristMotionMagic(wristSetpoint, WRIST_ALLOWABLE_ERROR));
        group.addSequential(new MoveElevatorMotionMagic(elevatorSetpoint, ELEVATOR_ALLOWABLE_ERROR));
        group.start();
        System.out.println("MoveElevatorAndWrist Initialized");
    }

	@Override
	protected boolean isFinished() {
		return group.isCompleted();
	}

    @Override
    protected void interrupted() {
        System.out.println("MoveElevatorAndWrist Interrupted");
        group.cancel();
    }

    @Override
    protected void end() {
        System.out.println("MoveElevatorAndWrist Ended");
        group.cancel();
    }
}