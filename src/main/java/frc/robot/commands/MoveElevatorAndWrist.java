package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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

    private static final int ELEVATOR_ALLOWABLE_ERROR = 1000;
    private static final int WRIST_ALLOWABLE_ERROR = 240;
    private int elevatorSetpoint;
    private int wristSetpoint;
    private CommandGroup group;

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
        if (!(currentWristPos >= Wrist.MIDDLE_POSITION && wristSetpoint >= Wrist.MIDDLE_POSITION)) {
            group.addSequential(new SetArm(Arm.OUT));
            group.addSequential(new WaitCommand(0.3));
            if (currentWristPos >= Wrist.MIDDLE_POSITION && wristSetpoint <= Wrist.MIDDLE_POSITION) { // Passthrough back to front
                group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_BACK, WRIST_ALLOWABLE_ERROR));
                group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));
            } else if (currentWristPos <= Wrist.MIDDLE_POSITION && wristSetpoint >= Wrist.MIDDLE_POSITION) { // Pasthrough front to back
                group.addSequential(new MoveWristMotionMagic(Wrist.HORIZONTAL_FRONT, WRIST_ALLOWABLE_ERROR));
                group.addSequential(new MoveElevatorMotionMagic(Elevator.PASSTHROUGH_HEIGHT, ELEVATOR_ALLOWABLE_ERROR));
            }
        }
        group.addSequential(new MoveWristMotionMagic(wristSetpoint, WRIST_ALLOWABLE_ERROR));
        group.addSequential(new MoveElevatorMotionMagic(elevatorSetpoint, ELEVATOR_ALLOWABLE_ERROR));
        group.start();
    }

    @Override
    protected boolean isFinished() {
        return group.isCompleted();
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