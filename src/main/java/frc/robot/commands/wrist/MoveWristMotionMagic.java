package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Wrist;

public class MoveWristMotionMagic extends Command {

    private double pos;

    public MoveWristMotionMagic(double pos) {
        requires(Wrist.getInstance());
        if (pos > Wrist.BACKMOST_POSITION || pos < Wrist.FRONTMOST_POSITION) {
            System.out.println("Wrist Motion Magic Setpoint is unsafe!");
            cancel();
        }
        this.pos = pos;
    }

    @Override
    protected void initialize() {
        Wrist.getInstance().getMaster().selectProfileSlot(Wrist.MOTION_MAGIC_SLOT, RobotMap.PRIMARY_PID_INDEX);
        Wrist.getInstance().getMaster().set(ControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF());
    }

    @Override
    protected void execute() {
        SmartDashboard.putNumber("Position", Wrist.getInstance().getMaster().getSelectedSensorPosition());
        SmartDashboard.putNumber("Error", Wrist.getInstance().getMaster().getClosedLoopError());
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Wrist.getInstance().getMaster().set(ControlMode.Disabled, 0, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF());
    }

    @Override
    protected void interrupted() {
        end();
    }
}