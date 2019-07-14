package frc.robot.commands.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.subsystems.Wrist;

public class MoveWristMotionMagic extends TimedCommand {

    private static final double TIMEOUT = 3.0;
    private double setpoint;
    private double prevVel;

    public MoveWristMotionMagic(double setpoint) {
        super(TIMEOUT);
        requires(Wrist.getInstance());
        if (setpoint > Wrist.BACKMOST_POSITION || setpoint < Wrist.FRONTMOST_POSITION) {
            System.out.println("Wrist Motion Magic Setpoint is unsafe!");
            cancel();
        }
        this.setpoint = setpoint;
        prevVel = 0;
    }

    @Override
    protected void initialize() {
        Wrist.getInstance().getMaster().selectProfileSlot(Wrist.MOTION_MAGIC_SLOT, RobotMap.PRIMARY_PID_INDEX);
    }

    @Override
    protected void execute() {
        double vel = Wrist.getInstance().getMaster().getActiveTrajectoryVelocity();
        double accelSign = Math.signum(vel - prevVel);
        double totalErrorSign = Math.signum(setpoint - Wrist.getInstance().getMaster().getSelectedSensorPosition());
        Wrist.getInstance().getMaster().set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, Wrist.getInstance().calculateGravFF() + Wrist.kS * totalErrorSign + Wrist.kA * accelSign * Wrist.MAX_ACCELERATION);
        SmartDashboard.putNumber("Position", Wrist.getInstance().getMaster().getSelectedSensorPosition());
        SmartDashboard.putNumber("Error", Wrist.getInstance().getMaster().getClosedLoopError());
        SmartDashboard.putNumber("Total Error", setpoint - Wrist.getInstance().getMaster().getSelectedSensorPosition());
        SmartDashboard.putNumber("Output", Wrist.getInstance().getMaster().getMotorOutputPercent());
        SmartDashboard.putNumber("accelSign", accelSign);
        prevVel = vel;
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