package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.RobotMap;
import frc.robot.subsystems.Climber;

public class ExtendClimbers extends TimedCommand {

    private static final double TIMEOUT = 30.0;//3.0
    public ExtendClimbers() {
        super(TIMEOUT);
        requires(Climber.getInstance());
        
    }

    @Override
    protected void initialize() {
        Climber.getInstance().getMaster().selectProfileSlot(Climber.MM_PID_SLOT, RobotMap.PRIMARY_PID_INDEX);

    }

    public void execute() {
        //Climber.getInstance().getMaster().set(ControlMode.MotionMagic, Climber.PUSHED_DOWN_POSITION);
    }
    
    protected void end() {
    
        Climber.getInstance().getMaster().set(ControlMode.Disabled, 0);
    }
}