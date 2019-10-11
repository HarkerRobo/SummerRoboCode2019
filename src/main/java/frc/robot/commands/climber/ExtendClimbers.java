package frc.robot.commands.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class ExtendClimbers extends InstantCommand {

    
    public ExtendClimbers() {
        requires(Climber.getInstance());
    }

    @Override
    protected void initialize() {
        
    }

    public void execute() {
        Climber.getInstance().getMaster().set(ControlMode.MotionMagic, Climber.PUSHED_DOWN_POSITION);
    }
    
    protected void end() {
        
    }
}