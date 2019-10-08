package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Climber;

public class ExtendClimbers extends InstantCommand {
    
    public ExtendClimbers(){
        requires(Climber.getInstance());
    }
    @Override
    protected void initialize() {

    }
    protected void end() {
        
    }
}