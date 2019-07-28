package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class SetArm extends InstantCommand {

    private DoubleSolenoid.Value value;

    public SetArm(DoubleSolenoid.Value value) {
        requires(Arm.getInstance());
        this.value = value;
    }

    @Override
    protected void initialize() {
        Arm.getInstance().getSolenoid().set(value);    
    }
}