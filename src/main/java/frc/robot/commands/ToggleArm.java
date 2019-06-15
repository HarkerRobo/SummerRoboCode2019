package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.Arm;

public class ToggleArm extends InstantCommand {
   
   public ToggleArm() {
        requires(Arm.getInstance());
   }
   
    @Override
    public void initialize() {
        System.out.println("ToggleArm Initialized");
        Value value = Arm.getInstance().getSolenoid().get();

        if (value == Value.kForward)
            Arm.getInstance().getSolenoid().set(Value.kReverse);
        else
            Arm.getInstance().getSolenoid().set(Value.kForward);
    }
}