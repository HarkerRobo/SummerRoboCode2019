package frc.robot.util;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Creates a PIDOutput to retrieve the necessary output from the PIDController
 * 
 * @author Angela Jia
 * 
 * @since 7/31/19
 */
public class PIDOutputGetter implements PIDOutput {

    private double output;

    public PIDOutputGetter(double output) {
        this.output = output;
    }

    @Override
    public void pidWrite(double output) {
        this.output = output;
    }

    public double getOutput() {
        return output;
    }

}