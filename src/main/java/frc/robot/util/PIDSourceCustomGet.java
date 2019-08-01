package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Creates a PIDSource which retrieves a sensor's value depending on the PIDSourceType whenever prompted by the PID Controller.
 * PIDSource.kDisplacement takes one reading of the sensor value, whereas PIDSource.kRate takes multiple readings
 * 
 * @author Angela Jia
 * @author Jatin Kohli
 * 
 * @since 7/31/19
 */
public class PIDSourceCustomGet implements PIDSource {

    private Supplier<Double> sensorValue;
    private PIDSourceType pidSource;

    public PIDSourceCustomGet(Supplier<Double> sensorValue, PIDSourceType pidSource) {
        this.sensorValue = sensorValue;
        this.pidSource = pidSource;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        this.pidSource = pidSource;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSource;
    }

    @Override
    public double pidGet() {
        return sensorValue.get();
    }

}