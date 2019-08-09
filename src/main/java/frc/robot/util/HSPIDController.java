package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Incorporates PIDController, PIDOutput, and PIDSource all into one class <p>
 * Default Period is 10ms
 * 
 * @author Jatin Kohli
 * 
 * @since 8/3/19
 */
public class HSPIDController {
    public static final double PID_CONTROLLER_PERIOD = 0.01;
    private PIDController controller;
    private PIDOutputGetter outputGetter;

    public HSPIDController(double kP, double kI, double kD, Supplier<Double> sensorValue, PIDSourceType type) {
        this(kP, kI, kD, 0, sensorValue, type, PID_CONTROLLER_PERIOD);
    }

    public HSPIDController(double kP, double kI, double kD, double kF, Supplier<Double> sensorValue, PIDSourceType type) {
        this(kP, kI, kD, kF, sensorValue, type, PID_CONTROLLER_PERIOD);
    }

    public HSPIDController(double kP, double kI, double kD, Supplier<Double> sensorValue, PIDSourceType type, double period) {
        this(kP, kI, kD, 0, sensorValue, type, period);
    }

    public HSPIDController(double kP, double kI, double kD, double kF, Supplier<Double> sensorValue, PIDSourceType type, double period) {
        outputGetter = new PIDOutputGetter();
        controller = new PIDController(kP, kI, kD, kF,
                new PIDSourceCustomGet(sensorValue, type), 
                outputGetter, period);
    }

    public void enable() {
        controller.enable();
    }

    public void reset() {
        controller.reset();
    }

    public double getOutput() {
        return outputGetter.getOutput();
    }

    public double getError() {
        return controller.getError();
    }

    public PIDController getPIDController() {
        return controller;
    }

    /**
     * Creates a PIDOutput to retrieve the necessary output from the PIDController
     * 
     * @author Angela Jia
     * 
     * @since 7/31/19
     */
    private class PIDOutputGetter implements PIDOutput {

        private double output;

        @Override
        public void pidWrite(double output) {
            this.output = output;
        }

        public double getOutput() {
            return output;
        }

    }

    /**
     * Creates a PIDSource which retrieves a sensor's value depending on the PIDSourceType whenever prompted by the PID Controller.
     * PIDSource.kDisplacement takes one reading of the sensor value, whereas PIDSource.kRate takes multiple readings
     * 
     * @author Angela Jia
     * @author Jatin Kohli
     * 
     * @since 7/31/19
     */
    private class PIDSourceCustomGet implements PIDSource {

        private Supplier<Double> sensorValue;
        private PIDSourceType pidSource;

        public PIDSourceCustomGet(Supplier<Double> sensorValue, PIDSourceType sourceType) {
            this.sensorValue = sensorValue;
            this.pidSource = sourceType;
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
}