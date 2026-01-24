package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.CRServo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.util.MathUtils;

import org.firstinspires.ftc.teamcode.R;

//custom cr servo for SPINDEXER. Sets minimum power to overcome wheel if error is too large
public class CRServoEx2<E extends Encoder> extends CRServo {
    //declaring varables
    private E encoder;
    private double cachingTolerance = 0.0001;
    private PIDFController pidf;

    public double error = 0;
    public double power = 0;
    public double positivePowerCount = 0;
    public double setCount = 0;
    double minPowerOvercome = 0.5;
    double minErrorThreshold = 30;//degrees
    /**
     * The mode in which the CR servo should behave.
     */
    public enum RunMode {
        /**
         * Mode in which the CR servo takes the shortest path to reach a specific angle
         */
        OptimizedPositionalControl,
        /**
         * Mode in which the CR servo is controlled with raw power
         */
        RawPower
    }
    private RunMode runmode;

    /**
     * The constructor for the CR Servo.
     *
     * @param hwMap hardwareMap
     * @param id ID of the CR servo as configured
     * @param encoder the absolute or relative encoder
     * @param runmode the runmode of the CR servo
     */
    public CRServoEx2(HardwareMap hwMap, String id, E encoder, RunMode runmode) {
        super(hwMap, id);
        this.encoder = encoder;
        this.runmode = runmode;
    }

    /**
     * A simple constructor for the CR Servo with no absolute encoder.
     * @param hwMap hardwareMap
     * @param id ID of the CR servo as configured
     */
    public CRServoEx2(HardwareMap hwMap, String id) {
        super(hwMap, id);
        this.encoder = null;
        this.runmode = RunMode.RawPower;
    }

    public void setMinPowerOvercome(double minPower){
        this.minPowerOvercome = minPower;
    }
    public double getMinPowerOvercome(){
        return minPowerOvercome;
    }

    public void setMinErrorThreshold(double minThreshold){
        this.minErrorThreshold = minThreshold;
    }
    public CRServoEx2<E> setReversed(boolean reversed) {
        if (reversed) {
            this.getServo().setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            this.getServo().setDirection(DcMotorSimple.Direction.FORWARD);
        }
        return this;
    }

    /**
     * @param runmode the new runmode to be set
     * @return this object for chaining purposes
     */
    public CRServoEx2<E> setRunMode(RunMode runmode) {
        this.runmode = runmode;
        return this;
    }

    /**
     * @return The runmode of this servo
     */
    public RunMode getRunmode() {
        return this.runmode;
    }

    /**
     * Sets the PIDF coefficients of the CR Servo for the PositionalControl runmodes
     * @param coefficients the coefficients for the PIDF controller
     * @return this object for chaining purposes
     */

    public CRServoEx2<E> setPIDFTOUse(PIDFCoefficients coefficients){
        this.pidf = new PIDFController(coefficients);
        return this;
    }


    /**
     * @param cachingTolerance the new caching tolerance between CR servo writes
     * @return this object for chaining purposes
     */
    public CRServoEx2<E> setCachingTolerance(double cachingTolerance) {
        this.cachingTolerance = cachingTolerance;
        return this;
    }

    /**
     * @return the caching tolerance of the CR servo before it writes a new power to the CR servo
     */
    public double getCachingTolerance() {
        return cachingTolerance;
    }

    /**
     * @param encoder the new encoder to be associated with the CR servo
     * @return this object for chaining purposes
     */
    public CRServoEx2<E> setEncoder(E encoder) {
        this.encoder = encoder;
        return this;
    }

    /**
     * @return the absolute encoder object associated with the CR servo
     */
    public E getEncoder() {
        return encoder;
    }

    /**
     * @param output the raw power or angle (based on the runmode of the CR servo) that the CR servo should be set to
     */
    @Override
    public void set(double output) {
        setCount++;
        if (runmode == RunMode.OptimizedPositionalControl) {
            if (encoder == null) {
                throw new IllegalStateException("Must have absolute encoder and PIDF coefficients for CR Servo to be in positional control");
            }

            double error = MathUtils.normalizeAngle(output - encoder.getAngle(), false, encoder.getAngleUnit());

            double power = pidf.calculate(0, error);
            this.error = error;
            this.power = power;

            if (power > 0) {
                positivePowerCount++;
            }
            crServo.setPower(power);

//            crServo.setPower(clamp(power, - 0.8, 0.8));//clamp to make sure not setting to too fast
        } else {
            this.power = output;
            crServo.setPower(output);
        }
    }


    public void set(double output, boolean nearWheel) {
        setCount++;
        if (runmode == RunMode.OptimizedPositionalControl) {
            if (encoder == null) {
                throw new IllegalStateException("Must have absolute encoder and PIDF coefficients for CR Servo to be in positional control");
            }

            double error = MathUtils.normalizeAngle(output - encoder.getAngle(), false, encoder.getAngleUnit());

            double sqrtError = Math.sqrt(Math.abs(error));
            double power = pidf.calculate(0, sqrtError);
            if(Math.abs(error) > 50 && Math.abs(error) < 70){
                power = Math.signum(error) * Math.max(Math.abs(power), minPowerOvercome);
            }
            this.error = error;
            this.power = power;

            if (power > 0) {
                positivePowerCount++;
            }
            crServo.setPower(power);

//            crServo.setPower(clamp(power, - 0.8, 0.8));//clamp to make sure not setting to too fast
        } else {
            this.power = output;
            crServo.setPower(output);
        }
    }
    public double clamp(double x1, double min, double max){
       return x1 < min ? min : x1 > max ? max : x1;
    }


    /**
     * @param pwmRange the PWM range the CR servo should be set to
     * @return this object for chaining purposes
     */
    public CRServoEx2<E> setPwm(PwmControl.PwmRange pwmRange) {
        getController().setServoPwmRange(crServo.getPortNumber(), pwmRange);
        return this;
    }

    /**
     * @return the extended servo controller object for the servo
     */
    public ServoControllerEx getController() {
        return (ServoControllerEx) crServo.getController();
    }

    /**
     * @return the SDK, unwrapped CR servo object
     */
    public com.qualcomm.robotcore.hardware.CRServo getServo() {
        return this.crServo;
    }


    /**
     * @param power power to be assigned to the servo if difference is greater than caching tolerance or if power is exactly 0
     */
    private void setPower(double power) {
        this.power = power;
        if ((Math.abs(power - crServo.getPower()) > cachingTolerance) || (power == 0 && crServo.getPower() != 0)) {
            crServo.setPower(power);
        }
    }

    @Override
    public String getDeviceType() {
        return "Extended " + super.getDeviceType();
    }
}
