package frc.robot.classes.spikemotor;

import java.util.ArrayList;
import java.util.Optional;

/**
 * An abstracted motor interface designed for easy replacement.
 */
public abstract class SpikeMotor {
    private boolean isInitialized = false;
    private ArrayList<SpikeMotor> followers;
    private SpikeMotor follower = null;

    /**
     * Initializes this motor with the given device number.
     *
     * @param deviceNumber an int giving the device number
     * @return nothing
     * @throws IllegalStateException if already initialized
     * @see #initImpl(int deviceNumber)
     */
    public final void init(int deviceNumber) {
        if (isInitialized) {
            throw new IllegalStateException(
                    "Motor cannot be initialized twice");
        }
        followers = new ArrayList<SpikeMotor>();
        initImpl(deviceNumber);
        isInitialized = true;
    }

    /**
     * Follows another motor.
     *
     * @param motor the SpikeMotor to follow
     * @return nothing
     * @throws IllegalStateException if either motor is not initialized
     */
    public final void follow(SpikeMotor otherMotor) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        if (!otherMotor.isInitialized) {
            throw new IllegalStateException(
                    "Cannot follow a motor that has not been initialized");
        }
        if (otherMotor.follower != null) {
            otherMotor = otherMotor.follower;
        }
        otherMotor.addFollower(this);
        follower = otherMotor;
    }

    /**
     * Adds another motor as a follower of this one.
     *
     * @param follower the SpikeMotor follower
     * @return nothing
     */
    protected void addFollower(SpikeMotor follower) {
        followers.add(follower);
    }

    /**
     * Sets the speed of the motor to the given value in ft/s.
     *
     * @param speed a double indicating the speed
     * @return nothing
     * @throws IllegalStateException if not initialized
     * @see #setSpeedImpl(double speed)
     */
    public final void setSpeed(double speed) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        for (SpikeMotor follower : followers) {
            follower.setSpeed(speed);
        }
        setSpeedImpl(speed);
    }

    /**
     * Gets the speed of the motor in ft/s.
     *
     * @return a double indicating the speed
     * @throws IllegalStateException if not initialized
     * @see #getSpeedImpl()
     */
    public final double getSpeed() {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        return getSpeedImpl();
    }

    /**
     * Sets the position of the motor in feet. Note that this does not move the
     * motor,
     * but instead sets the internal encoder such that a following getPosition()
     * call
     * will return the set value.
     *
     * @param position a double indicating the position to set
     * @return nothing
     * @throws IllegalStateException if not initialized
     * @see #setPositionImpl()
     */
    public final void setPosition(double position) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        for (SpikeMotor follower : followers) {
            follower.setPosition(position);
        }
        setPositionImpl(position);
    }

    /**
     * Gets the position of the motor in feet.
     *
     * @return a double indicating the position
     * @throws IllegalStateException if not initialized
     * @see #getPositionImpl()
     */
    public final double getPosition() {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        return getPositionImpl();
    }

    /**
     * Moves the motor to the given encoder position such that
     * a following getPosition() will return the set value.
     *
     * @param position a double indicating the position to set
     * @return nothing
     * @throws IllegalStateException if not initialized
     * @see #moveToImpl()
     */
    public final void moveTo(double position) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        for (SpikeMotor follower : followers) {
            follower.moveTo(position);
        }
        moveToImpl(position);
    }

    /**
     * Sets the PID values of the motor.
     * 
     * @param slotId the slot ID to set the PID of
     * @param kP     the P value
     * @param kI     the I value
     * @param kD     the D value
     * @param kF     the F value
     * @param izone  the integral zone
     * @return nothing
     * @throws IllegalStateException if not initialized
     * @see #setPidImpl()
     */
    public final void setPid(int slotId, double kP, double kI, double kD, double kF, Optional<Double> izone) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        setPidImpl(slotId, kP, kI, kD, kF, izone);
    }

    /**
     * Selects a PID slot.
     * 
     * @param slotId the slot ID to select
     * @return nothing
     * @throws IllegalStateException if not initialized
     * @see #selectPidImpl()
     */
    public final void selectPid(int slotId) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        selectPidImpl(slotId);
    }

    /**
     * Sets the closed loop ramp.
     * 
     * @param ramp the ramp value
     * @return nothing
     * @throws IllegalStateException if not initialized
     * @see #setClosedLoopRampImpl()
     */
    public final void setClosedLoopRamp(double ramp) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        setClosedLoopRampImpl(ramp);
    }

    /**
     * Sets the open loop ramp.
     * 
     * @param ramp the ramp value
     * @return nothing
     * @throws IllegalStateException if not initialized
     * @see #setOpenLoopRampImpl()
     */
    public final void setOpenLoopRamp(double ramp) {
        if (!isInitialized) {
            throw new IllegalStateException(
                    "Motor must be initialized first");
        }
        setOpenLoopRampImpl(ramp);
    }

    /**
     * The implementation of init.
     *
     * @param deviceNumber an int giving the device number
     * @return nothing
     * @see #init(int deviceNumber)
     */
    abstract protected void initImpl(int deviceNumber);

    /**
     * The implementation of setSpeed.
     * 
     * @param speed a double indicating the speed
     * @return nothing
     * @see #setSpeed(double speed)
     */
    abstract protected void setSpeedImpl(double speed);

    /**
     * The implementation of getSpeed.
     *
     * @return a double indicating the speed
     * @see #getSpeed()
     */
    abstract protected double getSpeedImpl();

    /**
     * The implementation of setPosition.
     *
     * @param position a double indicating the position to set
     * @return nothing
     * @see #setPosition()
     */
    abstract protected void setPositionImpl(double position);

    /**
     * The implementation of getPosition.
     *
     * @return a double indicating the position
     * @see #getPosition()
     */
    abstract protected double getPositionImpl();

    /**
     * The implementation of moveTo.
     *
     * @param position a double indicating the position to move to
     * @return nothing
     * @see #moveTo()
     */
    abstract protected void moveToImpl(double position);

    /**
     * Gets the conversion factor.
     *
     * @return the conversion factor
     */
    abstract protected double getConversionFactor();

    /**
     * The implementation of setPid.
     * 
     * @param slotId the slot ID to set the PID of
     * @param kP     the P value
     * @param kI     the I value
     * @param kD     the D value
     * @param kF     the F value
     * @param izone  the integral zone
     * @return nothing
     * @see #setPid()
     */
    abstract protected void setPidImpl(int slotId, double kP, double kI, double kD, double kF, Optional<Double> izone);

    /**
     * The implementation of selectPid.
     * 
     * @param slotId the slot ID to select
     * @return nothing
     * @see #selectPid()
     */
    abstract protected void selectPidImpl(int slotId);

    /**
     * The implementation of setClosedLoopRamp.
     * 
     * @param ramp the ramp value
     * @return nothing
     * @see #setClosedLoopRamp()
     */
    abstract protected void setClosedLoopRampImpl(double ramp);

    /**
     * The implementation of setOpenLoopRamp.
     * 
     * @param ramp the ramp value
     * @return nothing
     * @see #setOpenLoopRamp()
     */
    abstract protected void setOpenLoopRampImpl(double ramp);

    /**
     * Accelerates the mover towards the given speed.
     * 
     * @param speed   a double indicating the speed to accelerate to
     * @param maxStep a double indicating the maximum speed change
     * @return nothing
     * @see #accelTowardImpl()
     */
    public final void accelToward(double speed, double maxStep) {
        if (!isInitialized)
            throw new IllegalStateException(
                    "Motor must be initialized first");
        accelTowardImpl(speed, maxStep);
    }

    /**
     * The implementation of accelToward.
     * 
     * @param speed   a double indicating the speed to accelerate to
     * @param maxStep a double indicating the maximum speed change
     * @return nothing
     * @see #accelToward()
     */
    protected void accelTowardImpl(double speed, double maxStep) {
        double currentSpeed = getSpeed();
        double newSpeed = speed;
        if (speed > currentSpeed) {
            newSpeed = Math.min(currentSpeed + maxStep, speed);
        } else {
            newSpeed = Math.max(currentSpeed - maxStep, speed);
        }
        setSpeed(newSpeed);
    }

    /**
     * Move to a given angle in radians.
     * 
     * @param angle a double indicating the angle in radians
     * @return nothing
     */
    public final void moveToAngle(double angle) {
        moveTo(Math.PI * getConversionFactor() * angle);
    }

    /**
     * Move a given number of radians.
     * 
     * @param angle a double indicating the number of radians to turn
     * @return nothing
     */
    public final void moveByAngle(double angle) {
        moveTo(Math.PI * getConversionFactor() * angle + getPosition());
    }
}