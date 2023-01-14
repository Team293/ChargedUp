package frc.robot.classes.spikemotor;

/**
* An abstracted motor interface designed for easy replacement.
*/
abstract class SpikeMotor {
    private boolean isInitialized = false;

    /**
    * Initializes this motor with the given device number.
    *
    * @param deviceNumber    an int giving the device number
    * @return                nothing
    * @throws                IllegalStateException if already initialized
    * @see                   #initImpl(int deviceNumber)
    */
    public final void init(int deviceNumber) {
        if (isInitialized)
            throw new IllegalStateException(
                "Motor cannot be initialized twice"
            );
        initImpl(deviceNumber);
        isInitialized = true;
    }

    /**
    * Sets the speed of the motor to the given value in ft/s.
    *
    * @param speed    a double indicating the speed
    * @return         nothing
    * @throws         IllegalStateException if not initialized
    * @see            #setSpeedImpl(double speed)
    */
    public final void setSpeed(double speed) {
        if (!isInitialized)
            throw new IllegalStateException(
                "Motor must be initialized first"
            );
        setSpeedImpl(speed);
    }

    /**
    * Gets the speed of the motor in ft/s.
    *
    * @return    a double indicating the speed
    * @throws    IllegalStateException if not initialized
    * @see       #getSpeedImpl()
    */
    public final double getSpeed() {
        if (!isInitialized)
            throw new IllegalStateException(
                "Motor must be initialized first"
            );
        return getSpeedImpl();
    }

    /**
    * Gets the position of the motor in feet.
    *
    * @return    a double indicating the position
    * @throws    IllegalStateException if not initialized
    * @see       #getPositionImpl()
    */
    public final double getPosition() {
        if (!isInitialized)
            throw new IllegalStateException(
                "Motor must be initialized first"
            );
        return getPositionImpl();
    }

    /**
    * The implementation of init.
    *
    * @param deviceNumber    an int giving the device number
    * @return                nothing
    * @throws                IllegalStateException if already initialized
    * @see                   #init(int deviceNumber)
    */
    abstract protected void initImpl(int deviceNumber);

    /**
    * The implementation of setSpeed.
    * @param speed    a double indicating the speed
    * @return         nothing
    * @throws         IllegalStateException if not initialized
    * @see            #setSpeed(double speed)
    */
    abstract protected void setSpeedImpl(double speed);

    /**
    * The implementation of getSpeed.
    *
    * @return    a double indicating the speed
    * @throws    IllegalStateException if not initialized
    * @see       #getSpeed()
    */
    abstract protected double getSpeedImpl();

    /**
    * The implementation of getPosition.
    *
    * @return    a double indicating the position
    * @throws    IllegalStateException if not initialized
    * @see       #getPosition()
    */
    abstract protected double getPositionImpl();
}
