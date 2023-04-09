package frc.robot.classes;

public class KeyValue<T> {
    public String k;
    public T v;
    public int x;
    public int y;

    /**
     * Constructor for KeyValue class.
     * 
     * @param k - key
     * @param v - value
     */
    public KeyValue(String k, T v) {
        this.k = k;
        this.v = v;
        this.x = -1;
        this.y = -1;
    }

    /**
     * Overloaded constructor for KeyValue class with x and y coordinates
     * 
     * @param k - key
     * @param v - value
     * @param x - x coordinate
     * @param y - y coordinate
     */
    public KeyValue(String k, T v, int x, int y) {
        this.k = k;
        this.v = v;
        this.x = x;
        this.y = y;
    }
}
