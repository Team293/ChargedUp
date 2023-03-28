package frc.robot.classes;

public class KeyValue {
  public String k;
  public Object v;
  public int x;
  public int y;

  public KeyValue(String k, Object v) {
    this.k = k;
    this.v = v;
    this.x = -1;
    this.y = -1;
  }

  public KeyValue(String k, Object v, int x, int y) {
    this.k = k;
    this.v = v;
    this.x = x;
    this.y = y;
  }

}
