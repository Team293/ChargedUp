package frc.robot.classes;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Smartboard {
  public ShuffleboardTab tab;
  private HashMap<String, GenericEntry> entries = new HashMap<>();

  public Smartboard(String tabTitle, KeyValue... args) {
    tab = Shuffleboard.getTab(tabTitle);
    for (KeyValue arg : args) {
      if (arg.y == -1 || arg.x == -1) {
        entries.put(arg.k, (GenericEntry) tab.add(arg.k, arg.v).getEntry());
      } else {
        entries.put(arg.k, (GenericEntry) tab.add(arg.k, arg.v).withPosition(arg.x, arg.y).getEntry());
      }
    }
  }

  public double getDouble(String key, double defaultVal) {
    return entries.get(key).getDouble(defaultVal);
  }

  public boolean setDouble(String key, double val) {
    try {
      entries.get(key).setDouble(val);
      return true;
    } catch (NullPointerException e) {
      entries.put(key, (GenericEntry) tab.add(key, val).getEntry());
      return false;
    }
  }

  public boolean setBoolean(String key, boolean val) {
    try {
      entries.get(key).setBoolean(val);
      return true;
    } catch (NullPointerException e) {
      entries.put(key, (GenericEntry) tab.add(key, val).getEntry());
      return false;
    }
  }

  public ShuffleboardTab getTab() {
    return tab;
  }
}
