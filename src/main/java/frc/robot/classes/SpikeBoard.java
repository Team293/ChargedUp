package frc.robot.classes;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class SpikeBoard {
  public ShuffleboardTab tab;
  private HashMap<String, GenericEntry> entries = new HashMap<>();

  public SpikeBoard(String tabTitle, KeyValue... args) {
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
    boolean retVal = true;
    try {
      retVal = entries.get(key).setDouble(val);
    } catch (NullPointerException e) {
      entries.put(key, (GenericEntry) tab.add(key, val).getEntry());
    }
    return retVal;
  }

  public int getInt(String key, int defaultVal) {
    return Long.valueOf(entries.get(key).getInteger(defaultVal)).intValue();
  }

  public boolean setInt(String key, int val) {
    boolean retVal = true;
    try {
      retVal = entries.get(key).setInteger(val);
    } catch (NullPointerException e) {
      entries.put(key, (GenericEntry) tab.add(key, val).getEntry());
    }
    return retVal;
  }

  public boolean setBoolean(String key, boolean val) {
    boolean retVal = true;
    try {
      retVal = entries.get(key).setBoolean(val);
    } catch (NullPointerException e) {
      entries.put(key, (GenericEntry) tab.add(key, val).getEntry());
    }
    return retVal;
  }

  public boolean getBoolean(String key, boolean defaultVal) {
    return entries.get(key).getBoolean(defaultVal);
  }

  public boolean setString(String key, String val) {
    boolean retVal = true;
    try {
      retVal = entries.get(key).setString(val);
    } catch (NullPointerException e) {
      entries.put(key, (GenericEntry) tab.add(key, val).getEntry());
    }
    return retVal;
  }

  public String getString(String key, String defaultVal) {
    return entries.get(key).getString(defaultVal);
  }

  public ShuffleboardTab getTab() {
    return tab;
  }
}
