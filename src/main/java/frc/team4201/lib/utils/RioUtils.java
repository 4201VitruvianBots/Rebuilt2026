package frc.team4201.lib.utils;

/** Utility class for interfacing with the RoboRIO. */
public class RioUtils {
  /** RoboRIO Serial Number. */
  enum SERIAL_NUMBER {
    SIMULATION("");
    private final String serialNumber;

    SERIAL_NUMBER(final String serialNumber) {
      this.serialNumber = serialNumber;
    }

    public String get() {
      return serialNumber;
    }
  }
}
