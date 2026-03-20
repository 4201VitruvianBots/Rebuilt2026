package frc.team4201.lib.hardwareMonitor.annotations;

import frc.team4201.lib.hardwareMonitor.HardwareMonitor;

public abstract class BaseDeviceMonitor<T> {
  private final Class<T> m_clazz;

  protected BaseDeviceMonitor(Class<T> clazz) {
    this.m_clazz = clazz;
  }

  protected abstract HardwareMonitor.HEALTH_STATUS checkHealth(T object);
}
