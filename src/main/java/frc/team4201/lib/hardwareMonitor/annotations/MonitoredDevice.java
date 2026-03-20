package frc.team4201.lib.hardwareMonitor.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface MonitoredDevice {

  enum DEVICE_TYPE {
    PRIMARY,
    SECONDARY,
    BACKUP
  }

  String name() default "";

  DEVICE_TYPE type() default DEVICE_TYPE.PRIMARY;

  String canbus() default "";

  boolean skipDefaultInit() default false;
}
