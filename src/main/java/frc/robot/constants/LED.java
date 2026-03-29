package frc.robot.constants;

public class LED {
  public static final int kPWMPort = 8;

  public static final int kLEDCount = 42;

  public enum LED_STATES {
    DISABLED_NOT_READY("disabled_not_ready"),
    DISABLED("disabled"),
    IDLE("idle"),
    IDLE_CAN_ERROR("idle_can_error"),
    INTAKING("intaking"),
    SHOOTING("shooting"),
    RED_SHIFT_END("red_shift_end"),
    BLUE_SHIFT_END("blue_shift_end");

    private final String animation;

    LED_STATES(String animation) {
      this.animation = animation;
    }

    public String getAnimation() {
      return animation;
    }
  }
}
 