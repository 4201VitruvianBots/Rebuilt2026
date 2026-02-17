package frc.robot.constants;

public class LED {
  public enum LED_STATES {
    DISABLED("disabled"),
    IDLE("idle"),
    DRIVING("driving"),
    INTAKING("intaking"),
    SHOOTING("shooting"),
    CLIMBING("climbing");

    private final String animation;

    LED_STATES(String animation) {
      this.animation = animation;
    }

    public String getAnimation() {
      return animation;
    }
  }
}
