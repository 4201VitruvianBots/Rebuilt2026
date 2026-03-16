package frc.robot.constants;

public class LED {
  public static final int kPWMPort = 9;
  
  public static final int kLEDCount = 43;
  
  public enum LED_STATES {
    DISABLED("disabled"),
    IDLE("idle"),
    // DRIVING("driving"),
    INTAKING("intaking"),
    SHOOTING("shooting");
    // CLIMBING("climbing");

    private final String animation;

    LED_STATES(String animation) {
      this.animation = animation;
    }

    public String getAnimation() {
      return animation;
    }
  }
}
