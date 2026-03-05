package frc.team4201.lib.utils;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class POVUtils {
  /**
   * Generates a trigger that will activate only if the primary trigger has been activated. Will
   * deactivate once both triggers are deactivated.
   *
   * @param primary The trigger that must be activated first to latch the trigger on.
   * @param secondary The trigger that can activate the latched trigger as long as the primary
   *     trigger is latched on.
   */
  private static Trigger generateLatchedTrigger(Trigger primary, Trigger secondary) {
    return new Trigger(
        new BooleanSupplier() {
          private boolean latched = false;

          @Override
          public boolean getAsBoolean() {
            boolean a = primary.getAsBoolean();
            if (!latched && a) {
              latched = true; // latch when A first activates
            }
            return latched && (a || secondary.getAsBoolean());
          }
        });
  }

  /**
   * Generates a trigger that will activate when the POV is up, including the up-left and up-right
   * diagonals.
   *
   * @param controller The controller to read the POV from.
   */
  public static Trigger povUpWithTilt(CommandGenericHID controller) {
    return generateLatchedTrigger(
        controller.povUp(), controller.povUpLeft().or(controller.povUpRight()));
  }

  /**
   * Generates a trigger that will activate when the POV is down, including the down-left and
   * down-right diagonals.
   *
   * @param controller The controller to read the POV from.
   */
  public static Trigger povDownWithTilt(CommandGenericHID controller) {
    return generateLatchedTrigger(
        controller.povDown(), controller.povDownLeft().or(controller.povDownRight()));
  }

  /**
   * Generates a trigger that will activate when the POV is left, including the up-left and
   * down-left diagonals.
   *
   * @param controller The controller to read the POV from.
   */
  public static Trigger povLeftWithTilt(CommandGenericHID controller) {
    return generateLatchedTrigger(
        controller.povLeft(), controller.povUpLeft().or(controller.povDownLeft()));
  }

  /**
   * Generates a trigger that will activate when the POV is right, including the up-right and
   * down-right diagonals.
   *
   * @param controller The controller to read the POV from.
   */
  public static Trigger povRightWithTilt(CommandGenericHID controller) {
    return generateLatchedTrigger(
        controller.povRight(), controller.povUpRight().or(controller.povDownRight()));
  }
}
