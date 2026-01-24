package utils;

import java.lang.reflect.Field;

public class TestUtils {
  public static void setPrivateField(Object instance, String fieldName, Object valueToSet) {
    try {
      Field f = getDeclaredFieldRecursive(instance.getClass(), fieldName);
      f.setAccessible(true);
      f.set(instance, valueToSet);
    } catch (Exception e) {
      System.out.println(
          "Could not set field '" + fieldName + "' in Object '" + instance.toString() + "'");
      e.printStackTrace();
    }
  }

  public static Object getPrivateObject(Object instance, String fieldName) {
    try {
      Field f = getDeclaredFieldRecursive(instance.getClass(), fieldName);
      f.setAccessible(true);
      return f.get(instance);
    } catch (Exception e) {
      System.out.println(
          "Could not get field '" + fieldName + "' in Object '" + instance.toString() + "'");
      e.printStackTrace();
      return null;
    }
  }

  private static Field getDeclaredFieldRecursive(Class<?> currentClass, String fieldName)
      throws NoSuchFieldException {
    try {
      return currentClass.getDeclaredField(fieldName);
    } catch (NoSuchFieldException e) {
      if (currentClass.getSuperclass() == null || currentClass == Object.class) {
        throw e;
      } else {
        return getDeclaredFieldRecursive(currentClass.getSuperclass(), fieldName);
      }
    }
  }
}
