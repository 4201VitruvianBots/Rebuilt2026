package frc.team4201.lib.utils;

import java.util.function.Supplier;

public class MutableSupplier<T> implements Supplier<T> {
  private T value;

  @Override
  public T get() {
    return value;
  }

  public MutableSupplier(T initial) {
    this.value = initial;
  }

  public void set(T value) {
    this.value = value;
  }
}
