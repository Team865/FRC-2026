package frc.robot.util;

public class Benchmark {
  private final String name;
  private long startTimeNanos = 0;

  public Benchmark(String name) {
    this.name = name;
  }

  /** Unit for Time */
  public enum TimeUnit {
    SECONDS("s", 1_000_000_000),
    MILLISECONDS("ms", 1_000_000),
    MICROSECONDS("µs", 1_000),
    NANOSECONDS("ns", 1);

    public final String symbol;
    public final int multiplier;

    private TimeUnit(String symbol, int multiplier) {
      this.symbol = symbol;
      this.multiplier = multiplier;
    }
  }

  public void start() {
    startTimeNanos = System.nanoTime();
  }

  public void end(TimeUnit timeUnit) {
    long timeElapsed = System.nanoTime() - startTimeNanos;

    System.out.printf(
        "Time to run %s: %s %s\n",
        name, (double) timeElapsed / timeUnit.multiplier, timeUnit.symbol);
  }

  /**
   * Times how long it takes to run the {@code target} function
   *
   * @param target The function to run
   * @return the time taken in nanoseconds
   */
  public static long time(Runnable target) {
    long startTime = System.nanoTime();
    target.run();
    return System.nanoTime() - startTime;
  }

  /**
   * Times how long it takes to run the {@code target} function, and prints out the time taken
   *
   * @param identifier An identifier to represent the function. Used when outputting the time
   *     elapsed
   * @param target The function to run
   * @param timeUnit The time unit to print the result as
   * @return The time taken to run the function in nanoseconds
   */
  public static long timedPrint(String identifier, Runnable target, TimeUnit timeUnit) {
    long timeElapsed = time(target);

    System.out.println(
        String.format(
            "Time taken to run %s: %s %s",
            identifier, (double) timeElapsed / timeUnit.multiplier, timeUnit.symbol));

    return timeElapsed;
  }

  /**
   * Defaults time unit to milliseconds
   *
   * @see #timedPrint(String, Runnable, TimeUnit)
   */
  public static long timedPrint(String identifier, Runnable target) {
    return timedPrint(identifier, target, TimeUnit.MILLISECONDS);
  }
}
