package frc.team4201.lib.hardwareMonitor;

import com.google.testing.compile.Compilation;
import com.google.testing.compile.CompilationSubject;
import com.google.testing.compile.Compiler;
import com.google.testing.compile.JavaFileObjects;
import java.io.IOException;
import java.util.List;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;

public class TestMonitoredGenerator {

  public static final int kJavaVersion = 17;
  public static final List<Object> kJavaVersionOptions =
      List.of("-source", kJavaVersion, "-target", kJavaVersion);

  @Disabled
  void testGeneratedFileFromAnnotation() {
    String source =
        """
                  package frc.team4201.lib.hardware;

                  import com.ctre.phoenix6.hardware.TalonFX;
                  import frc.team4201.lib.hsm.Monitored;

                  public class Example {

                    @Monitored
                    private TalonFX m_talon = new TalonFX(0);
                  }
                """;

    String expectedGeneratedSource =
        """
                    package frc.team4201.lib.hardware;

                    import frc.team4201.lib.hsm.BaseDeviceMonitor;
                    import frc.team4201.lib.hsm.HardwareMonitor;
                    import frc.team4201.lib.hsm.Monitored;

                    public class ExampleLogger extends BaseDeviceMonitor<Example> {
                      public ExampleLogger() {
                        super(Example.class);
                      }

                      @Override
                      public void update(HardwareMonitor monitor, Example object) {
                      }
                    }
                    """;

    assertGenerates(source, expectedGeneratedSource);
  }

  @Disabled
  void testGeneratedMonitoredClass() {
    String source =
        """
              package frc.team4201.lib.hardware;

              import frc.team4201.lib.hsm.BaseDeviceMonitor;
              import frc.team4201.lib.hsm.HardwareMonitor;
              import frc.team4201.lib.hsm.HealthStatusFor;

              class TalonFX {}

              @HealthStatusFor(TalonFX.class)
              public class Example extends BaseDeviceMonitor<TalonFX> {
                public Example() {
                  super(TalonFX.class);
                }

                @Override
                public void update(HardwareMonitor monitor, TalonFX object) {
                }
              }
            """;

    String expectedGeneratedSource =
        """
              package frc.team4201.lib.hsm;

              import frc.team4201.lib.hardware.Example;

              public final class HardwareMonitor {

                public static final Example example = new Example();
              }
              """;

    assertGenerates(source, expectedGeneratedSource, "HardwareMonitor");
  }

  private void assertGenerates(String loggedClassContent, String loggerClassContent) {
    assertGenerates(loggedClassContent, loggerClassContent, "Example");
  }

  private void assertGenerates(
      String loggedClassContent, String loggerClassContent, String fileNameFilter) {
    Compilation compilation =
        Compiler.javac()
            .withOptions(kJavaVersionOptions)
            //            .withProcessors(new AnnotationProcessor())
            .compile(
                JavaFileObjects.forSourceString(
                    "frc.team4201.lib.hardware.Example", loggedClassContent));

    CompilationSubject.assertThat(compilation).succeeded();
    var generatedFiles = compilation.generatedSourceFiles();
    var generatedFile =
        generatedFiles.stream()
            .filter(jfo -> jfo.getName().contains(fileNameFilter))
            .findFirst()
            .orElseThrow(() -> new IllegalStateException("Logger file was not generated!"));
    try {
      var content = generatedFile.getCharContent(false);
      String expected = loggerClassContent.replace("\r\n", "\n").replaceAll(" +", " ").trim();
      String actual = content.toString().replace("\r\n", "\n").replaceAll(" +", " ").trim();
      Assertions.assertEquals(expected, actual);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}
