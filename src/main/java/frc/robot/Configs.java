package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.WheelConstants;

public final class Configs {

  public static final class MAXSwerveModule {

    public static final SparkBaseConfig SpeedConfig = new SparkMaxConfig() {
      {
        idleMode(IdleMode.kBrake);
        smartCurrentLimit(50);
        encoder.positionConversionFactor(WheelConstants.ROTATIONS_TO_METERS)
               .velocityConversionFactor(WheelConstants.ROTATIONS_TO_METERS / 60.0);

      }
    };

    public static final SparkBaseConfig AngleConfig = new SparkMaxConfig() {
      {
        idleMode(IdleMode.kBrake);
        smartCurrentLimit(20);
        absoluteEncoder.inverted(true);

        closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                  .pid(1.75, 0, 0)
                  .outputRange(
                      -.5, .5)
                  .positionWrappingEnabled(true)
                  .positionWrappingInputRange(0, 1);
      }
    };

  }
}
