package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.WheelConstants;

public final class Configs {
     public static final class MAXSwerveModule {
        public static final SparkMaxConfig speedConfig = new SparkMaxConfig();
        public static final SparkMaxConfig angleConfig = new SparkMaxConfig();

        static {
            speedConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(50);
            speedConfig.encoder
                .positionConversionFactor(WheelConstants.ROTATIONS_TO_METERS)
                .velocityConversionFactor(WheelConstants.ROTATIONS_TO_METERS/60.0);

            speedConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

            angleConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(20);
            angleConfig.absoluteEncoder
                .inverted(true)
                .positionConversionFactor(WheelConstants.TURNING_FACTOR)
                .velocityConversionFactor(WheelConstants.TURNING_FACTOR/60.0);
            angleConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(1.75,0,0)
                .outputRange(-1, 1)
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, WheelConstants.TURNING_FACTOR);
        }

        public static SparkMaxConfig getSpeedConfig() {
            return speedConfig;
        }

        public static SparkMaxConfig getAngleConfig() {
            return angleConfig;
        }
    }
}
