package frc.robot.util;

import java.nio.ByteBuffer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;

public class TimestampedVisionUpdateStruct implements Struct<TimestampedVisionUpdate> {
  @Override
  public Class<TimestampedVisionUpdate> getTypeClass() {
    return TimestampedVisionUpdate.class;
  }

  @Override
  public String getTypeName() {
    return "TimestampedVisionUpdate";
  }

  @Override
  public int getSize() {
    return Pose2d.struct.getSize() + 2*kSizeDouble;
  }

  @Override
  public String getSchema() {
    return "Pose2d pose;double timestamp;double stdDev";
  }

  @Override
  public Struct<?>[] getNested() {
    return new Struct<?>[] {Pose2d.struct};
  }

  @Override
  public TimestampedVisionUpdate unpack(ByteBuffer bb) {
    Pose2d pose = Pose2d.struct.unpack(bb);
    double timestamp = bb.getDouble();
    double stdDev = bb.getDouble();
    return new TimestampedVisionUpdate(pose, timestamp, stdDev);
  }

  @Override
  public void pack(ByteBuffer bb, TimestampedVisionUpdate value) {
    Pose2d.struct.pack(bb, new Pose2d(value.translation, value.rotation));
    bb.putDouble(value.timestamp);
    bb.putDouble(value.stdDev);
  }

  @Override
  public boolean isImmutable() {
    return true;
  }
}