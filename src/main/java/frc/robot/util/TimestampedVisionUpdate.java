package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.Timer;

public class TimestampedVisionUpdate extends Object { 
    public final Translation2d translation;
    public final Rotation2d rotation;
    public final double timestamp;
    public final double stdDev;
    public TimestampedVisionUpdate(Pose2d pose, double timestamp, double stdDev) { 
        this.translation = pose.getTranslation(); 
        this.rotation = pose.getRotation();
        this.timestamp = timestamp; 
        this.stdDev = stdDev;
    }

    public TimestampedVisionUpdate(Transform2d pose, double timestamp, double stdDev) { 
        this.translation = pose.getTranslation(); 
        this.rotation = pose.getRotation();
        this.timestamp = timestamp; 
        this.stdDev = stdDev;
    }

    public TimestampedVisionUpdate(Translation2d translation, Rotation2d rotation, double timestamp, double stdDev) { 
        this.translation = translation; 
        this.rotation = rotation;
        this.timestamp = timestamp; 
        this.stdDev = stdDev;
    }

    public TimestampedVisionUpdate() { 
        this.rotation = new Rotation2d(); 
        this.translation = new Translation2d(); 
        this.timestamp = Timer.getFPGATimestamp(); 
        this.stdDev = 0;
    }
    
    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        TimestampedVisionUpdate other = (TimestampedVisionUpdate) obj;
        return translation.equals(other.translation) && rotation.equals(other.rotation) && timestamp == other.timestamp && Double.compare(stdDev, other.stdDev) == 0;
    }

    @Override
    public int hashCode() {
        return translation.hashCode() ^ rotation.hashCode() ^ Double.hashCode(timestamp) ^ Double.hashCode(stdDev);
    }

  public static final TimestampedVisionUpdateStruct struct = new TimestampedVisionUpdateStruct();
}
