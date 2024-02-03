package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class CameraInfo {
    /** ID of tag */
    public final int tag_id;

    /** Position of that tag on field */
    public final Pose2d tag_position;

    /** Relative position of tag as seen by camera */
    public final Pose2d tag_view;

    public CameraInfo(int tag_id, Pose2d tag_position, Pose2d tag_view) {
      this.tag_id = tag_id;
      this.tag_position = tag_position;
      this.tag_view = tag_view;
    }

    @Override
    public String toString() {
      return String.format("#%d (%s) seen as %s",
          tag_id,
          FieldInfo.format(tag_position),
          FieldInfo.format(tag_view));
    }
  }