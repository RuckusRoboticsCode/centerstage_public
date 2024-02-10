package org.firstinspires.ftc.teamcode.helper;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.auto.util.CustomAprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.ArrayList;

public class CustomAprilTagLibrary {

    public final AprilTagMetadata[] data;

    private CustomAprilTagLibrary(AprilTagMetadata[] data)
    {
        this.data = data;
    }

    public AprilTagMetadata lookupTag(int id)
    {
        for (AprilTagMetadata tagMetadata : data)
        {
            if (tagMetadata.id == id)
            {
                return tagMetadata;
            }
        }

        return null;
    }

    public static CustomAprilTagLibrary getCenterStageTagLibrary()
    {
        return new CustomAprilTagLibrary.Builder()
                .addTag(1, "BlueAllianceLeft",
                        2, new VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(2, "BlueAllianceCenter",
                        2, new VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(3, "BlueAllianceRight",
                        2, new VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(4, "RedAllianceLeft",
                        2, new VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(5, "RedAllianceCenter",
                        2, new VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(6, "RedAllianceRight",
                        2, new VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0))
                .addTag(7, "RedAudienceWallLarge",
                        5, new VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(8, "RedAudienceWallSmall",
                        2, new VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(9, "BlueAudienceWallSmall",
                        2, new VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .addTag(10, "BlueAudienceWallLarge",
                        5, new VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
                        new Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0))
                .build();
    }

    public static class Builder
    {
        private ArrayList<AprilTagMetadata> data = new ArrayList<>();
        private boolean allowOverwrite = false;

        /**
         * Set whether to allow overwriting an existing entry in the tag
         * library with a new entry of the same ID
         * @param allowOverwrite whether to allow overwrite
         * @return the {@link AprilTagLibrary.Builder} object, to allow for method chaining
         */
        public CustomAprilTagLibrary.Builder setAllowOverwrite(boolean allowOverwrite)
        {
            this.allowOverwrite = allowOverwrite;
            return this;
        }

        /**
         * Add a tag to this tag library
         * @param aprilTagMetadata the tag to add
         * @return the {@link AprilTagLibrary.Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public CustomAprilTagLibrary.Builder addTag(AprilTagMetadata aprilTagMetadata)
        {
            for (AprilTagMetadata m : data)
            {
                if (m.id == aprilTagMetadata.id)
                {
                    if (allowOverwrite)
                    {
                        // This is ONLY safe bc we immediately stop iteration here
                        data.remove(m);
                        break;
                    }
                    else
                    {
                        throw new RuntimeException("You attempted to add a tag to the library when it already contains a tag with that ID. You can call .setAllowOverwrite(true) to allow overwriting the existing entry");
                    }
                }
            }

            data.add(aprilTagMetadata);
            return this;
        }

        /**
         * Add a tag to this tag library
         * @param id the ID of the tag
         * @param name a text name for the tag
         * @param size the physical size of the tag in the real world (measured black edge to black edge)
         * @param fieldPosition a vector describing the tag's 3d translation on the field
         * @param distanceUnit the units used for size and fieldPosition
         * @param fieldOrientation a quaternion describing the tag's orientation on the field
         * @return the {@link AprilTagLibrary.Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public CustomAprilTagLibrary.Builder addTag(int id, String name, double size, VectorF fieldPosition, DistanceUnit distanceUnit, Quaternion fieldOrientation)
        {
            return addTag(new AprilTagMetadata(id, name, size, fieldPosition, distanceUnit, fieldOrientation));
        }

        /**
         * Add a tag to this tag library
         * @param id the ID of the tag
         * @param name a text name for the tag
         * @param size the physical size of the tag in the real world (measured black edge to black edge)
         * @param distanceUnit the units used for size and fieldPosition
         * @return the {@link AprilTagLibrary.Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public CustomAprilTagLibrary.Builder addTag(int id, String name, double size, DistanceUnit distanceUnit)
        {
            return addTag(new AprilTagMetadata(id, name, size, new VectorF(0,0,0), distanceUnit, Quaternion.identityQuaternion()));
        }

        /**
         * Add multiple tags to this tag library
         * @param library an existing tag library to add to this one
         * @return the {@link AprilTagLibrary.Builder} object, to allow for method chaining
         * @throws RuntimeException if trying to add a tag that already exists
         * in this library, unless you called {@link #setAllowOverwrite(boolean)}
         */
        public CustomAprilTagLibrary.Builder addTags(AprilTagLibrary library)
        {
            for (AprilTagMetadata m : library.getAllTags())
            {
                // Delegate to this implementation so we get duplicate checking for free
                addTag(m);
            }
            return this;
        }

        /**
         * Create an {@link AprilTagLibrary} object from the specified tags
         * @return an {@link AprilTagLibrary} object
         */
        public CustomAprilTagLibrary build()
        {
            return new CustomAprilTagLibrary(data.toArray(new AprilTagMetadata[0]));
        }
    }
}
