// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * This is a demo program showing the detection of AprilTags. The image is
 * acquired from the USB
 * camera, then any detected AprilTags are marked up on the image and sent to
 * the dashboard.
 *
 * <p>
 * Be aware that the performance on this is much worse than a coprocessor
 * solution!
 */
public class Robot extends TimedRobot {

  // Camera calibration parameters
  // You will need to calibrate your camera to get these values
  // If you do not update these values, the detection will be inaccurate
  private final double CAM_FX = 1603.71;
  private final double CAM_FY = 1600.42;
  private final double CAM_CX = 986.8;
  private final double CAM_CY = 744.1;
  private final int IMG_WIDTH = 1920;  // If the IMG_WIDTH and IMG_HEIGHT do not match the camera's resolution, the camera may refuse to work
  private final int IMG_HEIGHT = 1080;

  private final double TAG_SIZE = 0.1524;

  private final Scalar OUTLINE_COLOR = new Scalar(0, 255, 0);
  private final Scalar BACKGROUND_COLOR = new Scalar(0, 0, 0);
  private final Scalar TEXT_COLOR = new Scalar(0, 0, 255);
  private final Scalar X_AXIS_COLOR = new Scalar(0, 0, 255);
  private final Scalar Y_AXIS_COLOR = new Scalar(0, 255, 0);
  private final Scalar Z_AXIS_COLOR = new Scalar(255, 0, 0);

  @Override
  public void robotInit() {
    var visionThread = new Thread(() -> apriltagVisionThreadProc());
    visionThread.setDaemon(true);
    visionThread.start();
  }

  // Projects a 3d pose into a image coordinate
  public Point project(Pose3d pose) {
    return project(pose.getTranslation());
  }

  // Projects a 3d position into an image coordinate. Pose must be in NED convention
  public Point project(Translation3d position) {
    position = position.div(position.getX());
    return new Point(-position.getY() * CAM_FX + CAM_CX, -position.getZ() * CAM_FY + CAM_CY);
  }

  // Draws a straight line in the image from one point in 3d space to another
  public void drawLine(Mat image, Pose3d pointA, Pose3d pointB, Scalar color, double thicknessInMeters) {
    drawLine(image, pointA.getTranslation(), pointB.getTranslation(), color, thicknessInMeters);
  } 

  // Draws a straight line in the image from one point in 3d space to another
  public void drawLine(Mat image, Translation3d pointA, Translation3d pointB, Scalar color, double thicknessInMeters) {
    double distance = (pointA.getX() + pointB.getX()) / 2;
    int thicknessInPixels = (int)Math.ceil(thicknessInMeters / distance * (CAM_FX + CAM_FY) / 2);
    Imgproc.line(image, project(pointA), project(pointB), color, thicknessInPixels);
  } 

  // Plots X, Y, and Z axes in the image for a given 3D pose
  public void plotAxes(Mat image, Pose3d pose, double axisLength, double lineThicknessInMeters) {
    Transform3d xTrans = new Transform3d(new Translation3d(axisLength, 0, 0), new Rotation3d());
    Transform3d yTrans = new Transform3d(new Translation3d(0, axisLength, 0), new Rotation3d());
    Transform3d zTrans = new Transform3d(new Translation3d(0, 0, axisLength), new Rotation3d());

    drawLine(image, pose, pose.plus(zTrans), Z_AXIS_COLOR, lineThicknessInMeters);
    drawLine(image, pose, pose.plus(yTrans), Y_AXIS_COLOR, lineThicknessInMeters);
    drawLine(image, pose, pose.plus(xTrans), X_AXIS_COLOR, lineThicknessInMeters);
  }

  void apriltagVisionThreadProc() {
    var detector = new AprilTagDetector();

    // look for tag16h5, you can chance this if you want to use another family
    detector.addFamily("tag16h5", 0);

    // Enable multi-thread for faster detection
    var config = detector.getConfig();
    config.numThreads = 6;
    detector.setConfig(config);

    // Set up Pose Estimator
    var poseEstConfig = new AprilTagPoseEstimator.Config(
        TAG_SIZE, CAM_FX, CAM_FY, CAM_CX, CAM_CY);
    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", IMG_WIDTH, IMG_HEIGHT);

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // Init smart dashboard
    SmartDashboard.putNumber("X", 0.07);
    SmartDashboard.putNumber("Y", 0);
    SmartDashboard.putNumber("Z", 0);
    SmartDashboard.putNumber("Roll", 0);
    SmartDashboard.putNumber("Pitch", 0);
    SmartDashboard.putNumber("Yaw", 0);
    SmartDashboard.putBoolean("Correct Cam Convention", false);
    SmartDashboard.putBoolean("Show Axes", false);
    SmartDashboard.putBoolean("Rotate Tag", false);
    SmartDashboard.putBoolean("Show Text Axes", false);
    SmartDashboard.putBoolean("Show Text", false);
    SmartDashboard.putBoolean("Animate Text", false);



    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat. If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      // Detect the april tags
      AprilTagDetection[] detections = detector.detect(grayMat);

      // For each april tag
      for (AprilTagDetection detection : detections) {
        if (detection.getId() != 0) {
          continue;
        }

        // Draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, OUTLINE_COLOR, 2);
        }

        // Determine pose
        Transform3d transform = estimator.estimate(detection);
        Pose3d tagWRTCam = new Pose3d().plus(transform);

        // Convert camera coordinate frame
        if(SmartDashboard.getBoolean("Correct Cam Convention", false))
          tagWRTCam = CoordinateSystem.convert(tagWRTCam, CoordinateSystem.EDN(), CoordinateSystem.NWU());

        // Rotate tag pose to what we would expect
        if(SmartDashboard.getBoolean("Rotate Tag", false))
          tagWRTCam = tagWRTCam.plus(new Transform3d(new Translation3d(), new Rotation3d(Math.PI/2,Math.PI/2, 0)));
        
        // Print tag pose in SmartDashboard
        SmartDashboard.putNumber("Tag X", tagWRTCam.getX());
        SmartDashboard.putNumber("Tag Y", tagWRTCam.getY());
        SmartDashboard.putNumber("Tag Z", tagWRTCam.getZ());
        SmartDashboard.putNumber("Tag Roll", tagWRTCam.getRotation().getX());
        SmartDashboard.putNumber("Tag Pitch", tagWRTCam.getRotation().getY());
        SmartDashboard.putNumber("Tag Yaw", tagWRTCam.getRotation().getZ());
        

        // Plot axes on april tag's pose
        if (SmartDashboard.getBoolean("Show Axes", true))
          plotAxes(mat, tagWRTCam, 0.025, 0.002);

        
        // Build transform to the position of the text
        Translation3d translation = new Translation3d(
          SmartDashboard.getNumber("X", 0), 
          SmartDashboard.getNumber("Y", 0), 
          SmartDashboard.getNumber("Z", 0)
        );
        Rotation3d rotation = new Rotation3d(
          Units.degreesToRadians(SmartDashboard.getNumber("Roll", 0)), 
          Units.degreesToRadians(SmartDashboard.getNumber("Pitch", 0)), 
          Units.degreesToRadians(SmartDashboard.getNumber("Yaw", 0))
        );
        Transform3d tagToTextCenter = new Transform3d(translation, rotation);
        Pose3d textCenterWRTCam = tagWRTCam.plus(tagToTextCenter);
        if (SmartDashboard.getBoolean("Show Text Axes", true))
          plotAxes(mat, textCenterWRTCam, 0.025, 0.002);


        // Animate the text spinning
        if (SmartDashboard.getBoolean("Animate Text", true))
        {
          double yaw = SmartDashboard.getNumber("Yaw", 0);
          yaw += 3;
          yaw = ((yaw + 180) % 360) - 180; // Wrap yaw to [-180, 180)
          SmartDashboard.putNumber("Yaw", yaw);
        }

  
        


        // Draw some text

        // Define the scale of the text and the lines we should draw
        double scale = 0.15 / 16;
        Integer[][] text = {
            { 0, 0, 0, 4 },
            { 0, 2, 2, 2 },
            { 2, 0, 2, 4 },
            { 4, 0, 4, 4 },
            { 4, 4, 6, 4 },
            { 4, 2, 6, 2 },
            { 4, 0, 6, 0 },
            { 8, 0, 8, 4 },
            { 8, 4, 10, 4 },
            { 12, 0, 12, 4 },
            { 12, 4, 14, 4 },
            { 16, 0, 16, 4 },
            { 16, 4, 18, 4 },
            { 18, 4, 18, 0 },
            { 18, 0, 16, 0 }
        };

        // Compute the origin of the text
        Transform3d textOriginToTextCenter = new Transform3d(new Translation3d(0, 9, -2), new Rotation3d()).times(scale);
        Pose3d textOriginWRTCam = textCenterWRTCam.plus(textOriginToTextCenter.inverse());

        // Draw the back background around the space the text will be
        if (SmartDashboard.getBoolean("Show Text", true)) {
          List<MatOfPoint> backgroundBorderPoints = new ArrayList<MatOfPoint>() {
            {
              add(new MatOfPoint(
                  project(textCenterWRTCam
                      .plus(new Transform3d(new Translation3d(0, -10, -3), new Rotation3d()).times(scale))),
                  project(textCenterWRTCam
                      .plus(new Transform3d(new Translation3d(0, -10, 3), new Rotation3d()).times(scale))),
                  project(textCenterWRTCam
                      .plus(new Transform3d(new Translation3d(0, 10, 3), new Rotation3d()).times(scale))),
                  project(textCenterWRTCam
                      .plus(new Transform3d(new Translation3d(0, 10, -3), new Rotation3d()).times(scale)))));
            }
          };
          Imgproc.fillPoly(mat, backgroundBorderPoints, BACKGROUND_COLOR);
        }

        // Draw each line of the text
        if (SmartDashboard.getBoolean("Show Text", true)) {
          for (var line : text) {
            Transform3d pt1Trans = new Transform3d(new Translation3d(0, line[0], -line[1]), new Rotation3d())
                .times(scale);
            Transform3d pt2Trans = new Transform3d(new Translation3d(0, line[2], -line[3]), new Rotation3d())
                .times(scale);
            Pose3d point1 = textOriginWRTCam.plus(pt1Trans);
            Pose3d point2 = textOriginWRTCam.plus(pt2Trans);
            drawLine(mat, point1, point2, TEXT_COLOR, 0.005);
          }
        }

      }

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    detector.close();
  }
}
