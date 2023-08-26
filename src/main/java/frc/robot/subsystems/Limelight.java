package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;;


public class Limelight extends SubsystemBase {
  public Limelight(){}

  //Limelight NetworkTables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  /*NetworkTableEntry tl = table.getEntry("tl");
  NetworkTableEntry cl = table.getEntry("cl");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tvert");
  NetworkTableEntry getpipe = table.getEntry("getpipe");
  NetworkTableEntry json = table.getEntry("json");
  NetworkTableEntry tclass = table.getEntry("tclass");
  NetworkTableEntry tc = table.getEntry("tc");*/
  /*
   *  tv	Whether the limelight has any valid targets (0 or 1)
      tx	Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
      ty	Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
      ta	Target Area (0% of image to 100% of image)
      tl	The pipeline’s latency contribution (ms). Add to “cl” to get total latency.
      cl	Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
      tshort	Sidelength of shortest side of the fitted bounding box (pixels)
      tlong	Sidelength of longest side of the fitted bounding box (pixels)
      thor	Horizontal sidelength of the rough bounding box (0 - 320 pixels)
      tvert	Vertical sidelength of the rough bounding box (0 - 320 pixels)
      getpipe	True active pipeline index of the camera (0 .. 9)
      json	Full JSON dump of targeting results
      tclass	Class ID of primary neural detector result or neural classifier result
   */

  
  public double x;
  public double y;
  public double state; 
  public double area;
  public double distance;


  /*
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);

   *  botpose	Robot transform in field-space. Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
      botpose_wpiblue	Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
      botpose_wpired	Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
      camerapose_targetspace	3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
      targetpose_cameraspace	3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
      targetpose_robotspace	3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
      botpose_targetspace	3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
      camerapose_robotspace	3D transform of the camera in the coordinate system of the robot (array (6))
      tid	ID of the primary in-view AprilTag
   */


/*
 * NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>);
      to set this data:

      ledMode	Sets limelight’s LED state
      0	use the LED Mode set in the current pipeline
      1	force off
      2	force blink
      3	force on
      camMode	Sets limelight’s operation mode
      0	Vision processor
      1	Driver Camera (Increases exposure, disables vision processing)
      pipeline	Sets limelight’s current pipeline
      0 .. 9	Select pipeline 0..9
      stream	Sets limelight’s streaming mode
      0	Standard - Side-by-side streams if a webcam is attached to Limelight
      1	PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
      2	PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
      snapshot	Allows users to take snapshots during a match
      0	Reset snapshot mode
      1	Take exactly one snapshot
      crop	Sets the crop rectangle. The pipeline must utilize the default crop rectangle in the web interface. The array must have exactly 4 entries.
      [0]	X0 - Min or Max X value of crop rectangle (-1 to 1)
      [1]	X1 - Min or Max X value of crop rectangle (-1 to 1)
      [2]	Y0 - Min or Max Y value of crop rectangle (-1 to 1)
      [3]	Y1 - Min or Max Y value of crop rectangle (-1 to 1)
      
      
      Ex:
      double[] cropValues = new double[4];
      cropValues[0] = -1.0;
      cropValues[1] = 1.0;
      cropValues[2] = -1.0;
      cropValues[3] = 1.0;
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("crop").setDoubleArray(cropValues);
 */



  //whether the limelight has any valid targets(0 or 1)
  public double getState(){
    return state;
  }
  //get x angle 
  public double getHorizontal(){
    return x;
  }
  //get y angle 
  public double getVertical(){
    return y;
  }
  //get y angle 
  public double getArea(){
    return y;
  }
  //get distance 
  public double getDistance(double verticalAngle, double mountAngle, double limelightHeight, double goalHeight){

    double angleToGoalDegrees = verticalAngle + mountAngle; 
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distance = (goalHeight - limelightHeight) / Math.tan(angleToGoalRadians);
    return distance;
  }

  @Override
  public void periodic() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    state = tv.getDouble(0.0);
    area = ta.getDouble(0.0);
    distance = getDistance(y, LimelightConstants.limelightMountAngleDegrees, LimelightConstants.limelightLensHeightInches, 0);
  }
}

