
package frc.robot.Subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.GoalConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Library.LimelightHelper.LimelightHelpers;
import frc.robot.Library.team1706.LinearInterpolationTable;

import java.awt.geom.Point2D;

public class Limelight extends SubsystemBase {

  private static Limelight instance = null;
  /** Creates a new Limelight. */
  public NetworkTable m_limTable;
  public double constTx = 4.0;
  public double constTv = 4.0;
  public double constTy = 4.0;
  public double tv ;
  public double ta ;
  public double tx ;
  public double ty ;
  public double simTx = constTx ;
  public double simTy = constTy ;
  public double simTv = constTv ;
  public int simuTxStop = 0;
  public OptionalDouble distancetoTarget = OptionalDouble.empty();
  MedianFilter medianFilter = new MedianFilter(4);

  private static Point2D[] points = new Point2D.Double[] {
    // (ty-angle,distance)
    new Point2D.Double(-24.0, 7.366/*290.0*/), // 242
    new Point2D.Double(-20.0, 6.045/*238.0*/), // 196
    new Point2D.Double(-17.5, 5.258/*207.0*/), // 163
    new Point2D.Double(-15.0, 4.724/*186.0*/), // 141
    new Point2D.Double(-12.5, 4.293/*169.0*/), // 121
    new Point2D.Double(-10.0, 3.912/*154.0*/), // 107
    new Point2D.Double(-5.0, 3.404/*134.0*/), // 96
    new Point2D.Double(0.0, 2.946/*116.0*/), // 85
    new Point2D.Double(5.0, 2.642/*104.0*/), // 77
    new Point2D.Double(10.0, 2.337/*92.0*/),
    new Point2D.Double(15.0, 2.108/*83.0*/),
    new Point2D.Double(20.0, 1.905/*75.0*/)
    //
};
private static LinearInterpolationTable distTable = new LinearInterpolationTable(points);

  
  public Limelight() {
    // m_limTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public static Limelight GetInstance() {
    if (instance == null){
      instance = new Limelight();
    }
    return instance;
  }
  

  @Override
  public void periodic() {
    // LimelightHelpers.getLatestResults("limelight");
    // This method will be called once per scheduler run
  }

}
