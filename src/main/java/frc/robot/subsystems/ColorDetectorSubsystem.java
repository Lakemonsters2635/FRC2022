package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;


import com.revrobotics.ColorMatchResult;

import java.util.Dictionary;
import java.util.Hashtable;

import com.revrobotics.ColorMatch;

public class ColorDetectorSubsystem {
  /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
   * The device will be automatically initialized with default parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This
   * can be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these are
   * here as a basic example.
   */
  // yELlow; 0.326172  0.540283  0.133789
  // blUE;  0.205566  0.476807  0.317383
  // GrEEn; 0.234619  0.566406  0.198975
  // rED; 0.384277  0.413818  0.201660
  // Offical Values
  // public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
  // public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
  // public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
  // public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);
  public static final Color kBlueTarget = new Color( 0.205566,  0.476807,  0.317383);
  public static final Color kGreenTarget = new Color(0.234619,  0.566406,  0.198975);
  public static final Color kRedTarget = new Color(0.384277,  0.413818,  0.201660);
  public static final Color kYellowTarget = new Color( 0.326172,  0.540283,  0.133789);

  private ColorMatchResult matchedResult = new ColorMatchResult(Color.kBlack, 0);
  public static Dictionary m_colorDictionary = new Hashtable();
  ///public Dictionary<String, Integer> ageDictionary = new Dictionary<String, Integer>();

  // Rev Color threshold
  // blue 0.143, 0.427, 0.429
  // green 0.197, 0.561, 0.240
  // red 0.561, 0.232, 0.114
  // yellow 0.361, 0.524, 0.113

  public void ColorMatcher() {
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);

    m_colorMatcher.setConfidenceThreshold(0.955);

    m_colorDictionary.put(kBlueTarget, kRedTarget);
    m_colorDictionary.put(kGreenTarget, kYellowTarget);
    m_colorDictionary.put(kRedTarget, kBlueTarget);
    m_colorDictionary.put(kYellowTarget, kGreenTarget);

  }

  public boolean isFinished(Color targetColor){
    Color colorFound = get_color();
    //NOTE: Discuss the following with students
    //What is the difference between comparing objects with "==" or "colorFound.equals"
    return (colorFound.equals(targetColor));
  }

  public double outputColor() {
    Color detectedColor = m_colorSensor.getColor();
    double redColor = detectedColor.red;

    // SmartDashboard.putNumber("red", redColor);
    return redColor;
  }

  // for output and distance calibration to SmartDashboard
  public int outputDistance() {
    int distanceThreshold = 90;
    int currentDistance = m_colorSensor.getProximity();
    if (currentDistance >= distanceThreshold) {
      System.out.println("target found");
      return distanceThreshold;
    }
    return currentDistance;
  }

  // bind this function to the shooter such that it stops spinning when object is in range
  // TODO 2/28 this function needs to have constants refactored and bound to the intake command on new bot
  public boolean outputDistanceForShooterTest() {
    int distanceThreshold = 90;
    int currentDistance = m_colorSensor.getProximity();
    //System.out.println(currentDistance);
    if (currentDistance >= distanceThreshold) {
      System.out.println("target found");
      return true;
    }
    return false;
  }


  public Color get_color() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and
     * can be useful if outputting the color to an RGB LED or similar. To read the
     * raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in well
     * lit conditions (the built in LED is a big help here!). The farther an object
     * is the more light from the surroundings will bleed into the measurements and
     * make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString = "Unknown";
    //m_colorMatcher.matchColor(colorToMatch)
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);

    if (match != null) {
      if (match.color == kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } 
    }


    if (match == null || match.confidence < 0.955) {
      //System.out.println("Unknown");
    } else if ( match != null) { 
       //System.out.println(colorString + "  \tconfidence: " + match.confidence) ;
       SmartDashboard.putNumber("Confidence", match.confidence);
    }
  
    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putString("Detected Color", colorString);
    //FireLog.log("detected_color", colorString);
    if (match != null) {
      return match.color;
    } else {
      return Color.kBlack;
    }
  }
}