// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.trobot5013lib.TrobotUtil;
import frc.robot.trobot5013lib.led.BlinkingPattern;
import frc.robot.trobot5013lib.led.ChaosPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.RainbowPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;

public class StatusLED extends SubsystemBase {
  private TrobotAddressableLED m_AddressableLED = new TrobotAddressableLED(9, 39);

  private String mPattern = "none";
  private TrobotAddressableLEDPattern autoPattern = new ChasePattern(new Color[] { Color.kDarkRed, Color.kBlue },
      3);
  private TrobotAddressableLEDPattern matchTimePattern =  new ChasePattern(
            new Color[] { Color.kDarkRed, Color.kOrangeRed, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple },
            1);
  private TrobotAddressableLEDPattern shuttlingPattern = new BlinkingPattern(Color.kDarkRed, 0.1);
  private TrobotAddressableLEDPattern ampPattern = new BlinkingPattern(Color.kWhite, 0.1);
  private TrobotAddressableLEDPattern duckingPattern = new BlinkingPattern(Color.kDeepPink, 0.1);
  private TrobotAddressableLEDPattern shotReadyPatterh = new SolidColorPattern(Color.kGreen);
  private TrobotAddressableLEDPattern hasTargetPattern = new BlinkingPattern(Color.kGreen, 0.1);
  private TrobotAddressableLEDPattern nonePattern = new BlinkingPattern(Color.kBlue, 0.1);
  private TrobotAddressableLEDPattern disabledPattern = new RainbowPattern();
  
  

  /** Creates a new StatusLED. */
  public StatusLED() {

  }

  @Override
  public void periodic() {
    // check conditions and set patterns
    boolean isDisabled = DriverStation.isDisabled();
    boolean isAutonomous = DriverStation.isAutonomous();
    double matchTime = DriverStation.getMatchTime();

    if (isAutonomous && !isDisabled) {
      if (mPattern != "Auto Police") {
        m_AddressableLED.setPattern(autoPattern);
      }
      mPattern = "Auto Police";
    } else if (!isDisabled) {
      if (matchTime >= 18 && matchTime <= 23) {
        if (mPattern != "Match Time") {
          m_AddressableLED.setPattern(matchTimePattern);
        }
        mPattern = "Match Time";
        SmartDashboard.putString("LED Command", mPattern);
      } 
    }
    else {
      if (mPattern != "Disabled Rainbow") {
        m_AddressableLED.setPattern(disabledPattern);
      }
      mPattern = "Disabled Rainbow";
    }
    
  }
}
