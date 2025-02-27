// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.format.SignStyle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle candle;
  private final CANdleConfiguration candleConfig;
  private final int ledNum;
  private Animation ledAnimation;

  public LEDSubsystem() {
    candle = new CANdle(LEDConstants.candle_ID);
    candleConfig = new CANdleConfiguration();

    candleConfig.stripType = LEDStripType.RGB;
    candleConfig.statusLedOffWhenActive = true;
    candleConfig.disableWhenLOS = false;
    candleConfig.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfig);

    ledNum = LEDConstants.ledNum;

    ledAnimation = null;
  }

  public void fireAnimation() {
    ledAnimation = new FireAnimation(0.2, 0.5, ledNum, 0.5, 0.5);
    candle.animate(ledAnimation);
    LEDConstants.LEDFlag = false;
  }

  public void hasGamePiece() {
    candle.animate(null);
    candle.setLEDs(0, 255, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void tracking() {
    ledAnimation = new StrobeAnimation(ledNum, ledNum, ledNum);
    candle.animate(ledAnimation);
    candle.setLEDs(0, 0, 255, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void arrivePosition_Base() {
    candle.animate(null);
    candle.setLEDs(0, 0, 255, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void intakeGamePiece() {
    ledAnimation = new StrobeAnimation(ledNum, ledNum, ledNum);
    candle.animate(ledAnimation);
    candle.setLEDs(255, 0, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void arrivePosition_Intake() {
    candle.animate(null);
    candle.setLEDs(255, 192, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void intakeArriving() {
    ledAnimation = new StrobeAnimation(ledNum, ledNum, ledNum);
    candle.animate(ledAnimation);
    candle.setLEDs(ledNum, ledNum, ledNum, ledNum, ledNum, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void shootGamePiece() {
    ledAnimation = new StrobeAnimation(ledNum, ledNum, ledNum);
    candle.animate(ledAnimation);
    candle.setLEDs(255, 192, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void climbing() {
    ledAnimation = new StrobeAnimation(ledNum, ledNum, ledNum);
    candle.animate(ledAnimation);
    candle.setLEDs(255, 255, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  public void onCage() {
    candle.animate(null);
    candle.setLEDs(255, 255, 0, 0, 0, ledNum);
    LEDConstants.LEDFlag = false;
  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(LEDConstants.LEDFlag) {
      if(LEDConstants.arrivePosition_Base) arrivePosition_Base();
      else if(LEDConstants.tracking) tracking();
      else if(LEDConstants.arrivePosition_Intake) arrivePosition_Intake();
      else if(LEDConstants.intakeArriving) intakeArriving();
      else if(LEDConstants.hasGamePiece) hasGamePiece();
      else if(LEDConstants.hasGamePiece) intakeGamePiece();
      else if(LEDConstants.onCage) onCage();
      else if(LEDConstants.climbing) climbing();
      else if(LEDConstants.fireAnimation) fireAnimation();
    }
  }
}
