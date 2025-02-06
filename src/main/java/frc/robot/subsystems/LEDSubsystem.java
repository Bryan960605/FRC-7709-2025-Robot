// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.format.SignStyle;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
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

  public void hadGamePiece() {
    ledAnimation = null;
    candle.animate(ledAnimation);
    candle.setLEDs(255, 255, 255, 255, ledNum, ledNum);
  }

  public void Tracking() {

  }

  public void IntakeGamePiece() {

  }

  public void arrivePosition() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
