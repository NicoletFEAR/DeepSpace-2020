/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Subsystem representing the compressor
 */
public class CompressAir extends Subsystem {
  private Compressor theCompressor;

  public CompressAir() {
    theCompressor = new Compressor(RobotMap.compressormodule);
  }

  public void toggleCompressor() {
    if (theCompressor.enabled() == false) {
      theCompressor.start();
    } else {
      theCompressor.stop();
    }
  }

  public boolean isEnabled() {
    return theCompressor.enabled();
  }

  @Override
  public void initDefaultCommand() {
  }
}