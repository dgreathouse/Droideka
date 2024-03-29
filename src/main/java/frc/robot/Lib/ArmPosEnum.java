// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

/** 
 * 10 Posible positions:
    HOME               = LEFT & RIGHT Bumper
    WALL_CONE,         = 
    WALL_CUBE,
    FLOOR_FRONT_BLOB,
    FAR_CONE,
    FAR_CUBE,
    MID_CONE,
    MID_CUBE,
    LOW_CONE,
    LOW_CUBE,
 */

public enum ArmPosEnum {
    HOME,
    WALL_CONE,
    FLOOR_FRONT_CUBE,
    FAR_CUBE,
    MID_CONE,
    MID_CUBE,
    LOW_CONE,
    LOW_CUBE,
    AUTO_FRONT_CUBE,
    AUTO_HOME,
    FAR_CUBE_REVERSE,
    MID_CUBE_REVERSE

}
