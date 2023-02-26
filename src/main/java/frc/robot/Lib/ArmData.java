// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

import java.util.HashMap;

/** Add your docs here. */
public class ArmData {




    HashMap<String, ArmInfo> data = new HashMap<String, ArmInfo>();
    public ArmData(){
        data.put(ArmPosEnum.HOME.toString(), new ArmInfo(0,0,0));
        data.put(ArmPosEnum.FAR_CONE.toString(), new ArmInfo(80, 180, 0));
        data.put(ArmPosEnum.FAR_CUBE.toString(), new ArmInfo(75, 170, -5));
        data.put(ArmPosEnum.MID_CONE.toString(), new ArmInfo(50, 90, 0));
        data.put(ArmPosEnum.MID_CUBE.toString(), new ArmInfo(45, 75, 0));
        data.put(ArmPosEnum.LOW_CONE.toString(), new ArmInfo(5, 100, 10));
        data.put(ArmPosEnum.LOW_CUBE.toString(), new ArmInfo(5, 155, 20));
        data.put(ArmPosEnum.WALL_CONE.toString(), new ArmInfo(-5, 25, -10));
        data.put(ArmPosEnum.WALL_CUBE.toString(), new ArmInfo(-5, 25, -20));
        data.put(ArmPosEnum.FLOOR_BACK_CUBE.toString(), new ArmInfo(-20, 75, 0));

    }
    public double getBicepAngle(ArmPosEnum _posString){
        return data.get(_posString.toString()).bAngle;
    }
    public double getForearmAngle(ArmPosEnum _posString){
        return data.get(_posString.toString()).fAngle;
    }

public class ArmInfo{

   public double bAngle = 0;
   public double fAngle = 0;
   public double hAngle = 0;

    public ArmInfo(double _bAngle, double _fAngle, double _hAngle){
        bAngle = _bAngle;
        fAngle = _fAngle;
        hAngle = _hAngle;
    }
}

}
