// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

import java.util.HashMap;

/** Add your docs here. */
public class ArmData {

    ArmPosEnum armPosEnum = ArmPosEnum.HOME;
    double angle = 0;

    HashMap<String, ArmInfo> data = new HashMap<String, ArmInfo>();
    public ArmData(){
        data.put(ArmPosEnum.HOME.toString(), new ArmInfo(0,1));
        
    }
    public double getAngle(ArmPosEnum _posString){
        return data.get(_posString.toString()).angle;
    }
    public double getWeight(ArmPosEnum _posString){
        return data.get(_posString.toString()).weight;

    }
public class ArmInfo{
    public double angle;
    public double weight;
    public ArmInfo(double _angle, double _weight){
        angle = _angle;
        weight = _weight;
    }
}

}
