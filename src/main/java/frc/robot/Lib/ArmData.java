// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Lib;

import java.util.HashMap;

/** Add your docs here. */
public class ArmData {




    HashMap<String, ArmInfo> data = new HashMap<String, ArmInfo>();
    public ArmData(){          // ARM INFO( Bicep/Shoulder   Forearm/Elbow    Wrist/Intake)

        
        data.put(ArmPosEnum.HOME.toString(), new ArmInfo(0,0));
        /******************  CONE DATA **************************/
        data.put(ArmPosEnum.WALL_CONE.toString(), new ArmInfo(90,  0));
        data.put(ArmPosEnum.MID_CONE.toString(), new ArmInfo(28.4,  -32));
        data.put(ArmPosEnum.LOW_CONE.toString(), new ArmInfo(0,  0));
        data.put(ArmPosEnum.FLOOR_FRONT_CUBE.toString(), new ArmInfo(-12, 0));

        /******************  CUBE DATA **************************/

        data.put(ArmPosEnum.FAR_CUBE.toString(), new ArmInfo(82,  20));
        data.put(ArmPosEnum.MID_CUBE.toString(), new ArmInfo(0, 0));
        data.put(ArmPosEnum.LOW_CUBE.toString(), new ArmInfo(0,  0));

        data.put(ArmPosEnum.FLOOR_FRONT_CUBE.toString(), new ArmInfo(-10, -105));
        /******************  AUTO ARM DATA **************************/
        
        data.put(ArmPosEnum.AUTO_FRONT_CUBE.toString(), new ArmInfo(-12, 108));


    }
    public double getBicepAngle(ArmPosEnum _posString){
        return data.get(_posString.toString()).bAngle;
    }

    public double getHandAngle(ArmPosEnum _posString){
        return data.get(_posString.toString()).hAngle;
    }

public class ArmInfo{

   public double bAngle = 0;

   public double hAngle = 0;

    public ArmInfo(double shoulderAngle, double wristAngle){
        bAngle = shoulderAngle;

        hAngle = wristAngle;
    }
}

}
