// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.deceivers.util;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class SparkMaxUtil {

    public static void checkError(REVLibError error, String message){
        if (error != REVLibError.kOk){
            DriverStation.reportError(message + ": " + error.name(), false);
        }
    }

    public static void checkErrorandThrow(REVLibError error, String message){
        if (error != REVLibError.kOk){
            throw new RuntimeException(message + ": " + error.name());
        }
    }
}
