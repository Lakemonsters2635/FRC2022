package frc.robot.models;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Vision;

public class PathSelecter {
    HashMap<String, VisionObject[]> pathDictionary;
   
    public void loadPathDictionary() {
        pathDictionary = new HashMap<String, VisionObject[]>();
        try {
            VisionObject[] pathArray =Robot.objectTrackerSubsystem.loadVisionSnapshot("/home/lvuser/PathBlueB.json");
            pathDictionary.put("PathBlueB", pathArray);
        }
        catch (IOException e) {
            System.out.println("error loading path dictionary");
        } 

    }


    public static String choosePath(){
        String pathName = null;
        SmartDashboard.putString("Path", "");
        SmartDashboard.putNumber("Retries", -1);
        Robot.objectTrackerSubsystem.data();
        VisionObject[] objects =  Robot.objectTrackerSubsystem.getObjectsOfType("powerCell");
        int maxRetries = 10;
        int retryCount = 0;
       
        while (retryCount < maxRetries && objects.length <2)
        {
            objects =  Robot.objectTrackerSubsystem.getObjectsOfType("powerCell");
           
            retryCount++;
        }
        SmartDashboard.putNumber("Retries", retryCount);
        String json = Robot.objectTrackerSubsystem.getObjectsJson();
        //System.out.println("Objects: " + (objects[0].z<=42));
        SmartDashboard.putString("ObjectJson", json);
        SmartDashboard.putNumber("Objects length", objects.length);
        if (objects == null || objects.length<2) {
            return null;
        }

        if (objects[0].z<38) {
            double tmp = objects[0].z;
            objects[0].z = objects[1].z;
            objects[1].z = tmp;
            
        }
        SmartDashboard.putNumber("Object[0] z", objects[0].z);
        // if (objects[0].z<=40 && objects[0].z>=38) {
        //     if(Math.abs(objects[1].x)<=15) {
        //         return "PathRedB";
        //     }
        //     else{
        //         return "PathRedA";
        //     }
        // }
        // else {
        //     if(Math.abs(objects[1].x)<=15) {
        //         return "PathBlueB";
        //     }
        //     else {
        //         return "PathBlueA";
        //     }
        // }

        // VisionObject[] objects = Robot.objectTrackerSubsystem.getObjectsOfType("powerCell");

        // if (objects == null || objects.length<2) {
        //     return null;
        // }

        // VisionObject closestObject = Robot.objectTrackerSubsystem.getClosestObject("powerCell");
        // VisionObject secondObject = Robot.objectTrackerSubsystem.getSecondClosestObject("powerCell");

        // if (closestObject.z<=85) {
        //     if(Math.abs(secondObject.x)<=15) {
        //         return "PathRedB";
        //     }
        //     else{
        //         return "PathRedA";
        //     }
        // }
        // else {
        //     if(objects.length == 3) {
        //         return "PathBlueB";
        //     }
        //     else {
        //         return "PathBlueA";
        //     }
        // }



        if (objects[0].z<=85) {
            if(Math.abs(objects[1].x)<=15) {
                pathName = "PathRedB";
            }
            else{
                pathName = "PathRedA";
            }
        }
        else {
            if(objects.length == 3) {
                pathName = "PathBlueB";
            }
            else {
                pathName = "PathBlueA";
            }
        }
        SmartDashboard.putString("Path", pathName);
        return pathName;
    }
    
}
