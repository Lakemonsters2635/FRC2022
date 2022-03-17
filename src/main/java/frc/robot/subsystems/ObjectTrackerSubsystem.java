package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;
import com.google.gson.Gson;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.models.VisionObject;


public class ObjectTrackerSubsystem extends Subsystem {
	NetworkTable monsterVision; 
    VisionObject[] foundObjects; 
    private String jsonString;

    // rotation matrix
    private double cameraTilt = 20.0 * Math.PI / 180.0; 
    private double[] cameraOffset = {0.0, 18.25, 9.0}; // goes {x, y, z}

    private double sinTheta = Math.sin(cameraTilt);
    private double cosTheta = Math.cos(cameraTilt);

	// Put methods for controlling this subsystem
    // here. Call these from Commands.
	public ObjectTrackerSubsystem(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault(); 
        monsterVision = inst.getTable("MonsterVision");	
        jsonString = "";
        // monsterVision.addEntryListener(
        //     "ObjectTracker",
        //     (monsterVision, key, entry, value, flags) -> {
        //    System.out.println("ObjectTracker changed value: " + value.getValue());
        // }, 
        // EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);          
    }
    
    
    public void data() {
        Gson gson = new Gson();
        NetworkTableEntry entry = monsterVision.getEntry("ObjectTracker");
        if(entry==null) {
            return;
        }
        jsonString = entry.getString("ObjectTracker");
        foundObjects = gson.fromJson(jsonString, VisionObject[].class);
        applyRotationTranslationMatrix();
        // for (VisionObject object : foundObjects){
        //     System.out.format("%s %.1f %.1f %.1f %.1f\n",object.objectLabel, object.x, object.y, object.z, object.confidence);
        // }       
    }

    private void applyRotationTranslationMatrix() {
        for (int i = 0; i < foundObjects.length; i++) {
            double x = foundObjects[i].x; 
            double y = foundObjects[i].y; 
            double z = foundObjects[i].z; 

            // rotation + translation
            foundObjects[i].x = x + cameraOffset[0]; 
            foundObjects[i].y = y * cosTheta - z * sinTheta + cameraOffset[1]; 
            foundObjects[i].z = y * sinTheta + z * cosTheta + cameraOffset[2];
        }
    }
    
    public String getObjectsJson()
    {
        return jsonString;
    }
    
    // private NetworkTableEntry getEntry(Integer index, String subkey) {
    //     try {
    //         NetworkTable table = monsterVision.getSubTable(index.toString());
    //         NetworkTableEntry entry = table.getEntry(subkey);
    //         return entry;
    //     }
    //     catch (Exception e){
    //         return null;
    //     } 
    // }
	
	public VisionObject getClosestObject(String objectLabel) {
        VisionObject[] objects = getObjectsOfType(objectLabel);
        if (objects == null || objects.length == 0) {
            return null; 
        }
        return objects[0];
    }
    public VisionObject getSecondClosestObject(String objectLabel) {
        VisionObject[] objects = getObjectsOfType(objectLabel);
        if (objects == null || objects.length == 0) {
            return null; 
        }
        return objects[1];
    }
    

    public int numberOfObjects() {
        return foundObjects.length; 
    }
    
    public VisionObject[] getObjects(double minimumConfidence) {

        if (foundObjects == null || foundObjects.length == 0)
            return null;

        List<VisionObject> filteredResult = Arrays.asList(foundObjects)
            .stream()
            .filter(vo -> vo.confidence >= minimumConfidence )
            .collect(Collectors.toList());

        VisionObject filteredArray[] = new VisionObject[filteredResult.size()];
        return filteredResult.toArray(filteredArray);

    }

    public VisionObject[] getObjectsOfType(String objectLabel) {

        if (foundObjects == null || foundObjects.length == 0)
            return null;
        // System.out.println("LINE 122!!!!!!!!!!!!!!");
        List<VisionObject> filteredResult = Arrays.asList(foundObjects)
            .stream()
            .filter(vo -> vo.objectLabel.equals(objectLabel) && vo.confidence > .40)
            .collect(Collectors.toList());

        VisionObject filteredArray[] = new VisionObject[filteredResult.size()];
        // System.out.println(filteredResult.size());
        return filteredResult.toArray(filteredArray);

    }
    public void saveVisionSnapshot(String fileName) 
    throws IOException {
        data();    
        Gson gson = new Gson();
        String str = gson.toJson(foundObjects);
        BufferedWriter writer = new BufferedWriter(new FileWriter(fileName));
        writer.write(str);
        
        writer.close();
    }

    public VisionObject[] loadVisionSnapshot(String fileName) 
    throws IOException {  
        Path filePath = Path.of(fileName);
        Gson gson = new Gson();

        String json = Files.readString(filePath);
        VisionObject[] snapShotObjects = gson.fromJson(json, VisionObject[].class);
        
        return snapShotObjects;
    }


    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

