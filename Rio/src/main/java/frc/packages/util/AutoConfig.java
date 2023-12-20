package frc.packages.util;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.HashMap;

import org.json.JSONObject;
import org.json.JSONTokener;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;

public class AutoConfig {
    private HashMap<String, PIDController> PIDControllers = new HashMap<String, PIDController>();
    private HashMap<String, SparkMaxPIDController> sparkPIDControllers = new HashMap<String, SparkMaxPIDController>();

    private HashMap<String, JSONObject> pidFiles = new HashMap<String, JSONObject>();

    /**
     * Reloads all values from the files and updates the configurable parameters
     */
    public void reload(){
        reloadFiles();
        setWPIpidControllers();
    }

    private void setWPIpidControllers(){
        for(String key : PIDControllers.keySet()){
            try {
                String[] split = key.split(";");
                PIDController controller = PIDControllers.get(key);
                JSONObject pid = pidFiles.get(split[0]);
                for(String nextKey : split[1].split(".")){
                    pid = pid.getJSONObject(nextKey);
                }
                controller.setP(pid.getDouble("p"));
                controller.setI(pid.getDouble("i"));
                controller.setD(pid.getDouble("d"));
            } catch(Exception e){
                e.printStackTrace();
            }
        }
        
    }

    private void reloadFiles(){
        for(String file : pidFiles.keySet()){
            try {
                FileInputStream inputPid = new FileInputStream(Filesystem.getDeployDirectory() + "/" + file);
                JSONTokener pidTokener = new JSONTokener(inputPid);
                pidFiles.put(file, new JSONObject(pidTokener));
            } catch (FileNotFoundException e) {
                System.out.println("JSON Config file not found: " + file);
                System.exit(1);
            }
        }
    }


    private void addFile(String file){
        if(!pidFiles.containsKey(file)){
            pidFiles.put(file, new JSONObject());
            reloadFiles();
        }
    }
    /**
     * Sets up a WPI PID controller to be automatically configured
     * @param controller the controller to configure
     * @param file the json file the PID Info will be found in, relative to the deploy folder (includes extension)
     * @param key the JSON key the PID Info will be found under (can be nested with .)
     */
    public void addPIDController(PIDController controller, String file, String key){
        addFile(file);
        PIDControllers.put(file + ";" + key, controller);
    }
}
