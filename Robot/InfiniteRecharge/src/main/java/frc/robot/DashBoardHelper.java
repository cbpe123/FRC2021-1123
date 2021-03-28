package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



/**
 * This is a class that will slow down how fast values are uploaded to shuffleboard.
 * This is to help bandwidth. Input the name of the value you want, the value itself, 
 * then often you want it posted in seconds. Call this periodically
 */

public final class DashBoardHelper extends SubsystemBase{
    int ticker = 100;
    String value = "";
    double time = 0.0;
    String name = "";
    String previousValue = value;
    int numberOfTimesCalled = 0;  //This is a test variable

   public void updateValue(String inputName, double inputValue, double inputTime){
    name = inputName;
    value = String.valueOf(inputValue);
    time = inputTime;
    // ticker++;
    // if(ticker >= time/0.02){
    //     ticker = 0;
    //     SmartDashboard.putNumber(name, value);
    //     // numberOfTimesCalled++;
    //     // SmartDashboard.putNumber("number of times called", numberOfTimesCalled);
    // }
   }

   public void updateValue(String inputName, int inputValue, double inputTime){
    name = inputName;
    value = String.valueOf(inputValue);
    time = inputTime;
   }
   
   public void updateValue(String inputName, String inputValue, double inputTime){
    name = inputName;
    value = inputValue;
    time = inputTime;
    // ticker++;
    // if(ticker >= time/0.02){
    //     ticker = 0;
    //     SmartDashboard.putString(name, value);
    // }
   }
   @Override
   public void periodic(){
    ticker++;
    if(hasChanged(value) == true && ticker > time/0.02){
        ticker = 0;
        SmartDashboard.putString(name, value);
        numberOfTimesCalled++;
        SmartDashboard.putNumber("Number of times called", numberOfTimesCalled);
    }
    previousValue = value;
   }

   private boolean hasChanged(String inputValue){
    if(inputValue == previousValue){
        return false;
    }
    return true;
   }
}
