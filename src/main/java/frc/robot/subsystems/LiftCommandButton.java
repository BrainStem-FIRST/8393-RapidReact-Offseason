package frc.robot.subsystems;
import frc.robot.subsystems.LiftSubsystem;

public class LiftCommandButton {
    
   HangingSteps currentStep; 
   private final LiftSubsystem liftSubsystem = new LiftSubsystem();


    public LiftCommandButton(HangingSteps hs){
        currentStep = hs;
    }

    public void buttonHit() {
        if(currentStep == HangingSteps.STEP1) {
            currentStep = HangingSteps.STEP2;
     } else if (currentStep == HangingSteps.STEP2){
            currentStep = HangingSteps.STEP3;
      } else if (currentStep == HangingSteps.STEP3){
            currentStep = HangingSteps.STEP1;
      }

      liftSubsystem.hitButton = true;
        
    }

    public HangingSteps getState(){
        return currentStep;
    }
    
}
