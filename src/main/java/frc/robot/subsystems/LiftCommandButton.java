package frc.robot.subsystems;




public class LiftCommandButton {
    
   HangingSteps currentStep; 


    public LiftCommandButton(HangingSteps hs){
        currentStep = hs;
    }

    public void buttonHit() {
        if(currentStep == HangingSteps.STEP1) 
            currentStep = HangingSteps.STEP2;
        else if (currentStep == HangingSteps.STEP2)
            currentStep = HangingSteps.STEP3;
        else if (currentStep == HangingSteps.STEP3)
            currentStep = HangingSteps.STEP1;
    }

    public HangingSteps getState(){
        return currentStep;
    }
    
}
