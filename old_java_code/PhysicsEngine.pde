// Softbody softbody = new Softbody(new PVector(10, -20), 0, 2, 2);
// Softbody softbody2 = new Softbody(new PVector(-10, -20), 0, 2, 2);

public void setup() {
    if(System.getProperty("os.name").toLowerCase().contains("mac")) {
        windowMove(0, 22);
    }
    UI_QualitySettings qualitySettings = new UI_QualitySettings();
    if(qualitySettings.settings.getString("TextQuality").equals("High")){
        hint(ENABLE_STROKE_PURE);
    } else {
        hint(DISABLE_STROKE_PURE);
    }

    textFont(createFont(sketchPath() + "/data/fonts/InterDisplay-SemiBold.ttf", 128, true), 10);
    dash = new DashedLines(this);
    dash.pattern(1, 0.5);
/*--------------------- Timing Utilities ---------------------*/ 
    FrameTimeUtility.init();

    //MAYBE DO SOMETHING TO CHECK WHICH FRAME RATE TO USE
    // frameRate(100);
/*--------------------- Camera Utilities ---------------------*/
    Camera = new Camera();
    playTimeTracker = new UI_QualitySettings();
    playTimeTracker.startPlaytimeTracking();
/*---------------------------- UI ----------------------------*/
    UI_Manager.init();
/*-------------------------- Rigidbodies ------------------------*/
    rigidbodyList = new ArrayList<Rigidbody>();
/*------------------------------------------------------------*/

        // Rigidbody springBody = RigidbodyGenerator.CreateBoxBody(4f, 1f, 1f, 0.5f, false, true, 0.05f, new PVector(0, 0, 0), new PVector(255, 255, 255));
    // springBody.setVelocity(new PVector(0,20));
    // springBody.SetInitialPosition(new PVector(-10, -5));

    // Spring springLeft = new Spring(springBody, new PVector(2,0), new PVector(-8, -10));
    // Spring springRight = new Spring(springBody, new PVector(-2,0), new PVector(-12, -10));

    // springBody.addForceToForceRegistry(springLeft);
    // springBody.addForceToForceRegistry(springRight);

    // springBody.addForceToForceRegistry(new Gravity(springBody));

    // AddBodyToBodyEntityList(springBody);






    Rigidbody floor = RigidbodyGenerator.CreateBoxBody(1000f, 5f, 1f, 0.5f, true, true, 0.05f, new PVector(0,0,0), new PVector(255,255,255));

    floor.SetInitialPosition(new PVector(0, 10));
   
    AddBodyToBodyEntityList(floor);

    // Rigidbody circle = RigidbodyGenerator.CreateCircleBody(1f, 1f, 0.5f, false, true, 0.05f, new PVector(0,0,0), new PVector(255,255,255));
    // circle.SetInitialPosition(new PVector(18, 0));
    // circle.setVelocity(new PVector(0, 60));


    // Spring spring1 = new Spring(circle, new PVector(0, 0), new PVector(0, -20));
    // spring1.setPerfectSpring(true);
    // spring1.setSpringConstant(10);
    // spring1.setEquilibriumLength(0);
    // Spring spring2 = new Spring(circle, new PVector(0, 0), new PVector(0, 20));
    // spring2.setPerfectSpring(true);
    // spring2.setSpringConstant(10);
    // spring2.setEquilibriumLength(0);
    // Spring spring3 = new Spring(circle, new PVector(0, 0), new PVector(-20, 0));
    // spring3.setPerfectSpring(true);
    // spring3.setSpringConstant(10);
    // spring3.setEquilibriumLength(0);
    // Spring spring4 = new Spring(circle, new PVector(0, 0), new PVector(20, 0));
    // spring4.setPerfectSpring(true);
    // spring4.setSpringConstant(10);
    // spring4.setEquilibriumLength(0);

    // circle.addForceToForceRegistry(spring1);
    // circle.addForceToForceRegistry(spring2);
    // circle.addForceToForceRegistry(spring3);
    // circle.addForceToForceRegistry(spring4);



    // AddBodyToBodyEntityList(circle);



}



public void draw() {
    FrameTimeUtility.calculateFrameTime();
        Camera.onFrameStart();
            /*--------------------- Main Methods ---------------------*/
            UI_Manager.interactionDraw();
            
            Step(FrameTimeUtility.DT, SUB_STEP_COUNT);
            /*--------------------------------------------------------*/
    
        Camera.onFrameEnd();
        UI_Manager.draw();
    
    FrameTimeUtility.displayTimings();
    FrameTimeUtility.updateFrameTime();
}


