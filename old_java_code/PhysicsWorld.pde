
/*
====================================================================================================
===================================  PHYSICS ENGINE OBJECTS  =======================================
====================================================================================================
*/
/*------------------------------------ NEVER CHANGE THIS -----------------------------------------*/

public static boolean IS_TEXTFIELD_ACTIVE = false;

public static boolean IS_PAUSED = false;
public static boolean IS_PAUSED_LOCK = false;

public static ArrayList<PVector> pointsOfContactList = new ArrayList<PVector>();

public static ArrayList<Rigidbody> rigidbodyList = new ArrayList<Rigidbody>();
public static ArrayList<Softbody> softbodyList = new ArrayList<Softbody>();

public ArrayList<ArrayList<Integer>> collisionPairs = new ArrayList<ArrayList<Integer>>();
    

public PVector[] contactList = new PVector[0];
public PVector[] impulseList = new PVector[2];
public PVector[] raList = new PVector[2]; 
public PVector[] rbList = new PVector[2];
public PVector[] frictionImpulseList = new PVector[2];
public float[] jList = new float[2];

/*
====================================================================================================
===================================  PHYSICS ENGINE CONSTANTS  =====================================
====================================================================================================
*/

public final float MIN_BODY_AREA = 0.001f * 0.001f; // m^2
public final float MAX_BODY_AREA = 300f * 300f; // m^2

public final float MIN_BODY_DENSITY = 0.01f; //g/cm^3
public final float MAX_BODY_DENSITY = 300.0f; //g/cm^3

public final float MIN_BODY_WIDTH = 0.01f; // m
public final float MAX_BODY_WIDTH = 300; // m

public final float MIN_BODY_HEIGHT = 0.01f; // m
public final float MAX_BODY_HEIGHT = 300.0f; // m

public final float MIN_BODY_RADIUS = 0.01f;
public final float MAX_BODY_RADIUS = 300.0f;

public final float MIN_MOUSE_VELOCITY_MAG = 0.0f;
public final float MAX_MOUSE_VELOCITY_MAG = 15.0f;

public final int MIN_ITERATIONS = 1;
public final int MAX_ITERATIONS = 1024;

public PVector BACKGROUND_COLOUR = new PVector(16, 18, 19);

public static int SUB_STEP_COUNT = 128;
public float SCROLL_SENSITIVITY = 1.1f;


public boolean DRAW_CONTACT_POINTS = false;
public boolean DRAW_AABBS = false;
public boolean DRAW_STATS = false;
public boolean TEXT_SMOOTHING = false;


public PVector GRAVITY_VECTOR = new PVector(0, 9.81f);
public float GRAVITY_MAG = 9.81f;

public float COEFFICIENT_OF_STATIC_FRICTION = 0.8f;
public float COEFFICIENT_OF_KINETIC_FRICTION = 0.3f;





public ArrayList<ForceRegistry> ALL_FORCES_ARRAYLIST = new ArrayList<ForceRegistry>();


public static float VERTEX_SNAP_RADIUS = 0.25f;



/*
====================================================================================================
===================================  PHYSICS ENGINE METHODS  =======================================
====================================================================================================
*/

//Iterations for substeps for each frame
public void Step(float dt, int totalIterations) {
    
    
    /*-----------------Related to Timekeeping Debugging -----------------*/
    FrameTimeUtility.startTotalWorldStepTime();
    /*-------------------------------------------------------------------*/
    
    
    totalIterations = PhysEngMath.Clamp(totalIterations, MIN_ITERATIONS, MAX_ITERATIONS);
    
    for (int currentIteration = 0; currentIteration < totalIterations; currentIteration++) {
        
        /*-----------------Related to Timekeeping Debugging -----------------*/
        FrameTimeUtility.startSubWorldStepTime();
        /*-------------------------------------------------------------------*/
        
        this.collisionPairs.clear();

        StepBodies(dt, totalIterations);
        BroadPhaseStep();
        NarrowPhaseStep();
        
        /*-----------------Related to Timekeeping Debugging -----------------*/
        FrameTimeUtility.updateSubWorldStepTime();
        /*-------------------------------------------------------------------*/
    }
    /*-----------------Related to Timekeeping Debugging -----------------*/
    FrameTimeUtility.updateTotalWorldStepTime();
    /*-------------------------------------------------------------------*/
}


/*
====================================================================================================
=================================== Collision Resolution Methods ===================================
====================================================================================================
*/

public void SeperateBodies(Rigidbody rigidbodyA, Rigidbody rigidbodyB, PVector minimumTranslationVector) {
    if(IS_PAUSED) {
        return;
    }

    if (rigidbodyA.getIsStatic() || rigidbodyA.getIsTranslationallyStatic()) {
        
        rigidbodyB.Move(minimumTranslationVector);
        
    } else if (rigidbodyB.getIsStatic() || rigidbodyB.getIsTranslationallyStatic()) {
        
        rigidbodyA.Move(PVector.mult(minimumTranslationVector, -1.0f));
        
    } else {
        
        rigidbodyA.Move(PVector.mult(minimumTranslationVector, -0.5f));
        rigidbodyB.Move(PVector.mult(minimumTranslationVector, 0.5f));
    }
}


public void ResolveCollisionLinear(CollisionManifold collisionManifold) {
    
    Rigidbody rigidbodyA = collisionManifold.getRigidbodyA();
    Rigidbody rigidbodyB = collisionManifold.getRigidbodyB();
    PVector normal = collisionManifold.getNormal();
    float depth = collisionManifold.getDepth();
    
    
    PVector velocityA = rigidbodyA.getVelocity().copy();
    PVector velocityB = rigidbodyB.getVelocity().copy();
    float restitution = min(rigidbodyA.getRestitution(), rigidbodyB.getRestitution());
    PVector relativeVelocity = PVector.sub(velocityB, velocityA);
    
    if (PVector.dot(relativeVelocity, normal) > 0.0f) {
        return;
    }
    
    float invMassA = rigidbodyA.getInvMass();
    float invMassB = rigidbodyB.getInvMass();
    
    float j = -(1f + restitution) * PVector.dot(relativeVelocity, normal) / (invMassA + invMassB);
    
    PVector impulse = PVector.mult(normal, j);
    
    
    velocityA = PVector.add(velocityA, PVector.mult(PVector.mult(impulse, -1), invMassA));
    velocityB = PVector.add(velocityB, PVector.mult(impulse, invMassB));
    
    rigidbodyA.setVelocity(velocityA);
    rigidbodyB.setVelocity(velocityB);
    
}

public void ResolveCollisionRotation(CollisionManifold contact) {
    
    Rigidbody rigidbodyA = contact.getRigidbodyA();
    Rigidbody rigidbodyB = contact.getRigidbodyB();
    float invMassA = rigidbodyA.getInvMass();
    float invMassB = rigidbodyB.getInvMass();
    float invRotationalInertiaA = rigidbodyA.getInvRotationalInertia();
    float invRotationalInertiaB = rigidbodyB.getInvRotationalInertia();
    PVector normal = contact.getNormal();
    PVector velocityA = rigidbodyA.getVelocity();
    PVector velocityB = rigidbodyB.getVelocity();
    float angularVelocityA = rigidbodyA.getAngularVelocity();
    float angularVelocityB = rigidbodyB.getAngularVelocity();

    int contactCount = contact.getContactCount();
    contactList = contact.getPointsOfContact();

    float e = min(rigidbodyA.getRestitution(), rigidbodyB.getRestitution());


    for(int i = 0; i < contactCount; i++) {
        this.impulseList[i] = new PVector();
        this.raList[i] = new PVector();
        this.rbList[i] = new PVector();
    }

    for (int i = 0; i < contactCount; i++) {
        PVector ra = PVector.sub(contactList[i], rigidbodyA.getPosition());
        PVector rb = PVector.sub(contactList[i], rigidbodyB.getPosition());

        raList[i] = ra;
        rbList[i] = rb;

        PVector raPerp = new PVector(-ra.y, ra.x);
        PVector rbPerp = new PVector(-rb.y, rb.x);

        PVector angularLinearVelocityA = PVector.mult(raPerp, angularVelocityA);
        PVector angularLinearVelocityB = PVector.mult(rbPerp, angularVelocityB);

        PVector relativeVelocity = PVector.sub(PVector.add(velocityB, angularLinearVelocityB),
                                               PVector.add(velocityA, angularLinearVelocityA));

        float contactVelocityMag = relativeVelocity.dot(normal);

        if (contactVelocityMag > 0f) {
            continue;
        }

        float raPerpDotN = raPerp.dot(normal);
        float rbPerpDotN = rbPerp.dot(normal);

        float denom = (invMassA + invMassB) +
            ((raPerpDotN * raPerpDotN) * invRotationalInertiaA) +
            ((rbPerpDotN * rbPerpDotN) * invRotationalInertiaB);

        float j = -(1f + e) * contactVelocityMag;
        j /= denom;
        j /= (float)contactCount;

        PVector impulse = PVector.mult(normal, j);
        impulseList[i] = impulse;
    }

    for(int i = 0; i < contactCount; i++) {
        PVector impulse = impulseList[i];
        PVector ra = raList[i];
        PVector rb = rbList[i];

        //float raCrossImpulse = ra.x * impulse.y - ra.y * impulse.x;
        //float rbCrossImpulse = rb.x * impulse.y - rb.y * impulse.x;

        velocityA.add(PVector.mult(impulse, -invMassA));
        velocityB.add(PVector.mult(impulse, invMassB));

        angularVelocityA += ra.cross(impulse).z * -1 * invRotationalInertiaA;
        angularVelocityB += rb.cross(impulse).z * invRotationalInertiaB;

        rigidbodyA.setVelocity(velocityA);
        rigidbodyB.setVelocity(velocityB);
        rigidbodyA.setAngularVelocity(angularVelocityA);
        rigidbodyB.setAngularVelocity(angularVelocityB);
    }
}


public void ResolveCollisionRotationAndFriction(CollisionManifold contact) {

    Rigidbody rigidbodyA = contact.getRigidbodyA();
    Rigidbody rigidbodyB = contact.getRigidbodyB();

    float invMassA = rigidbodyA.getInvMass();
    float invMassB = rigidbodyB.getInvMass();

    float invRotationalInertiaA = rigidbodyA.getInvRotationalInertia();
    float invRotationalInertiaB = rigidbodyB.getInvRotationalInertia();

    PVector positionA = rigidbodyA.getPosition();
    PVector positionB = rigidbodyB.getPosition();

    PVector velocityA = rigidbodyA.getVelocity();
    PVector velocityB = rigidbodyB.getVelocity();

    float angularVelocityA = rigidbodyA.getAngularVelocity();
    float angularVelocityB = rigidbodyB.getAngularVelocity();

    PVector normal = contact.getNormal();
    int contactCount = contact.getContactCount();
    contactList = contact.getPointsOfContact();



    float restitution = min(rigidbodyA.getRestitution(), rigidbodyB.getRestitution());

    float coefficientOfStaticFriction = (rigidbodyA.getCoefficientOfStaticFriction()
                                        + rigidbodyB.getCoefficientOfStaticFriction()) * 0.5f;
    float coefficientOfKineticFriction = (rigidbodyA.getCoefficientOfKineticFriction()
                                         + rigidbodyB.getCoefficientOfKineticFriction()) * 0.5f;
    
    for(int i = 0; i < contactCount; i++) {
        this.impulseList[i] = new PVector();
        this.raList[i] = new PVector();
        this.rbList[i] = new PVector();
        this.frictionImpulseList[i] = new PVector();
        this.jList[i] = 0f;
    }

    for(int i = 0; i < contactCount; i++) {

        PVector ra = PVector.sub(contactList[i], positionA);
        PVector rb = PVector.sub(contactList[i], positionB);

        raList[i] = ra;
        rbList[i] = rb;

        PVector raPerp = new PVector(-ra.y, ra.x);
        PVector rbPerp = new PVector(-rb.y, rb.x);

        PVector angularLinearVelocityA = PVector.mult(raPerp, angularVelocityA);
        PVector angularLinearVelocityB = PVector.mult(rbPerp, angularVelocityB);

        PVector relativeVelocity = PVector.sub(PVector.add(velocityB, angularLinearVelocityB),
                                               PVector.add(velocityA, angularLinearVelocityA));

        float contactVelocityMagnitude = relativeVelocity.dot(normal);
        
        if(contactVelocityMagnitude > 0f) {
            continue;
        }
        
        float raPerpendicularDotN = raPerp.dot(normal);
        float rbPerpendicularDotN = rbPerp.dot(normal);

        float denom = invMassA + invMassB
                    + (raPerpendicularDotN * raPerpendicularDotN) * invRotationalInertiaA
                    + (rbPerpendicularDotN * rbPerpendicularDotN) * invRotationalInertiaB;

        float j = -(1f + restitution) * contactVelocityMagnitude;
        j /= denom;
        j /= (float)contactCount;

        jList[i] = j;

        PVector impulse = PVector.mult(normal, j);
        impulseList[i] = impulse;
    }

    for(int i = 0; i < contactCount; i++) {

        PVector impulse = impulseList[i];
        PVector ra = raList[i];
        PVector rb = rbList[i];

        velocityA.add(PVector.mult(impulse, -invMassA));
        velocityB.add(PVector.mult(impulse, invMassB));

        angularVelocityA += -ra.cross(impulse).z * invRotationalInertiaA;
        angularVelocityB += rb.cross(impulse).z * invRotationalInertiaB;

    }

    for(int i = 0; i < contactCount; i++) {
        PVector ra = PVector.sub(contactList[i], positionA);
        PVector rb = PVector.sub(contactList[i], positionB);

        raList[i] = ra;
        rbList[i] = rb;

        PVector raPerp = new PVector(-ra.y, ra.x);
        PVector rbPerp = new PVector(-rb.y, rb.x);

        PVector angularLinearVelocityA = PVector.mult(raPerp, angularVelocityA);
        PVector angularLinearVelocityB = PVector.mult(rbPerp, angularVelocityB);

        PVector relativeVelocity = PVector.sub(PVector.add(velocityB, angularLinearVelocityB),
                                               PVector.add(velocityA, angularLinearVelocityA));
        PVector tangent = PVector.sub(relativeVelocity, PVector.mult(normal, relativeVelocity.dot(normal)));

        if(PhysEngMath.Equals(tangent, new PVector())) {
            continue;
        } else {
            tangent.normalize();
        }

        float raPerpDotT = raPerp.dot(tangent);
        float rbPerpDotT = rb.dot(tangent);

        float denom = invMassA + invMassB
                    + (raPerpDotT * raPerpDotT) * invRotationalInertiaA
                    + (rbPerpDotT * rbPerpDotT) * invRotationalInertiaB;

        float jt = -relativeVelocity.dot(tangent);
        jt /= denom;
        jt /= (float)contactCount;

        PVector frictionImpulse;
        float j = jList[i];

        if(abs(jt) <= j * coefficientOfStaticFriction) {
            frictionImpulse = PVector.mult(tangent, jt);
        } else {
            frictionImpulse = PVector.mult(tangent, -j * coefficientOfKineticFriction);
        }
        
        this.frictionImpulseList[i] = frictionImpulse;
    }
        for(int i = 0; i < contactCount; i++) {
            PVector frictionImpulse = frictionImpulseList[i];
            PVector ra = raList[i];
            PVector rb = rbList[i];

            velocityA.add(PVector.mult(frictionImpulse, -invMassA));
            velocityB.add(PVector.mult(frictionImpulse, invMassB));

            angularVelocityA += -ra.cross(frictionImpulse).z * invRotationalInertiaA;
            angularVelocityB += rb.cross(frictionImpulse).z * invRotationalInertiaB;

            rigidbodyA.setVelocity(velocityA);
            rigidbodyB.setVelocity(velocityB);

            rigidbodyA.setAngularVelocity(angularVelocityA);
            rigidbodyB.setAngularVelocity(angularVelocityB);
        }

    }

/*
====================================================================================================
============================ Broad & Narrow - Phase Collision Methods ==============================
====================================================================================================
*/

public void BroadPhaseStep() {
    for (int i = 0; i < rigidbodyList.size() - 1; i++) {
        
        Rigidbody rigidbodyA = rigidbodyList.get(i);
        ArrayList<Rigidbody> rigidbodyACollisionExclusionList = rigidbodyA.getCollisionExclusionList();
        AABB rigidbodyA_AABB = rigidbodyA.GetAABB();
        
        
        for (int j = i + 1; j < rigidbodyList.size(); j++) {
            Rigidbody rigidbodyB = rigidbodyList.get(j);
            AABB rigidbodyB_AABB = rigidbodyB.GetAABB();

            if(rigidbodyACollisionExclusionList.contains(rigidbodyB)) {
                continue;
            }

            if ((rigidbodyA.getIsStatic() && rigidbodyB.getIsStatic())) {
                continue;
            }
            

            //Remove if shit breaks
            if(!rigidbodyA.getCollidability() || !rigidbodyB.getCollidability()) {
                continue;
            }
            
            if (!Collisions.IntersectAABB(rigidbodyA_AABB, rigidbodyB_AABB)) {
                continue;
            }

            ArrayList<Integer> pair = new ArrayList<Integer>(Arrays.asList(i, j));
            collisionPairs.add(pair);
        }
    }
}

public void NarrowPhaseStep() {
    for (int i = 0; i < collisionPairs.size(); i++) {
        ArrayList<Integer> pair = collisionPairs.get(i);

        Rigidbody rigidbodyA = rigidbodyList.get(pair.get(0));
        Rigidbody rigidbodyB = rigidbodyList.get(pair.get(1));
        
        CollisionResult collisionResult = Collisions.Collide(rigidbodyA, rigidbodyB);
            
        if (collisionResult.getIsColliding()) {
            
            PVector minimumTranslationVector = PVector.mult(collisionResult.getNormal(), collisionResult.getDepth());
            
            SeperateBodies(rigidbodyA, rigidbodyB, minimumTranslationVector);
            Collisions.FindCollisionPoints(rigidbodyA, rigidbodyB, collisionResult);
            CollisionManifold collisionManifold = new CollisionManifold(rigidbodyA, rigidbodyB, collisionResult);
            this.ResolveCollisionRotationAndFriction(collisionManifold);

            for(PVector contact : collisionResult.getPointsOfContact()) {
                pointsOfContactList.add(contact);
            }
        }
    }
}
    

public void StepBodies(float dt, int totalIterations) {
    if(IS_PAUSED) {
        return;
    }
    
    for(Rigidbody rigidbody : rigidbodyList) {
        if(rigidbody.getIsStatic() ) {
            continue;
        }

        rigidbody.update(dt, totalIterations);
    }
}
            
/*
==================================================================================================
======================================== Helper Methods  =========================================
==================================================================================================
*/
public void AddBodyToBodyEntityList(Rigidbody body) {
    rigidbodyList.add(body);
}

public void RemoveBodyFromBodyEntityList(Rigidbody body) {
     rigidbodyList.remove(body);
}
        
public void RemoveBodyFromBodyEntityList(int index) {
   
   if(index < 0 || index >= rigidbodyList.size()) {
        throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + rigidbodyList.size());
    }

    rigidbodyList.remove(index);
}

public Rigidbody GetBodyFromBodyEntityList(int index) {

    if(index < 0 || index >= rigidbodyList.size()) {
        throw new IndexOutOfBoundsException("Index: " + index + ", Size: " + rigidbodyList.size());
    }

    return rigidbodyList.get(index);
}
                        
public void ClearBodyEntityList() {
    rigidbodyList.clear();
}



    
                                        
                                        