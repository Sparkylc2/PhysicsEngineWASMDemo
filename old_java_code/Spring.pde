public class Spring implements ForceRegistry {

    private Rigidbody rigidbodyA;
    private Rigidbody rigidbodyB;

    private PVector anchorPoint = new PVector();
    private PVector localAnchorA = new PVector();
    private PVector localAnchorB = new PVector();
    
    private boolean drawSpring = true;

    //Some default values
    private boolean lockTranslationToXAxis = false;
    private boolean lockTranslationToYAxis = false;
    
    private boolean isPerfectSpring = false;

    private float equilibriumLength = 1f;
    private float springConstant = 50;
    private float damping = 0.5f;
    
    private float springLength;

    private boolean isTwoBodySpring;


    /*--------------- Reusable --------------- */
    private PVector worldAnchorA = new PVector();
    private PVector worldAnchorB = new PVector();

    private PVector velocityA = new PVector();
    private PVector velocityB = new PVector();

    private PVector direction = new PVector();


    public Spring () {
        this.rigidbodyA = null;
        this.rigidbodyB = null;
    }


    public Spring(Rigidbody rigidbody, PVector localAnchorA, PVector anchorPoint) {

        this.rigidbodyA = rigidbody;
        this.localAnchorA.set(localAnchorA);
        this.anchorPoint.set(anchorPoint);

        this.springLength = PhysEngMath.Transform(localAnchorA, rigidbodyA.getPosition(), rigidbodyA.getAngle()).sub(anchorPoint).mag();

        this.isTwoBodySpring = false;

    }

    public Spring(Rigidbody rigidbodyA, Rigidbody rigidbodyB, PVector localAnchorA, PVector localAnchorB) {
        
        this.rigidbodyA = rigidbodyA;
        this.rigidbodyB = rigidbodyB;
        
        this.localAnchorA.set(localAnchorA);
        this.localAnchorB.set(localAnchorB);
        
        this.springLength = PhysEngMath.Transform(localAnchorB, rigidbodyB.getPosition(), rigidbodyB.getAngle()).sub(PhysEngMath.Transform(localAnchorA, rigidbodyA.getPosition(), rigidbodyA.getAngle())).mag();

        this.isTwoBodySpring = true;

    }

    @Override
    public PVector getForce(Rigidbody rigidbody, PVector position) {
        float totalForceMagnitude = 0f;
        float displacement = 0f;

        if(isTwoBodySpring) {
                this.velocityA.set(rigidbodyA.getVelocity());
                this.velocityB.set(rigidbodyB.getVelocity());

                this.worldAnchorA.set(PhysEngMath.Transform(this.localAnchorA, (rigidbody == rigidbodyA) ? position : rigidbodyA.getPosition(), rigidbodyA.getAngle()));
                this.worldAnchorB.set(PhysEngMath.Transform(this.localAnchorB,(rigidbody == rigidbodyB) ? position : rigidbodyB.getPosition(), rigidbodyB.getAngle()));

                if(lockTranslationToYAxis) {
                    if(rigidbody == rigidbodyA) {
                        this.rigidbodyA.setVelocity(velocityA.set(0, velocityA.y));
                    } else {
                        this.rigidbodyB.setVelocity(velocityB.set(0, velocityB.y));
                    }
                } else if(lockTranslationToXAxis) {
                    if(rigidbody == rigidbodyA) {
                       this. rigidbodyA.setVelocity(velocityA.set(velocityA.x, 0));
                    } else {
                        this.rigidbodyB.setVelocity(velocityB.set(velocityB.x, 0));
                    }
                }

                this.direction.set(worldAnchorB.sub(worldAnchorA));
                displacement = this.direction.mag();
                direction.normalize();


                totalForceMagnitude = (displacement - this.springLength * this.equilibriumLength) * this.springConstant;

                if(!isPerfectSpring){                                                                                    
                    totalForceMagnitude += (direction.dot(velocityB.sub(velocityA)) * this.damping);
                }
                
                return this.direction.mult((rigidbody == rigidbodyA) ? totalForceMagnitude : -totalForceMagnitude);
        } else {
            worldAnchorA.set(PhysEngMath.Transform(this.localAnchorA, position, rigidbodyA.getAngle()));
            worldAnchorB.set(anchorPoint);
    
            velocityA.set(rigidbodyA.getVelocity());
            velocityB.set(0,0);

            this.direction.set(worldAnchorB.sub(worldAnchorA));
            displacement = direction.mag();
            direction.normalize();

            if(lockTranslationToYAxis) rigidbodyA.setVelocity(velocityA.set(0, velocityA.y));
            else if(lockTranslationToXAxis) rigidbodyA.setVelocity(velocityA.set(velocityA.x, 0));
            
            if(!isPerfectSpring){                                                                                    
                totalForceMagnitude = (displacement - this.springLength * this.equilibriumLength) * this.springConstant;
                totalForceMagnitude += (direction.dot(velocityB.sub(velocityA)) * this.damping);
            } else {
                totalForceMagnitude = (displacement - this.springLength * this.equilibriumLength) * this.springConstant;
            }

            return this.direction.mult(totalForceMagnitude);
        }
    } 

    
           

public void draw() {
    if(this.drawSpring) {
        PVector worldAnchorA;
        PVector worldAnchorB;
        PVector direction;
        float length;

        if(isTwoBodySpring) {
            worldAnchorA = PhysEngMath.Transform(localAnchorA, rigidbodyA.getPosition(), rigidbodyA.getAngle());
            worldAnchorB = PhysEngMath.Transform(localAnchorB, rigidbodyB.getPosition(), rigidbodyB.getAngle());
        } else {
            worldAnchorA = PhysEngMath.Transform(localAnchorA, rigidbodyA.getPosition(), rigidbodyA.getAngle());
            worldAnchorB = this.anchorPoint;
        }

        direction = PVector.sub(worldAnchorA, worldAnchorB);
        length = direction.mag();
        direction.normalize();

        fill(255);

        float segments = 5;
        float segmentLength = length / segments;

        float offsetMagnitude = 0.5;


        strokeWeight(0.3);
        stroke(0); 
        line(worldAnchorA.x, worldAnchorA.y, worldAnchorB.x, worldAnchorB.y);
        stroke(255); 
        strokeWeight(0.1);
        line(worldAnchorA.x, worldAnchorA.y, worldAnchorB.x, worldAnchorB.y);
        
        PVector segmentStart = new PVector();
        PVector segmentEnd = new PVector();
        PVector midPoint = new PVector();
        PVector offset = new PVector();
        PVector directionSegmentLength = PVector.mult(direction, segmentLength);

        for(int i = 0; i < segments; i++) {

            segmentStart.set(PVector.add(worldAnchorB, PVector.mult(directionSegmentLength, i)));
            segmentEnd.set(PVector.add(worldAnchorB, PVector.mult(directionSegmentLength, i + 1)));


            midPoint.set(PVector.lerp(segmentStart, segmentEnd, 0.5f));


            if(i % 2 == 0) {
                offset.set(new PVector(-direction.y, direction.x).mult(offsetMagnitude));
            } else {
                offset.set(new PVector(direction.y, -direction.x).mult(offsetMagnitude));
            }

            PVector midPointOffset = PVector.add(midPoint, offset);

            strokeWeight(0.2);
            stroke(0);
            line(segmentStart.x, segmentStart.y, midPointOffset.x, midPointOffset.y);
            line(midPointOffset.x, midPointOffset.y, segmentEnd.x, segmentEnd.y);
            strokeWeight(0.1);
            stroke(255);
            line(segmentStart.x, segmentStart.y, midPointOffset.x, midPointOffset.y);
            line(midPointOffset.x, midPointOffset.y, segmentEnd.x, segmentEnd.y);
        }
    } else {
        return;
    }
}
    

    @Override
    public PVector getApplicationPoint(Rigidbody rigidbody, PVector position) {
            if(rigidbody == rigidbodyA) {
                return PhysEngMath.Transform(localAnchorA, position, rigidbodyA.getAngle());
            } else {
                return PhysEngMath.Transform(localAnchorB, position, rigidbodyB.getAngle());
            }
    }

/*
====================================================================================================
================================== Getters & Setters ===============================================
====================================================================================================
*/
    public void setRigidbodyA(Rigidbody rigidbody){
        this.rigidbodyA = rigidbody;
    }
    public void setSpringConstant(float springConstant) {
        this.springConstant = springConstant;
    }

    public void setSpringLength(float springLength) {
        this.springLength = springLength;
    }

    public void setEquilibriumLength(float equilibriumLength) {
        this.equilibriumLength = equilibriumLength;
    }

    public void setDamping(float damping) {
        this.damping = damping;
    }

    public void setLockTranslationToXAxis(boolean lockTranslationToXAxis) {
        this.lockTranslationToXAxis = lockTranslationToXAxis;
    }

    public void setLockTranslationToYAxis(boolean lockTranslationToYAxis) {
        this.lockTranslationToYAxis = lockTranslationToYAxis;
    }

    public void setPerfectSpring(boolean isPerfectSpring) {
        this.isPerfectSpring = isPerfectSpring;
    }
    public void setDrawSpring(boolean drawSpring) {
        this.drawSpring = drawSpring;
    }
    public void setAnchorPoint(PVector anchorPoint) {
        this.anchorPoint.set(anchorPoint);
    }

    public void setAnchorPoint(float x, float y) {
        this.anchorPoint.set(x, y);
    }

    public void setLocalAnchorA(PVector localAnchorA) {
        this.localAnchorA.set(localAnchorA);
    }

    public void setLocalAnchorA(float x, float y) {
        this.localAnchorA.set(x, y);
    }

    public void setLocalAnchorB(PVector localAnchorB) {
        this.localAnchorB.set(localAnchorB);
    }

    public void setLocalAnchorB(float x, float y) {
        this.localAnchorB.set(x, y);
    }

    public float getSpringConstant() {
        return this.springConstant;
    }

    public float getSpringLength() {
        return this.springLength;
    }

    public float getEquilibriumLength() {
        return this.equilibriumLength;
    }

    public float getDamping() {
        return this.damping;
    }

    public boolean getLockTranslationToXAxis() {
        return this.lockTranslationToXAxis;
    }

    public boolean getLockTranslationToYAxis() {
        return this.lockTranslationToYAxis;
    }

    public boolean getPerfectSpring() {
        return this.isPerfectSpring;
    }

    public PVector getAnchorPoint() {
        return this.anchorPoint;
    }

    public PVector getLocalAnchorA() {
        return this.localAnchorA;
    }

    public PVector getLocalAnchorB() {
        return this.localAnchorB;
    }

    public boolean getDrawSpring() {
        return this.drawSpring;
    }
    @Override
    public Rigidbody getRigidbodyA() {
        return this.rigidbodyA;
    }
    @Override
    public Rigidbody getRigidbodyB() {
        if(isTwoBodySpring) {
            return this.rigidbodyB;
        }
        return this.rigidbodyA;
    }

    public boolean getIsTwoBodySpring() {
        return this.isTwoBodySpring;
    }
}


