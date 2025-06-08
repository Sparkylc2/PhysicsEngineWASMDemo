public class Rod implements ForceRegistry {
/*-------------------------------------------------------------------------------------------------*/
    private float length = 0f;
    private float stiffness = 250000.0f;
    private float damping = 1.0f;

    private PVector localAnchorA = new PVector();
    private PVector localAnchorB = new PVector();
    private PVector anchorPoint = new PVector();

    private boolean isTwoBodyRod;
    private boolean isJoint;

    private Rigidbody rigidbodyA;
    private Rigidbody rigidbodyB;


/*-------------------------------------------------------------------------------------------------*/
    private PVector worldAnchorA = new PVector();
    private PVector worldAnchorB = new PVector();

    private PVector drawWorldAnchorA = new PVector();
    private PVector drawWorldAnchorB = new PVector();

    private PVector relativeVelocity = new PVector();

    private PVector velocityA = new PVector();
    private PVector velocityB = new PVector();

    private PVector dampingForce = new PVector();
    private PVector direction = new PVector();
    private PVector rodForce = new PVector();
    private PVector force = new PVector();
    private PVector rigidbodyOrientation = new PVector();

    private float displacement;
    private float dot;
    private float currentRigidbodyAngle;

/*-------------------------------------------------------------------------------------------------*/
//Reusable stuff


    public Rod(Rigidbody rigidbodyA, PVector localAnchorA, PVector anchorPoint) {

        this.rigidbodyA = rigidbodyA;

        this.anchorPoint.set(anchorPoint);
        this.localAnchorA.set(localAnchorA);

        this.isTwoBodyRod = false;

        if(rigidbodyA == null) {
            throw new NullPointerException("Rigidbody A is null");
        }
        
        this.length = PVector.sub(PhysEngMath.Transform(this.localAnchorA, this.rigidbodyA.getPosition(), this.rigidbodyA.getAngle()), 
                                  this.anchorPoint)
                                  .mag();
    }

    public Rod(Rigidbody rigidbodyA, Rigidbody rigidbodyB, PVector localAnchorA, PVector localAnchorB) {

        this.rigidbodyA = rigidbodyA;
        this.rigidbodyB = rigidbodyB;

        this.localAnchorA.set(localAnchorA);
        this.localAnchorB.set(localAnchorB);

        this.isTwoBodyRod = true;

        if(rigidbodyA == null) {
            throw new NullPointerException("Rigidbody A is null");
        } else if(rigidbodyB == null) {
            throw new NullPointerException("Rigidbody B is null");
        }

        this.length = PVector.sub(PhysEngMath.Transform(this.localAnchorB, this.rigidbodyB.getPosition(), this.rigidbodyB.getAngle()), 
                                  PhysEngMath.Transform(this.localAnchorA, this.rigidbodyA.getPosition(), this.rigidbodyA.getAngle()))
                                  .mag();
    }


@Override
public PVector getForce(Rigidbody rigidbody, PVector position) {

    this.force.set(0,0,0);

    if(isTwoBodyRod) {
        if(rigidbody == rigidbodyA) {
                this.worldAnchorA.set(PhysEngMath.Transform(this.localAnchorA, position, this.rigidbodyA.getAngle()));
                this.worldAnchorB.set(PhysEngMath.Transform(this.localAnchorB, this.rigidbodyB.getPosition(), this.rigidbodyB.getAngle()));

                this.velocityA.set(rigidbodyA.getVelocity());
                this.velocityB.set(rigidbodyB.getVelocity());

                this.direction.set(this.worldAnchorB.sub(this.worldAnchorA));
                
                this.displacement = direction.mag();
                this.direction.normalize();

                this.relativeVelocity.set(PVector.sub(this.velocityB, this.velocityA));

                this.dot = PVector.dot(relativeVelocity, this.direction);
                this.dampingForce.set(PVector.mult(this.direction, damping * dot));

                this.force.add(this.direction.mult(this.stiffness * (displacement - this.length)));
                this.force.add(this.dampingForce);
                
                return this.force;

            } else {
                this.worldAnchorA.set(PhysEngMath.Transform(this.localAnchorA, this.rigidbodyA.getPosition(), this.rigidbodyA.getAngle()));
                this.worldAnchorB.set(PhysEngMath.Transform(this.localAnchorB, position, this.rigidbodyB.getAngle()));

                this.velocityA.set(rigidbodyA.getVelocity());
                this.velocityB.set(rigidbodyB.getVelocity());


                this.direction.set(this.worldAnchorB.sub(this.worldAnchorA));
                this.displacement = direction.mag();
                this.direction.normalize();


                this.relativeVelocity.set(PVector.sub(this.velocityA, this.velocityB));
                this.dot = PVector.dot(relativeVelocity, this.direction);
                

                this.dampingForce.set(PVector.mult(direction, -damping * dot * relativeVelocity.mag() / 2));

                this.force.add(this.dampingForce);
                this.force.add(this.direction.mult(this.stiffness * (displacement - this.length)));

                return this.force.mult(-1);
            } 

    } else {

        this.worldAnchorA.set(PhysEngMath.Transform(this.localAnchorA, position, this.rigidbodyA.getAngle()));
        this.worldAnchorB.set(this.anchorPoint);

        this.velocityA.set(rigidbodyA.getVelocity());
        this.velocityB.set(0,0);    

        this.direction.set(this.worldAnchorB.sub(this.worldAnchorA));
        this.displacement = direction.mag();
        this.direction.normalize();

        this.relativeVelocity.set(this.velocityB.sub(this.velocityA));
        this.dot = PVector.dot(relativeVelocity, this.direction);

        this.dampingForce.set(this.direction.copy().mult(-damping * dot * relativeVelocity.mag() / 2));

        this.force.add(this.direction.mult(this.stiffness * (displacement - this.length)));
        this.force.add(this.dampingForce);
        
        return this.force;
                
    }
}



@Override
public void draw() {
    if(isTwoBodyRod){

        this.drawWorldAnchorA.set(PhysEngMath.Transform(this.localAnchorA, this.rigidbodyA.getPosition(), this.rigidbodyA.getAngle()));
        this.drawWorldAnchorB.set(PhysEngMath.Transform(this.localAnchorB, this.rigidbodyB.getPosition(), this.rigidbodyB.getAngle()));

        strokeWeight(0.15);
        stroke(0);
        line(this.drawWorldAnchorA.x, this.drawWorldAnchorA.y, this.drawWorldAnchorB.x, this.drawWorldAnchorB.y);
        strokeWeight(0.1);
        stroke(255);
        line(this.drawWorldAnchorA.x, this.drawWorldAnchorA.y, this.drawWorldAnchorB.x, this.drawWorldAnchorB.y);
    } else {

        this.drawWorldAnchorA.set(PhysEngMath.Transform(this.localAnchorA, this.rigidbodyA.getPosition(), this.rigidbodyA.getAngle()));

        strokeWeight(0.15);
        stroke(0);
        line(this.drawWorldAnchorA.x, this.drawWorldAnchorA.y, this.anchorPoint.x, this.anchorPoint.y);
        strokeWeight(0.1);
        stroke(255);
        line(this.drawWorldAnchorA.x, this.drawWorldAnchorA.y, this.anchorPoint.x, this.anchorPoint.y);

    }
}  



@Override
public PVector getApplicationPoint(Rigidbody rigidbody, PVector position) {
        if(rigidbody == rigidbodyA) {
            return PhysEngMath.Transform(localAnchorA, rigidbodyA.getPosition(), rigidbody.getAngle());
        } else {
            return PhysEngMath.Transform(localAnchorB, rigidbodyB.getPosition(), rigidbody.getAngle());
        }
}


/*
====================================================================================================
===================================GETTERS AND SETTERS==============================================
====================================================================================================
*/
public void setLength(float length) {
    this.length = length;
  }

public void setAnchorPoint(PVector anchorPoint) {
    this.anchorPoint.set(anchorPoint);
  }

public void setLocalAnchorA(PVector localAnchorA) {
    this.localAnchorA.set(localAnchorA);
  }

public void setLocalAnchorB(PVector localAnchorB) {
    this.localAnchorB.set(localAnchorB);
  }

public void setStiffness(float stiffness) {
    this.stiffness = stiffness;
  }

public void setDamping(float damping) {
    this.damping = damping;
  }

public void setTwoBodyRod(boolean isTwoBodyRod) {
    this.isTwoBodyRod = isTwoBodyRod;
  }

public void setIsJoint(boolean isJoint) {
    this.isJoint = isJoint;

    if(this.isJoint) {
        if(this.isTwoBodyRod){
            this.length = 0f;
            rigidbodyA.addBodyToCollisionExclusionList(rigidbodyB);
            rigidbodyB.addBodyToCollisionExclusionList(rigidbodyA);
        } else {
            this.length = 0f;
        }
    }
}   



public float getLength() {
    return length;
  }

public PVector getAnchorPoint() {
    return anchorPoint;
  }

public PVector getLocalAnchorA() {
    return localAnchorA;
  }

public PVector getLocalAnchorB() {
    return localAnchorB;
  }

public float getStiffness() {
    return stiffness;
  }

public float getDamping() {
    return damping;
  }

public boolean getTwoBodyRod() {
    return isTwoBodyRod;
  }

public boolean getIsJoint() {
    return this.isJoint;
}
@Override
public Rigidbody getRigidbodyA() {
    return rigidbodyA;
  }
@Override
public Rigidbody getRigidbodyB() {
    if(this.isTwoBodyRod) {
        return this.rigidbodyB;
    }
    return this.rigidbodyA;
  }



}



                /*
                if(!this.isHingeable) {
                    float rigidbodyAngle = rigidbodyB.getAngle();
                    this.rigidbodyOrientation.set(cos(rigidbodyAngle), sin(rigidbodyAngle));

                    float angleBetween = PVector.angleBetween(this.direction, this.rigidbodyOrientation);

                    if (this.direction.copy().cross(this.rigidbodyOrientation).z < 0) {
                        angleBetween = -angleBetween;
                    }

                    float angleDifference = angleBetween - this.initialAngleB;
                    //this.netTorque = -angleStiffness * angleDifference - angleDamping * rigidbody.getAngularVelocity();
                    //rigidbodyB.setAngle(rigidbodyB.getAngle()-angleDifference);
                    //force.add(0, 0, correctiveTorque);
                }
                */
  
