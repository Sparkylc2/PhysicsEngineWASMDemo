public class Motor implements ForceRegistry {

    private final Rigidbody rigidbody;
    private PVector localAnchor;

    private float targetAngularVelocity;

    private boolean drawMotorForce;
    private boolean drawMotor;


    public Motor(Rigidbody rigidbody) {
        this.rigidbody = rigidbody;
        this.localAnchor = new PVector(this.rigidbody.getRadius(), 0);
        this.drawMotorForce = true;
    }

    public Motor(Rigidbody rigidbody, float targetAngularVelocity) {
        this.rigidbody = rigidbody;
        this.targetAngularVelocity = targetAngularVelocity;
        this.localAnchor = new PVector(this.rigidbody.getRadius(), 0);
        this.drawMotorForce = true;
    }


@Override
public PVector getForce(Rigidbody rigidbody, PVector position) {

    if(this.rigidbody != rigidbody) {
        throw new IllegalArgumentException("Rigidbody is not the same as the one this force is applied to");
    }

    this.rigidbody.setAngularVelocity(targetAngularVelocity);
    
    return new PVector(0, 0);
    
    
}

@Override
public void draw() {

    if(!drawMotorForce) {
        return;
    }
    if(PhysEngMath.Equals(this.targetAngularVelocity, 0)) {
        return;
    }
    PVector position = this.rigidbody.getPosition();
    boolean isClockwise = targetAngularVelocity > 0;

    float size = this.rigidbody.getRadius() * 0.5f;
    float arrowSize = size * 0.15f;
    float startAngle = 0;
    float endAngle = 3 * PI/2;// Change this to control the curvature of the arrow

    // Draw the curved part of the arrow
    pushMatrix();
    translate(position.x, position.y);
    rotate(this.rigidbody.getAngle() + PI/6);
    noFill();
    stroke(255, 0, 0); // Red color
    arc(0, 0, size, size, startAngle, endAngle);

    // Calculate the start and end of the arc
    float startX =  size * cos(startAngle)/2;
    float startY = size * sin(startAngle)/2;
    float endX = size * cos(endAngle)/2;
    float endY = size * sin(endAngle)/2;

    fill(255, 0, 0); // Red color
    noStroke();
    
    if(isClockwise) {
        triangle(endX, endY-arrowSize, endX, endY+arrowSize, endX+arrowSize*2, endY);
    } else {
        triangle(startX-arrowSize, startY, startX + arrowSize, startY, startX, startY - 2 * arrowSize);
    }

    popMatrix();

}
@Override
public PVector getApplicationPoint(Rigidbody rigidbody, PVector position) {
    return PhysEngMath.Transform(localAnchor, position, rigidbody.getAngle());
}


/*
====================================================================================================
======================================== Getters and Setters ========================================
====================================================================================================
*/
    public void setTargetAngularVelocity(float targetAngularVelocity) {
        this.targetAngularVelocity = targetAngularVelocity;
    }

    public void setLocalAnchor(PVector localAnchor) {
        this.localAnchor = localAnchor;
    } 
    public void setDrawMotor(boolean drawMotor) {
        this.drawMotor = drawMotor;
    }

    public boolean getDrawMotor() {
        return drawMotor;
    }


    public boolean getDrawMotorForce() {
        return drawMotorForce;
    }

    public void setDrawMotorForce(boolean drawMotorForce) {
        this.drawMotorForce = drawMotorForce;
    }

    public float getTargetAngularVelocity() {
        return targetAngularVelocity;
    }

    public PVector getLocalAnchor() {
        return localAnchor;
    }
    @Override
    public Rigidbody getRigidbodyA(){
        return this.rigidbody;
    }
    @Override
    public Rigidbody getRigidbodyB(){
        return this.rigidbody;
    }



}