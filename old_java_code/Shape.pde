public class Shape {


  private int opacity = 166;


  private PVector fill = new PVector();
  private PVector stroke = new PVector();

  public Shape() {
  }
  
  public void draw() {
    
    background(16, 18, 19);
    drawRigidbodies();
  /*---------------------------------Collision Point Debugging--------------------------------------*/
    if(DRAW_AABBS) {
      drawAABB();
    }
    
    if(DRAW_CONTACT_POINTS) {
      drawCollisionPoints();
    }
  /*-----------------------------------------------------------------------------------------------*/
    drawForces();
  }

/*
====================================================================================================
====================================== Drawing Methods =============================================
====================================================================================================
*/
  public void drawRigidbodies() {

    for(int body = 0; body < rigidbodyList.size(); body++) {

      Rigidbody rigidbody = rigidbodyList.get(body);
        if(rigidbody.getIsVisible()) {
          if (rigidbody.getShapeType() == ShapeType.CIRCLE) {
            drawCircle(rigidbody.getPosition(), rigidbody.getRadius(),
                      rigidbody.getAngle(), rigidbody.getStrokeWeight(), rigidbody.getFillColour(),
                       rigidbody.getStrokeColour(), false);
          }

          if (rigidbody.getShapeType() == ShapeType.BOX) {
            
              drawPolygon(rigidbody.getPosition(), rigidbody.GetTransformedVertices(),
                          rigidbody.getStrokeWeight(), rigidbody.getFillColour(),
                          rigidbody.getStrokeColour(), false);
          }
        }
      }
  }

  public void drawCircle(PVector position, float radius, float angle, float strokeWeight, PVector fillColour,
    PVector strokeColour, boolean inEditMode) {

    float diameter = radius * 2.0f;

    this.stroke.set(strokeColour);
    this.fill.set(fillColour);


    if(inEditMode) {
      fill(this.fill.x, this.fill.y, this.fill.z, this.opacity);
      stroke(this.stroke.x, this.stroke.y, this.stroke.z, this.opacity);
    } else {
      fill(this.fill.x, this.fill.y, this.fill.z);
      stroke(this.stroke.x, this.stroke.y, this.stroke.z);
    }

    strokeWeight(strokeWeight);
    ellipseMode(CENTER);
    ellipse(position.x, position.y,  diameter,  diameter);

    PVector va = new PVector();
    PVector vb = new PVector(radius, 0);
    va = PhysEngMath.Transform(va, position, angle);
    vb = PhysEngMath.Transform(vb, position, angle);
    line(va.x, va.y, vb.x, vb.y);
    }

  public void drawCircle(PVector position, float radius, float angle, float strokeWeight, PVector fillColour,
    PVector strokeColour, float opacity) {

    float diameter = radius * 2.0f;

    this.stroke.set(strokeColour);
    this.fill.set(fillColour);

    fill(this.fill.x, this.fill.y, this.fill.z, opacity);
    stroke(this.stroke.x, this.stroke.y, this.stroke.z, opacity);

    strokeWeight(strokeWeight);
    ellipseMode(CENTER);
    ellipse(position.x, position.y,  diameter,  diameter);

    PVector va = new PVector();
    PVector vb = new PVector(radius, 0);
    va = PhysEngMath.Transform(va, position, angle);
    vb = PhysEngMath.Transform(vb, position, angle);
    line(va.x, va.y, vb.x, vb.y);
  }
  

  public void drawBox(PVector position, float width, float height, float angle, float strokeWeight,
    PVector fillColour, PVector strokeColour, boolean inEditMode) {

    this.stroke.set(strokeColour);
    this.fill.set(fillColour);


    if(inEditMode) {
      fill(this.fill.x, this.fill.y, this.fill.z, this.opacity);
      stroke(this.stroke.x, this.stroke.y, this.stroke.z, this.opacity);
    } else {
      fill(this.fill.x, this.fill.y, this.fill.z);
      stroke(this.stroke.x, this.stroke.y, this.stroke.z);
    }
    strokeWeight(strokeWeight);
    rectMode(CENTER);
    pushMatrix();
    rect(position.x, position.y, width, height);
    popMatrix();
  }

  public void drawBox(PVector position, float width, float height, float angle, float strokeWeight,
    PVector fillColour, PVector strokeColour, int opacity) {

    this.stroke.set(strokeColour);
    this.fill.set(fillColour);


    fill(this.fill.x, this.fill.y, this.fill.z, opacity);
    stroke(this.stroke.x, this.stroke.y, this.stroke.z, opacity);

    strokeWeight(strokeWeight);
    rectMode(CENTER);

    pushMatrix();
    rotate(angle);
    rect(position.x, position.y, width, height);
    popMatrix();
  }

  
  public void drawPolygon(PVector position, PVector[] transformedVertices, float strokeWeight,
    PVector fillColour, PVector strokeColour, boolean inEditMode) {
    
    this.stroke.set(strokeColour);
    this.fill.set(fillColour);

    if(inEditMode) {
      fill(this.fill.x, this.fill.y, this.fill.z, this.opacity);
      stroke(this.stroke.x, this.stroke.y, this.stroke.z, this.opacity);
    } else {
      fill(this.fill.x, this.fill.y, this.fill.z);
      stroke(this.stroke.x, this.stroke.y, this.stroke.z);
    }

    strokeWeight(strokeWeight);

    beginShape();
    for (PVector transformedVertex : transformedVertices) {
      vertex(transformedVertex.x, transformedVertex.y);
    }
    endShape(CLOSE);
  }






  public void drawAABB() {
    for(Rigidbody rigidbody : rigidbodyList) {
        AABB aabb = rigidbody.GetAABB();
        rectMode(CORNERS);
        stroke(255, 0, 0);
        noFill();
        rect(aabb.getMin().x, aabb.getMin().y, aabb.getMax().x, aabb.getMax().y);
    }
  }

/*-----------------------------------------------------------------------------------------------*/
public void drawForces() {
    for(Rigidbody rigidbody : rigidbodyList) {
        for(ForceRegistry force : rigidbody.getForceRegistry()) {
            force.draw();
        }
    }
}
/*---------------------------------Collision Point Debugging--------------------------------------*/
  public void drawCollisionPoints() {
      for(PVector point : pointsOfContactList) {
        stroke(0, 0, 0);
        strokeWeight(0.1f);
        noFill();
        rectMode(CENTER);
        rect(point.x, point.y, 1, 1);
      }
        pointsOfContactList.clear();
    }

  
/*-----------------------------------------------------------------------------------------------*/

/*
public void drawGrid(){

  float majorGridSize = 120;
  float secondaryMajorGridSize = 60;
  float minorGridSize = 30;

  // Calculate the number of grid lines based on the screen size and grid size
  int majorNumVerticalLines = min(ceil(width / majorGridSize), 1000);
  int secondaryMajornumVerticalLines = min(ceil(width / secondaryMajorGridSize), 1000);
  int minorNumVerticalLines = min(ceil(width / minorGridSize), 1000);

  int majorNumHorizontalLines = min(ceil(height / majorGridSize), 1000);
  int secondaryMajornumHorizontalLines = min(ceil(height / secondaryMajorGridSize), 1000);
  int minorNumHorizontalLines = min(ceil(height / minorGridSize), 1000);

  float majorOffsetX = ((Camera.position.x-width/2) * Camera.zoom) % majorGridSize;
  float majorOffsetY = ((Camera.position.y-height/2) * Camera.zoom) % majorGridSize;

  float secondaryMajorOffsetX = ((Camera.position.x-width/2)* Camera.zoom) % secondaryMajorGridSize;
  float secondaryMajorOffsetY = ((Camera.position.y-height/2) * Camera.zoom) % secondaryMajorGridSize;

  float minorOffsetX = ((Camera.position.x-width/2) * Camera.zoom) % minorGridSize;
  float minorOffsetY = ((Camera.position.y-height/2) * Camera.zoom) % minorGridSize;

// Draw the major vertical gridlines
for (int i = 0; i <= majorNumVerticalLines; i++) {
    float x = i * majorGridSize + majorOffsetX;
    fill(#3f3f3f);
    rect(x, 0, 1, height);
}

// Draw the major horizontal grid lines
for (int i = 0; i <= majorNumHorizontalLines; i++) {
    float y = i * majorGridSize + majorOffsetY;
    fill(#3f3f3f);
    rect(0, y, width, 0.25);
}

// Draw the secondary major vertical gridlines
for (int i = 0; i <= secondaryMajornumVerticalLines; i++) {
    float x = i * secondaryMajorGridSize + secondaryMajorOffsetX;
    fill(#3f3f3f);
    rect(x, 0, 0.5, height);
}

// Draw the secondary major horizontal grid lines
for (int i = 0; i <= secondaryMajornumHorizontalLines; i++) {
    float y = i * secondaryMajorGridSize + secondaryMajorOffsetY;
    fill(#3f3f3f);
    rect(0, y, width, 0.25);
}

//Draw the minor vertical gridlines
for (int i = 0; i <= minorNumVerticalLines; i++) {
    float x = i * minorGridSize + minorOffsetX;
    fill(#3f3f3f);
    rect(x, 0, 0.25, height);
}

// Draw the minor horizontal grid lines
for (int i = 0; i <= minorNumHorizontalLines; i++) {
    float y = i * minorGridSize + minorOffsetY;
    fill(#3f3f3f);
    rect(0, y, width, 0.25);
}
         
}
*/


}
