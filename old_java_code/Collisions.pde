public class CollisionResult {
 
  private boolean isColliding;
  private PVector normal = new PVector();
  private float depth;

  private float distanceSquared;

  private PVector[] pointsOfContact;

  private int contactCount;

  


/*
====================================================================================================
================================ PointSegmentDistance Constructor ==================================
====================================================================================================
*/

//Default constructor for no collision
  public CollisionResult(){

    this.isColliding = false;
    this.normal.set(0,0,0);
    this.depth = 0f;
    this.pointsOfContact = null;
    this.contactCount = 0;
  }


  public CollisionResult(float distanceSquared, PVector pointOfContact) {


    this.distanceSquared = distanceSquared;

    this.pointsOfContact = new PVector[] {pointOfContact};
  }



  public CollisionResult(PVector[] pointsOfContact) {
    this.isColliding = true;

    this.pointsOfContact = pointsOfContact;
    this.contactCount = pointsOfContact.length;
  }

  public CollisionResult(boolean isColliding, PVector normal, float depth, PVector[] pointsOfContact) {
    this.isColliding = isColliding;
    this.normal = normal;
    this.depth = depth;

    this.pointsOfContact = pointsOfContact;
    this.contactCount = pointsOfContact.length;
  }

  
  public CollisionResult(boolean isColliding, PVector normal, float depth) {
    this.isColliding = isColliding;
    this.normal = normal;
    this.depth = depth;
    this.pointsOfContact = null;
    this.contactCount = 0;
  }
 
  /*
  =====================================================================================================
  =========================================== Getters/Setters =========================================
  =====================================================================================================
  */

  public boolean getIsColliding() {
    return isColliding;
  }
  
  public PVector getNormal() {
    return normal;
  }

  public float getDepth() {
    return depth;
  }

  public PVector[] getPointsOfContact() {
    return pointsOfContact;
  }

  public int getContactCount() {
    return contactCount;
  }
  
  public float getDistanceSquared() {
    return distanceSquared;
  }


  public void setIsColliding(boolean isColliding) {
    this.isColliding = isColliding;
  }

  public void setNormal(PVector normal) {
    this.normal = normal;
  }

  public void setDepth(float depth) {
    this.depth = depth;
  }
  public void setDistanceSquared(float distanceSquared) {
    this.distanceSquared = distanceSquared;
  }

  //Overloaded Methods
  public void setPointsOfContact(PVector[] pointsOfContact) {
    this.pointsOfContact = pointsOfContact;
  }
  
  public void setPointsOfContact(PVector pointOfContact) {
    this.pointsOfContact = new PVector[] {pointOfContact};
  }

  public void setPointsOfContact(PVector pointOfContactA, PVector pointOfContactB) {
    this.pointsOfContact = new PVector[] {pointOfContactA, pointOfContactB};
  }

  public void setContactCount(int contactCount) {
    this.contactCount = contactCount;
  }

}












public class CollisionManifold {

    private final Rigidbody RigidbodyA;
    private final Rigidbody RigidbodyB;
    private final PVector Normal;
    private final float Depth;

    private final PVector[] PointsOfContact;
    private final int ContactCount;

    public CollisionManifold(Rigidbody rigidbodyA, Rigidbody rigidbodyB,
                             CollisionResult collisionResult) {

        this.RigidbodyA = rigidbodyA;
        this.RigidbodyB = rigidbodyB;
        this.Normal = collisionResult.getNormal();
        this.Depth = collisionResult.getDepth();
        this.PointsOfContact = collisionResult.getPointsOfContact();
        this.ContactCount = collisionResult.getContactCount();
    }
    

    public Rigidbody getRigidbodyA() {
        return RigidbodyA;
    }

    public Rigidbody getRigidbodyB() {
        return RigidbodyB;
    }

    public PVector getNormal() {
        return Normal;
    }

    public float getDepth() {
        return Depth;
    }

    public PVector[] getPointsOfContact() {
        return PointsOfContact;
    }

    public int getContactCount() {
        return ContactCount;
    }


}














public static class Collisions {
  CollisionResult collisionResult;

  //This is required as there is no enclosing instance of physics class for collisionResult
  public static PhysicsEngine physicsEngine = new PhysicsEngine();




  /*------------------- Reusable  ------------------- */
  public static PVector ZeroNormal = new PVector();
/*
====================================================================================================
===================================== COLLIDE INFO =================================================
====================================================================================================
*/

/*optimizied*/
public static CollisionResult Collide(Rigidbody rigidbodyA, Rigidbody rigidbodyB) {

    ShapeType shapeTypeA = rigidbodyA.getShapeType();
    ShapeType shapeTypeB = rigidbodyB.getShapeType();

    if(shapeTypeA == ShapeType.BOX) {

      if(shapeTypeB == ShapeType.BOX) {

        return IntersectPolygon(rigidbodyA.getPosition(),
                                           rigidbodyA.GetTransformedVertices(),
                                           rigidbodyB.getPosition(),
                                           rigidbodyB.GetTransformedVertices());

      } else if(shapeTypeB == ShapeType.CIRCLE) {
        CollisionResult result = Collisions.IntersectCirclePolygon(rigidbodyB.getPosition(),
                                                                   rigidbodyB.getRadius(),
                                                                   rigidbodyA.getPosition(),
                                                                   rigidbodyA.GetTransformedVertices()
                                                                   );
        result.setNormal(result.getNormal().mult(-1));
        return result;

      }

    }
  if (shapeTypeA == ShapeType.CIRCLE) {
        
        if(shapeTypeB == ShapeType.BOX) {

          return IntersectCirclePolygon(rigidbodyA.getPosition(), 
                                                   rigidbodyA.getRadius(), 
                                                   rigidbodyB.getPosition(), 
                                                   rigidbodyB.GetTransformedVertices());

        } else if(shapeTypeB == ShapeType.CIRCLE) {

          return IntersectCircle(rigidbodyA.getPosition(), rigidbodyB.getPosition(), rigidbodyA.getRadius(), rigidbodyB.getRadius());

        }
      }

    return physicsEngine.new CollisionResult();
  }


/*
====================================================================================================
===================================== CONTACT-POINTS COLLISIONS ====================================
======================================= COLLISION-RESULT ===========================================
*/

/*optimized*/

public static void FindCollisionPoints(Rigidbody rigidbodyA, Rigidbody rigidbodyB,
                                                  CollisionResult collisionResult) {
    int contactCount = 0;

    ShapeType shapeTypeA = rigidbodyA.getShapeType();
    ShapeType shapeTypeB = rigidbodyB.getShapeType();

    if(shapeTypeA == ShapeType.BOX) {

      if(shapeTypeB == ShapeType.BOX) {
        PVector[] pointsOfContact = FindPolygonsCollisionPoints(rigidbodyA.GetTransformedVertices(),
                                                                rigidbodyB.GetTransformedVertices());
        collisionResult.setPointsOfContact(pointsOfContact);
        collisionResult.setContactCount(pointsOfContact.length);
        return;

      } else if(shapeTypeB == ShapeType.CIRCLE) {
        PVector[] pointsOfContact = FindCirclePolygonCollisionPoint(rigidbodyB.getPosition(),
                                                                    rigidbodyB.getRadius(),
                                                                    rigidbodyA.getPosition(),
                                                                    rigidbodyA.GetTransformedVertices());
        collisionResult.setPointsOfContact(pointsOfContact);
        collisionResult.setContactCount(1);
        return;
      }

    }
  if (shapeTypeA == ShapeType.CIRCLE) {
        
        if(shapeTypeB == ShapeType.BOX) {
          PVector[] pointsOfContact = FindCirclePolygonCollisionPoint(rigidbodyA.getPosition(),
                                                                      rigidbodyA.getRadius(),
                                                                      rigidbodyB.getPosition(),
                                                                      rigidbodyB.GetTransformedVertices());
          collisionResult.setPointsOfContact(pointsOfContact);
          collisionResult.setContactCount(1);
          return;

        } else if(shapeTypeB == ShapeType.CIRCLE) {
           PVector[] pointsOfContact = FindCirclesCollisionPoint(rigidbodyA.getPosition(),
                                                                 rigidbodyA.getRadius(),
                                                                 rigidbodyB.getPosition(),
                                                                 rigidbodyB.getRadius());
            collisionResult.setPointsOfContact(pointsOfContact);
            collisionResult.setContactCount(1);
            return;
        }
      }

    /*
    For displaying the points of contact
        for(PVector point : pointsOfContact) {
             //pointsOfContactList.add(point);
        }
   */

}
/*
====================================================================================================
============================== CIRCLE-CIRCLE COLLISION CONTACT POINT ===============================
======================================= COLLISION-RESULT ===========================================
====================================================================================================
*/
 private static PVector[] FindCirclesCollisionPoint(PVector centerA, float radiusA,
                                                         PVector centerB, float radiusB) {
    

    return new PVector[] {PVector.add(centerA, PVector.sub(centerB, centerA).normalize().mult(radiusA))};
    }
/*
====================================================================================================
=============================POLYGON-POLYGON COLLISION CONTACT POINT ===============================
======================================= COLLISION-RESULT ===========================================
====================================================================================================
*/


private static PVector[] FindPolygonsCollisionPoints(PVector[] transformedVerticesA,
                                                           PVector[] transformedVerticesB) {
  PVector contactPointA = new PVector();
  PVector contactPointB = new PVector();

  PVector vertexA = new PVector();
  PVector vertexB = new PVector();
  
  PVector point = new PVector();
  int contactCount = 0;

  float minDistanceSquared = Float.MAX_VALUE;

  for(int i = 0; i < transformedVerticesA.length; i++) {

      point.set(transformedVerticesA[i]);

      for(int j = 0; j < transformedVerticesB.length; j++) {
        
        CollisionResult pointSegmentDistanceResult = PointSegmentDistance(point, transformedVerticesB[j], transformedVerticesB[(j + 1) % transformedVerticesB.length]);

        if(PhysEngMath.Equals(pointSegmentDistanceResult.getDistanceSquared(),minDistanceSquared)) {

            if(!PhysEngMath.Equals(pointSegmentDistanceResult.getPointsOfContact()[0], contactPointA)) {

              contactPointB.set(pointSegmentDistanceResult.getPointsOfContact()[0]);
              contactCount = 2;

            }
            
        } else if(pointSegmentDistanceResult.getDistanceSquared() < minDistanceSquared) {

            minDistanceSquared = pointSegmentDistanceResult.getDistanceSquared();
            contactPointA.set(pointSegmentDistanceResult.getPointsOfContact()[0]);
            contactCount = 1;
        }
      }
    }


  for(int i = 0; i < transformedVerticesB.length; i++) {

      point.set(transformedVerticesB[i]);

      for(int j = 0; j < transformedVerticesA.length; j++) {
          
          vertexA.set(transformedVerticesA[j]);
          vertexB.set(transformedVerticesA[(j + 1) % transformedVerticesA.length]);
  
          CollisionResult pointSegmentDistanceResult = PointSegmentDistance(point, transformedVerticesA[j], transformedVerticesA[(j + 1) % transformedVerticesA.length]);

        if(PhysEngMath.Equals(pointSegmentDistanceResult.getDistanceSquared(),minDistanceSquared)) {

            if(!PhysEngMath.Equals(pointSegmentDistanceResult.getPointsOfContact()[0], contactPointA)) {
              contactPointB.set(pointSegmentDistanceResult.getPointsOfContact()[0]);
              contactCount = 2;
            }

        } else if(pointSegmentDistanceResult.getDistanceSquared() < minDistanceSquared) {
            minDistanceSquared = pointSegmentDistanceResult.getDistanceSquared();
            contactPointA.set(pointSegmentDistanceResult.getPointsOfContact()[0]);
            contactCount = 1;
        }
      }
    }

    if(contactCount == 1){
      return new PVector[] {contactPointA};
    }
    
    return new PVector[] {contactPointA, contactPointB};
  
}

/*
====================================================================================================
=============================CIRCLE-POLYGON COLLISION CONTACT POINT ===============================
======================================= COLLISION-RESULT ===========================================
====================================================================================================
*/

/*Somewhat optimized */

private static PVector[] FindCirclePolygonCollisionPoint(PVector circleCenter,
                                                         float circleRadius,
                                                         PVector polygonCenter,
                                                         PVector[] transformedVertices) {

    float minDistanceSquared = Float.MAX_VALUE;
    PVector contactPoint = new PVector();

    for(int i = 0; i < transformedVertices.length; i++) {

        CollisionResult pointSegmentDistanceResult = PointSegmentDistance(circleCenter, transformedVertices[i], transformedVertices[(i + 1) % transformedVertices.length]); 

        if(pointSegmentDistanceResult.getDistanceSquared() < minDistanceSquared) {

            minDistanceSquared = pointSegmentDistanceResult.getDistanceSquared();
            contactPoint.set(pointSegmentDistanceResult.getPointsOfContact()[0]);
        }
    }

    return new PVector[] {contactPoint};
  }


/*
====================================================================================================
===================================== AABB-AABB Collisions =========================================
====================================================================================================
*/
public static boolean IntersectAABB (AABB aabbA, AABB aabbB) {

  if(aabbA.getMax().x <= aabbB.getMin().x || aabbB.getMax().x <= aabbA.getMin().x
    || aabbA.getMax().y <= aabbB.getMin().y || aabbB.getMax().y <= aabbA.getMin().y) {

    return false;
  }

  return true;
}

/*
====================================================================================================
===================================== AABB-AABB Point Collision ====================================
====================================================================================================
*/
public static boolean IntersectAABBWithPoint(AABB aabb, PVector point) {
    if (point.x >= aabb.getMin().x && point.x <= aabb.getMax().x &&
        point.y >= aabb.getMin().y && point.y <= aabb.getMax().y) {
        return true; // The point is within the AABB
    }
    return false; // The point is outside the AABB
  }

/*
====================================================================================================
===================================== CIRCLE-CIRCLE COLLISIONS =====================================
======================================= COLLISION-RESULT ===========================================
====================================================================================================
*/
/*somewhat Optimized*/
  public static CollisionResult IntersectCircle(PVector centerA, PVector centerB,
                                                float radiusA, float radiusB) {

    PVector direction = PVector.sub(centerB, centerA);
    float distance = direction.mag();
    float radiusSum = (radiusA + radiusB);
    
    if(distance < radiusSum) {
      return physicsEngine.new CollisionResult(true, direction.normalize(), radiusSum - distance);
    } else {
      return physicsEngine.new CollisionResult();
    }
  }
  
                                             

/*
====================================================================================================
===================================== POLYGON-POLYGON COLLISIONS ===================================
======================================= COLLISION-RESULT============================================
====================================================================================================
*/

/*Optimized somewhat*/
public static CollisionResult IntersectPolygon(PVector centerA,
                                               PVector[] transformedVerticesA,
                                               PVector centerB,
                                               PVector[] transformedVerticesB) {

    float depth = Float.MAX_VALUE;

    PVector axis = new PVector();
    PVector normal = new PVector();
    PVector edge = new PVector();

    PVector transformedVertexA = new PVector();
    PVector transformedVertexB = new PVector();

    PVector secondTransformedVertexA = new PVector();
    PVector secondTransformedVertexB = new PVector();


    for(int vertexIndexA = 0; vertexIndexA < transformedVerticesA.length; vertexIndexA++) {
 

      //Gets the transformed vertices in polygon A, when at the end of the list, loops back to the start
      transformedVertexA.set(transformedVerticesA[vertexIndexA]);
      transformedVertexB.set(transformedVerticesA[(vertexIndexA + 1) % transformedVerticesA.length]);

      //Finds the edge between the two vertices,
      edge.set(PVector.sub(transformedVertexB, transformedVertexA));

      //Finds the normal or "axis" from the edge vector
      axis.set(-edge.y, edge.x);
      axis.normalize();

      //Projects the vertices of polygon A onto the axis. Format is [min, max]
      float[] minMaxA = ProjectVertices(transformedVerticesA, axis);
      float[] minMaxB = ProjectVertices(transformedVerticesB, axis);

      if(minMaxA[0] >= minMaxB[1] || minMaxB[0] >= minMaxA[1]) {
        return physicsEngine.new CollisionResult();
      }

      float axisDepth = min(minMaxB[1]-minMaxA[0], minMaxA[1]-minMaxB[0]);

      if(axisDepth < depth) {
        depth = axisDepth;
        normal.set(axis);
      }
    }

    for(int vertexIndexB = 0; vertexIndexB < transformedVerticesB.length; vertexIndexB++) {
      //!!!ALL OF THIS ASSUMES A CLOCKWISE WINDING ORDER!!!

      //Gets the transformed vertices in polygon A, when at the end of the list, loops back to the start
      secondTransformedVertexA.set(transformedVerticesB[vertexIndexB]);
      secondTransformedVertexB.set(transformedVerticesB[(vertexIndexB + 1) % transformedVerticesB.length]);

      //Finds the edge between the two vertices, 
      edge.set(PVector.sub(secondTransformedVertexB, secondTransformedVertexA));

      //Finds the normal or "axis" from the edge vector
      axis.set(-edge.y, edge.x);
      axis.normalize();

      //Projects the vertices of polygon A onto the axis. Format is [min, max]
      float[] minMaxA = ProjectVertices(transformedVerticesA, axis);
      float[] minMaxB = ProjectVertices(transformedVerticesB, axis);

      if(minMaxA[0] >= minMaxB[1] || minMaxB[0] >= minMaxA[1]) {        
        return physicsEngine.new CollisionResult();
      }

      float axisDepth = min(minMaxB[1]-minMaxA[0], minMaxA[1]-minMaxB[0]);
      
      if(axisDepth < depth) {
        depth = axisDepth;
        normal.set(axis);
      }
    }

    //This is correction code so that the normal points in the correct direction
    //If its not pointing in the correct direction, flip the normal
  
    if(PVector.dot(PVector.sub(centerB, centerA), normal) < 0) {
      normal.mult(-1);
    }

    return physicsEngine.new CollisionResult(true, normal, depth);
  }

//The overloaded method is used when the center of the polygon is not known


/*
====================================================================================================
===================================== CIRCLE-POLYGON COLLISIONS ====================================
======================================= COLLISION-RESULT ===========================================
====================================================================================================
*/


public static CollisionResult IntersectCirclePolygon(PVector circleCenter, float circleRadius,
                                                     PVector polygonCenter,
                                                     PVector[] transformedVertices){
    boolean isColliding;
    float depth = Float.MAX_VALUE;
    PVector normal = new PVector();

    PVector edge = new PVector();
    PVector axis = new PVector();

    float axisDepth = 0f;

    for(int vertexIndex = 0; vertexIndex < transformedVertices.length; vertexIndex++) {
      //!!!ALL OF THIS ASSUMES A CLOCKWISE WINDING ORDER!!!

      //Finds the edge between the two vertices,
      edge.set(PVector.sub(transformedVertices[(vertexIndex + 1) % transformedVertices.length], transformedVertices[vertexIndex]));
      
      //Finds the normal or "axis" from the edge vector
       axis.set(-edge.y, edge.x);
       axis.normalize();

      //Projects the vertices of polygon A onto the axis. Format is [min, max]
      float[] minMaxA = ProjectVertices(transformedVertices, axis);

      //Projects the circle onto the axis. Format is [min, max]
      float[] minMaxB = ProjectCircle(circleCenter, axis, circleRadius);

      if(minMaxA[0] >= minMaxB[1] || minMaxB[0] >= minMaxA[1]) {

        return physicsEngine.new CollisionResult();
      }

       axisDepth = min(minMaxB[1]-minMaxA[0], minMaxA[1]-minMaxB[0]);
        if(axisDepth < depth) {
            depth = axisDepth;
            normal.set(axis);
        }
  }

    int index = FindClosestPointOnPolygon(circleCenter, transformedVertices);
    
    if(index == -1) {
      return physicsEngine.new CollisionResult();
    }
    axis.set(PVector.sub(transformedVertices[index], circleCenter).normalize());

     //Projects the vertices of polygon A onto the axis. Format is [min, max]
      float[] minMaxA = ProjectVertices(transformedVertices, axis);
      //Projects the circle onto the axis. Format is [min, max]
      float[] minMaxB = ProjectCircle(circleCenter, axis, circleRadius);


      if(minMaxA[0] >= minMaxB[1] || minMaxB[0] >= minMaxA[1]) {
        
        return physicsEngine.new CollisionResult();
      }

      axisDepth = min(minMaxB[1]-minMaxA[0], minMaxA[1]-minMaxB[0]);

      if(axisDepth < depth) {
        depth = axisDepth;
        normal.set(axis);
      }

        if(PVector.dot(PVector.sub(polygonCenter, circleCenter), normal) < 0) {
          normal.mult(-1);
        }
  
        isColliding = true;

        return physicsEngine.new CollisionResult(isColliding, normal, depth);
}




/*
====================================================================================================
========================================= HELPER-METHODS ===========================================
====================================================================================================
*/

//Returns an array with array = [min, max]
private static float[] ProjectVertices(PVector[] vertices, PVector axis){

  float min = Float.MAX_VALUE;
  float max = Float.MIN_VALUE;

  for(PVector vertex : vertices) {

    float projectedVertex = PVector.dot(vertex, axis);

    if(projectedVertex < min) {
      min = projectedVertex;
    }

    if(projectedVertex > max) {
      max = projectedVertex;
    }

  }
  return new float[] {min, max};
}

private static float[] ProjectCircle(PVector center, PVector axis, float radius) {
  
  PVector directionAndRadius = PVector.mult(axis, radius);

  float min = PVector.dot(PVector.add(center, directionAndRadius), axis);
  float max = PVector.dot(PVector.sub(center, directionAndRadius), axis);
   
  if(min > max){
    float temp = min;
    min = max;
    max = temp;
  }

  return new float[] {min, max};
}

private static int FindClosestPointOnPolygon(PVector circleCenter, PVector[] transformedVertices) {
    int result = -1;
    float minDistanceSq = Float.MAX_VALUE;

    // Check for null or empty input
    if (transformedVertices == null || transformedVertices.length == 0 || circleCenter == null) {
        System.out.println("Input is null or empty");
        return result; // Return -1 to indicate error
    }

    for (int vertexIndex = 0; vertexIndex < transformedVertices.length; vertexIndex++) {
        // Additional check for null vertices, if necessary
        if (transformedVertices[vertexIndex] == null) {
            continue; // Skip this iteration
        }

        float distanceSq = PVector.sub(circleCenter, transformedVertices[vertexIndex]).magSq();

        if (distanceSq < minDistanceSq) {
            minDistanceSq = distanceSq;
            result = vertexIndex;
        }
    }

    return result;
}



public static CollisionResult PointSegmentDistance(PVector point, PVector lineSegmentStart, PVector lineSegmentEnd) {
    //PVector.sub(point, lineSegmentStart) is equivalent to pointToLineSegment

    PVector lineSegment = new PVector();
    lineSegment.set(PVector.sub(lineSegmentEnd, lineSegmentStart));

    float projection = PVector.dot(PVector.sub(point, lineSegmentStart), lineSegment);
    float lineSegmentLengthSquared = lineSegment.magSq();

    float distance = projection / lineSegmentLengthSquared;

    PVector closestPoint = new PVector();

    if (distance <= 0f) {
        closestPoint.set(lineSegmentStart);
    } else if (distance >= 1f) {
        closestPoint.set(lineSegmentEnd);
    } else {
        closestPoint.set(PVector.add(lineSegmentStart, PVector.mult(lineSegment, distance)));
    }

    float distanceSquared = PVector.sub(point, closestPoint).magSq();

    return physicsEngine.new CollisionResult(distanceSquared, closestPoint);
    }

/*
====================================================================================================
========================================= LINE INTERSECTION ========================================
====================================================================================================
*/

    public static PVector CalculateLineIntersection(PVector p1, PVector p2, PVector p3, PVector p4) {
        float denominator = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

        if (denominator == 0) {
            return null; // The lines are parallel
        }

        float intersectX = ((p1.x * p2.y - p1.y * p2.x) * (p3.x - p4.x) - (p1.x - p2.x) * (p3.x * p4.y - p3.y * p4.x)) / denominator;
        float intersectY = ((p1.x * p2.y - p1.y * p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x * p4.y - p3.y * p4.x)) / denominator;

        return new PVector(intersectX, intersectY);
    }

    public static boolean IsPointInLineSegment(PVector point, PVector lineStart, PVector lineEnd) {
        float lineLength = PVector.dist(lineStart, lineEnd);
        float totalDist = PVector.dist(lineStart, point) + PVector.dist(point, lineEnd);

        // Allow for a small error due to floating point precision
        float epsilon = 0.0001f;

        return Math.abs(totalDist - lineLength) < epsilon;
    }
} 
