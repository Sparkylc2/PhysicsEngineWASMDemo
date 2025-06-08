
public static class PhysEngMath {

    private static final PVector emptyVector = new PVector(0, 0);
    
    //Precision for float comparison, equal to 0.00005 meters
    public static final float precision = 0.0001f;

    public static float Clamp(float value, float min, float max) {
        if (min == max) {
            return min;
        } else if (min > max) {
            throw new IllegalArgumentException("min must be less than max");
        } else if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        } else {
            return value;
        }
  }


public static float fastInverseSq(PVector vector) {
    return fastInverseSq(vector.x * vector.x + vector.y * vector.y);
}


public static float fastInverseSq (float number) {
    float x = number;
    float xhalf = 0.5f * x;
    int i = Float.floatToIntBits(x);
    i = 0x5f3759df - (i >> 1);
    x = Float.intBitsToFloat(i);
    for (int it = 0; it < 2; it++) {
        x = x * (1.5f - xhalf * x * x);
    }
    x *= number;
    return x;
}

public static PVector[] OrderVerticesClockwise(PVector[] vertices) {
  // Calculate the centroid
  PVector centroid = new PVector(0, 0);
  for (PVector v : vertices) {
    centroid.add(v);
  }
  centroid.div(vertices.length);

  // Calculate angle for each vertex
  float[] angles = new float[vertices.length];
  for (int i = 0; i < vertices.length; i++) {
    angles[i] = PVector.sub(vertices[i], centroid).heading();
  }

  // Sort vertices based on angles
  for (int i = 0; i < angles.length; i++) {
    for (int j = i + 1; j < angles.length; j++) {
      if (angles[i] > angles[j]) {
        // Swap angles
        float tempAngle = angles[i];
        angles[i] = angles[j];
        angles[j] = tempAngle;
        // Swap vertices
        PVector tempVertex = vertices[i];
        vertices[i] = vertices[j];
        vertices[j] = tempVertex;
      }
    }
  }

  // Return ordered vertices
  return vertices;
}

public static PVector MouseVelocityCalculationAndClamp(PVector initial, PVector current, float min, float max) {

    float dx = current.x - initial.x;
    float dy = current.y - initial.y;

    float magnitude = fastInverseSq(dx * dx + dy * dy);  

    if (Equals(magnitude, 0f)) { // To avoid division by zero
        dx = 0f;
        dy = 0f;
    } else {
        dx /= magnitude;
        dy /= magnitude;
    }

    magnitude = constrain(magnitude, min, max);

    dx *= magnitude;
    dy *= magnitude;

    return new PVector(dx, dy);
}


public static PVector SquareVelocity(PVector vector) {
    float x = vector.x;
    float y = vector.y;

    float magSq = x * x + y * y;
    float mag = fastInverseSq(magSq);

    if(Equals(mag, 0f)) {
        x = 0f;
        y = 0f;
    } else {
        x /= mag;
        y /= mag;
    }

    return new PVector(x * magSq, y * magSq);
}

public static PVector Clamp(PVector vector, float min, float max) {

    float dx = vector.x;
    float dy = vector.y;

    float mag = fastInverseSq(dx * dx + dy * dy);

    if (Equals(mag, 0f)) { // To avoid division by zero
        dx = 0f;
        dy = 0f;
    } else {
        dx /= mag;
        dy /= mag;
    }

    mag = constrain(mag, min, max);

    dx *= mag;
    dy *= mag;

    return new PVector(dx, dy);
  }

  public static int Clamp(int value, int min, int max) {
    if (min == max) {
      return min;
    } else if (min > max) {
      throw new IllegalArgumentException("min must be less than max");
    } else if (value < min) {
      return min;
    } else if (value > max) {
      return max;
    } else {
      return value;
    }
  }
  

  public static PVector Transform(PVector vertex, float angle) {
    float sin = sin(angle);
    float cos = cos(angle);

    return new PVector(vertex.x * cos - vertex.y * sin, vertex.x * sin + vertex.y * cos);
  }
  
  public static PVector Transform(PVector vertex, PVector position, float angle){
    float sin = sin(angle);
    float cos = cos(angle);
    
    return new PVector(vertex.x * cos - vertex.y * sin + position.x, vertex.x * sin + vertex.y * cos + position.y);
  }

  public static PVector ReverseTransform(PVector vertex, PVector position, float angle){
  PVector translatedVertex = new PVector(vertex.x + position.x, vertex.y + position.y);


  float sin = sin(angle); 
  float cos = cos(angle);

  // Rotate the translated vertex
  return new PVector(
    translatedVertex.x * cos - translatedVertex.y * sin, // Rotate x
    translatedVertex.x * sin + translatedVertex.y * cos  // Rotate y
  );
}

  
  //Overloaded method for Transform
    public static PVector Transform(float x, float y, float angle) {
        float sin = sin(angle);
        float cos = cos(angle);
    
        return new PVector(x * cos - y * sin + x, x * sin + y * cos + y);
  }

  

  public static PVector zeroTransform = Transform(0, 0, 0);



    public static boolean Equals(float a, float b) {
        return Math.abs(a - b) < precision;
    }

public static boolean Equals(PVector a, PVector b) {
    return PVector.sub(a, b).magSq() < precision * precision; //magSq is faster than mag
  }

    

public static PVector SnapController(MouseObject Mouse, Rigidbody rigidbody, PVector point) {
    if(rigidbody == null) {
        return point;
    }
    
    if(Mouse.getSnappingEnabled()) {
        if(rigidbody.getShapeType() == ShapeType.BOX) {
            PVector[] vertices = rigidbody.GetTransformedVertices();

            for(int i = 0; i < vertices.length; i++) {
                if(PVector.sub(vertices[i], point).magSq() < VERTEX_SNAP_RADIUS) {
                    return PVector.sub(vertices[i], rigidbody.getPosition());
                }
            }

            PVector closestPoint = new PVector();
            float minDistanceSq = Float.MAX_VALUE;

            for (int i = 0; i < vertices.length; i++) {
                PVector start = vertices[i];
                PVector end = vertices[(i + 1) % vertices.length]; // Loop back to the first vertex

                PVector closestOnEdge = getClosestPointOnLine(start, end, point);
                float distanceSq = PVector.dist(closestOnEdge, point);

                if (distanceSq < minDistanceSq) {
                    minDistanceSq = distanceSq;
                    closestPoint = closestOnEdge;
                }
            }

            if(PVector.sub(closestPoint, rigidbody.getPosition()).magSq() / 2 > PVector.sub(point, rigidbody.getPosition()).magSq()) {
                return new PVector();
            } else {
                return PVector.sub(closestPoint, rigidbody.getPosition());
            }

        } else if(rigidbody.getShapeType() == ShapeType.CIRCLE) {
            if(PVector.sub(point, rigidbody.getPosition()).magSq() < rigidbody.getRadius() / 2) {
                return new PVector();
            } else {
                return point.sub(rigidbody.getPosition()).normalize().mult(rigidbody.getRadius()).copy();
            }
        } else {
            throw new IllegalArgumentException("Rigidbody is not a circle or box");
        }
    } else {
        return PVector.sub(point, rigidbody.getPosition());
    }
}

public static PVector WorldSnapController(MouseObject Mouse, Rigidbody rigidbody, PVector point) {
    if(rigidbody == null) {
        return point;
    }

    if(Mouse.getSnappingEnabled()) {
        if(rigidbody.getShapeType() == ShapeType.BOX) {
            PVector[] vertices = rigidbody.GetTransformedVertices();

            for(int i = 0; i < vertices.length; i++) {
                if(PVector.sub(vertices[i], point).magSq() < VERTEX_SNAP_RADIUS) {
                    return vertices[i];
                }
            }

            PVector closestPoint = new PVector();
            float minDistanceSq = Float.MAX_VALUE;

            for (int i = 0; i < vertices.length; i++) {
                PVector start = vertices[i];
                PVector end = vertices[(i + 1) % vertices.length]; // Loop back to the first vertex

                PVector closestOnEdge = getClosestPointOnLine(start, end, point);
                float distanceSq = PVector.dist(closestOnEdge, point);

                if (distanceSq < minDistanceSq) {
                    minDistanceSq = distanceSq;
                    closestPoint = closestOnEdge;
                }
            }

            if(PVector.sub(closestPoint, rigidbody.getPosition()).magSq() / 2 > PVector.sub(point, rigidbody.getPosition()).magSq()) {
                return rigidbody.getPosition();
            } else {
                return closestPoint;
            }

        } else if(rigidbody.getShapeType() == ShapeType.CIRCLE) {
            if(PVector.sub(point, rigidbody.getPosition()).magSq() < rigidbody.getRadius()/2) {
                return rigidbody.getPosition();
            } else {
                return point.sub(rigidbody.getPosition()).normalize().mult(rigidbody.getRadius()).add(rigidbody.getPosition());
            }
        } else {
            throw new IllegalArgumentException("Rigidbody is not a circle or box");
        }
    } else {
        return point;
    }
}


    public static PVector[] reverseVertices(PVector[] vertices) {
        PVector[] reversedVertices = new PVector[vertices.length];

        for (int i = 0; i < vertices.length; i++) {
            reversedVertices[i] = vertices[vertices.length - 1 - i];
        }
        return reversedVertices;
    }

    private static PVector getClosestPointOnLine(PVector start, PVector end, PVector point) {
        PVector line = PVector.sub(end, start);
        float len = line.mag();
        line.normalize();
        PVector v = PVector.sub(point, start);
        float d = PVector.dot(v, line);
        d = constrain(d, 0, len);
        return PVector.add(start, line.mult(d));
    }

}
