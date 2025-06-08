public class AABB {

  private final PVector Min;
  private final PVector Max;

/*
====================================================================================================
========================================== Constructors ============================================
====================================================================================================
*/

  public AABB(PVector min, PVector max) {
      this.Min = min;
      this.Max = max;
  }
  
  
/*-------------------------------------Overloaded Constructor-------------------------------------*/
  public AABB(float minX, float minY, float maxX, float maxY) {
      this.Min = new PVector(minX, maxY);
      this.Max = new PVector(maxX, maxY);
  }
  
  public AABB(PVector vec1, PVector vec2, boolean autoFix) {
      this.Min = new PVector(min(vec1.x, vec2.x), min(vec1.y, vec2.y));
      this.Max = new PVector(max(vec1.x, vec2.x), max(vec1.y, vec2.y));
  
  }
  
  public void drawAABB() {
    rectMode(CORNERS);
    dash.rect(Min.x, Min.y, Max.x, Max.y);
  }
  

  
  public void shiftAABB(PVector amount) {
    this.Min.add(amount);
    this.Max.add(amount);
  }


  public float calculateArea() {
    return abs((Max.x - Min.x) * (Max.y - Min.y));
  }

  public void recalculateMaxAndMin(ArrayList<Rigidbody> rigidbodies) {
    float minX = Float.MAX_VALUE;
    float minY = Float.MAX_VALUE;
    float maxX = -Float.MAX_VALUE;
    float maxY = -Float.MAX_VALUE;


    float prcnt = 0.125f;

    for (Rigidbody rb : rigidbodies) {
      PVector rbMin = rb.GetAABB().getMin();
      PVector rbMax = rb.GetAABB().getMax();

      if (rbMin.x < minX) {
        minX = rbMin.x;
      }
      if (rbMin.y < minY) {
        minY = rbMin.y;
      }
      if (rbMax.x > maxX) {
        maxX = rbMax.x;
      }
      if (rbMax.y > maxY) {
        maxY = rbMax.y;
      }
    }

    float prcntPad = max(abs(maxX - minX) * prcnt, abs(maxY - minY) * prcnt);
    
    Min.x = minX - prcntPad;
    Min.y = minY - prcntPad;
    Max.x = maxX + prcntPad;
    Max.y = maxY + prcntPad;
  }


  public PVector calculateCenter() {
    return new PVector((Max.x + Min.x) / 2, (Max.y + Min.y) / 2);
  }



/*
====================================================================================================
==============================================GETTERS & SETTERS=====================================
====================================================================================================
*/
    public PVector getMin() {
      return Min;
    }
  
    public PVector getMax() {
      return Max;
    }
  
}