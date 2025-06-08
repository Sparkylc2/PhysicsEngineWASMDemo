public class Gravity implements ForceRegistry {
  private final Rigidbody rigidbody;

  public Gravity(Rigidbody rigidbody) {
    this.rigidbody = rigidbody;
  }

  @Override
  public PVector getForce(Rigidbody rigidbody, PVector position) {
    return GRAVITY_VECTOR.copy();
  }

  @Override
  public void draw() {
    // Do nothing
  }

    @Override
    public PVector getApplicationPoint(Rigidbody rigidbody, PVector position) {
      return rigidbody.getPosition();
    }
    @Override
    public Rigidbody getRigidbodyA() {
      return this.rigidbody;
    }
    @Override
    public Rigidbody getRigidbodyB(){
      return this.rigidbody;
    }
}
