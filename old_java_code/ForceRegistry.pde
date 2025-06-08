public interface ForceRegistry {
    public PVector getForce(Rigidbody rigidbody, PVector position);
    public void draw();
    public PVector getApplicationPoint(Rigidbody rigidbody, PVector position);
    public Rigidbody getRigidbodyA();
    public Rigidbody getRigidbodyB();
}