public class MouseObjectResult {
	private Rigidbody SelectedRigidbody;
	private PVector SelectedRigidbodyPosition = new PVector();
	private PVector Coordinate = new PVector();
    private PVector LocalCoordinate = new PVector();


	public MouseObjectResult(Rigidbody selectedRigidbody, PVector coordinate) {

		this.SelectedRigidbody = selectedRigidbody;
        
		if(this.SelectedRigidbody != null) {
			this.SelectedRigidbodyPosition.set(selectedRigidbody.getPosition());
            //this.LocalCoordinate.set(PVector.sub(coordinate, this.SelectedRigidbody.getPosition()));
            this.LocalCoordinate.set(PhysEngMath.Transform(PVector.sub(coordinate, this.SelectedRigidbodyPosition), -this.SelectedRigidbody.getAngle()));
		}
		this.Coordinate.set(coordinate);
	}


	public Rigidbody getSelectedRigidbody() {
		return this.SelectedRigidbody;
	}

	public PVector getWorldCoordinate() {
		return this.Coordinate.copy();
	}

    public PVector getTransformedLocalCoordinate() {
        if(this.SelectedRigidbody == null) {
            return this.Coordinate.copy();
        }
        
        return this.LocalCoordinate.copy();
    }

	public String toString() {
		return "MouseObjectResult: " + SelectedRigidbody + ", " + Coordinate;
	}


}
