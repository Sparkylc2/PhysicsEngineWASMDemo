
public class MouseObject {

	private ArrayList<MouseObjectResult> interactionResults = new ArrayList<MouseObjectResult>();

	private PVector mouseCoordinates = new PVector();
    private PVector previousMouseCoordinates = new PVector();


    private Rigidbody initialSelectedRigidbody = null;
    private float easing = 0.175f;
    private boolean mLeft, mRight, mCenter;
    private boolean isMouseDown = false;
    private boolean mouseDownLeft = false;
    private boolean mouseDownRight = false;
    private PVector mouseDownCoordinates = new PVector();
    private boolean isMouseUp = false;
    private PVector mouseUpCoordinates = new PVector();


    private boolean isMouseOverUI = false;
    private boolean wasMousePressedOverWindow = false;

	private Rigidbody currentRigidbodyUnderMouse;

  	private boolean showCursorTrail = true;
  	private ArrayList<PVector> cursorTrailArrayList = new ArrayList<PVector>();


    private boolean snappingEnabled = true;


	public MouseObject() {
        //do nothing
	}


    public void updateMouse() {
        this.updateMouseCoordinates();
        this.currentRigidbodyUnderMouse = this.getRigidbodyUnderMouse();
    }

	public Rigidbody getRigidbodyUnderMouse() {
        PVector mouseCoordinates = Camera.screenToWorld();
    	for (Rigidbody rigidbody : rigidbodyList) {
            if(this.initialSelectedRigidbody == rigidbody) {
                continue;
            }

            if(!Collisions.IntersectAABBWithPoint(rigidbody.GetAABB(), mouseCoordinates)) {
           		continue;
            }
            if(rigidbody.contains(mouseCoordinates.x, mouseCoordinates.y)) {
                if(this.initialSelectedRigidbody != null) {
                    rigidbodyList.remove(this.initialSelectedRigidbody);
                    rigidbodyList.add(0, this.initialSelectedRigidbody);
                }
           		return rigidbody;
            }
    	}

    	return null;
	}	

	public void addSelectedRigidbody() {
		if(this.interactionResults.size() > 2) {
            this.interactionResults.clear();
            this.initialSelectedRigidbody = null;
		}

		this.interactionResults.add(new MouseObjectResult(this.currentRigidbodyUnderMouse, this.mouseCoordinates));
        this.initialSelectedRigidbody = this.currentRigidbodyUnderMouse;
        
        if(this.interactionResults.size() == 2) {
            if(this.interactionResults.get(0).getSelectedRigidbody() == null && this.interactionResults.get(1).getSelectedRigidbody() == null) {
                this.interactionResults.clear();
                this.initialSelectedRigidbody = null;
            }
        }

	}

	public void updateMouseCoordinates() {
        PVector snappedMouseCoordinates = PhysEngMath.WorldSnapController(this, this.currentRigidbodyUnderMouse, Camera.screenToWorld());
        this.previousMouseCoordinates.set(this.mouseCoordinates);
        this.mouseCoordinates.x = lerp(this.mouseCoordinates.x, snappedMouseCoordinates.x, this.easing);
        this.mouseCoordinates.y = lerp(this.mouseCoordinates.y, snappedMouseCoordinates.y, this.easing);
	}

    public void updateMouseDownCoordinates() {
        this.isMouseDown = true;
        this.isMouseUp = false;

        if(mouseButton == LEFT) {
            this.mouseDownLeft = true;
        }
        this.mouseDownCoordinates.set(PhysEngMath.WorldSnapController(this, this.currentRigidbodyUnderMouse, Camera.screenToWorld()));
    }

    public void updateMouseUpCoordinates() {
        this.isMouseUp = true;
        this.isMouseDown = false;
        this.addSelectedRigidbody();

        if(this.mouseDownLeft) {
            this.mouseDownLeft = false;
        }

        this.mouseUpCoordinates.set(PhysEngMath.WorldSnapController(this, this.currentRigidbodyUnderMouse, Camera.screenToWorld()));
    }
    
    public void drawCursor() {
        if(!showCursorTrail) {
            return;
        }
        fill(255, 255, 255, 121);
        stroke(0, 0, 0, 180);
        strokeWeight(0.025);

        ellipse(Camera.screenToWorld().x, Camera.screenToWorld().y, 0.175f, 0.175f);
        fill(255, 0, 0);
        strokeWeight(0.1f);
        stroke(255, 0, 0);
        ellipse(this.mouseCoordinates.x, this.mouseCoordinates.y, 0.1f, 0.1f);


        if(cursorTrailArrayList.size() < 20) {
            cursorTrailArrayList.add(new PVector(mouseCoordinates.x, mouseCoordinates.y));
        } else {
            cursorTrailArrayList.add(cursorTrailArrayList.get(0).set(this.mouseCoordinates.x, this.mouseCoordinates.y));
            cursorTrailArrayList.remove(0);
        }
        noFill();
        beginShape();
            // Add the first point twice to guide the beginning of the curve
            curveVertex(cursorTrailArrayList.get(0).x, cursorTrailArrayList.get(0).y);
            for(PVector cursorTrailVertex : cursorTrailArrayList) {
                curveVertex(cursorTrailVertex.x, cursorTrailVertex.y);
            }
            // Add the last point twice to guide the end of the curve
            curveVertex(cursorTrailArrayList.get(cursorTrailArrayList.size() - 1).x, cursorTrailArrayList.get(cursorTrailArrayList.size() - 1).y);
        endShape();
    }

    public void setMouseCoordinates(PVector mouseCoordinates) {
        this.mouseCoordinates.set(mouseCoordinates);
    }

	public PVector getMouseCoordinates() {
		return this.mouseCoordinates;
	}

    public PVector getPreviousMouseCoordinates() {
        return this.previousMouseCoordinates;
    }
    
    public boolean getIsMouseDown() {
        return this.isMouseDown;
    }

    public PVector getMouseDownCoordinates() {
        return this.mouseDownCoordinates;
    }

    public boolean getIsMouseUp() {
        return this.isMouseUp;
    }
    public PVector getMouseUpCoordinates() {
        return this.mouseUpCoordinates;
    }

    public Rigidbody getCurrentRigidbodyUnderMouse() {
        return this.currentRigidbodyUnderMouse;
    }

    public ArrayList<MouseObjectResult> getMouseObjectResults() {
        return this.interactionResults;
    }

    public void clearMouseObjectResults() {
        this.interactionResults.clear();
    }
    
    public void setSnappingEnabled(boolean snappingEnabled) {
        this.snappingEnabled = snappingEnabled;
    }

    public boolean getSnappingEnabled() {
        return this.snappingEnabled;
    }

    public boolean getIsMouseDownLeft() {
        return this.mouseDownLeft;
    }
}
