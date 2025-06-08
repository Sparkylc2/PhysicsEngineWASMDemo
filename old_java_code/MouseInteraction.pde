
public void mousePressed(){
    if(mouseButton == LEFT){
        Mouse.updateMouseDownCoordinates();
        UI_Manager.onMousePress();
        return;
    }
}

public void mouseReleased() {
    if(mouseButton == LEFT) {
        Mouse.updateMouseUpCoordinates();
        UI_Manager.onMouseRelease();
        return;
    }

}

public void mouseClicked() {
    if(mouseButton == LEFT) {
        UI_Manager.onMouseClick();
        return;
    }
} 

public void mouseWheel(MouseEvent event) {
    if(UI_Manager.getIsOverWindows()) {
        return;
    }
    
    float e = -event.getCount();
    Camera.zoom(pow(SCROLL_SENSITIVITY, e), mouseX, mouseY);
}

public void mouseDragged() {
    if(!UI_Manager.getIsOverWindows() && mouseButton == RIGHT) {
        Camera.move(pmouseX - mouseX, pmouseY - mouseY);
        return;
    }

    if(mouseButton == LEFT) {
        UI_Manager.onMouseDrag();
        return;
    }
}

public void mouseMoved() {
    //editor.dragSnap();
}

