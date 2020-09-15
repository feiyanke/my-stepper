import processing.serial.*; 

Serial myPort;    // The serial port
float step_angle = 1.8;
int   steps = int(360/1.8);
float step_angle_r = step_angle * PI / 180;
float angle = 0;

void stepP() {
  angle += step_angle_r;
  if(angle > PI*2) {
    angle -= PI*2;
  }
}

void stepN() {
  angle -= step_angle_r;
  if(angle < 0) {
    angle += PI*2;
  }
}

void setup(){
  size(200, 200);
  background(255,255,255);
  myPort = new Serial(this, "COM1", 115200); 
  myPort.buffer(1);
  
  thread("run");
}


void draw(){
  background(255,255,255);
  translate(width/2, height/2);
  rotate(angle);
  strokeWeight(10);
  line(0, 0, 50, 0);
}

void serialEvent(Serial p) {
  int a = p.read();
  if(a == '1'){
    stepP();
  } else if(a == '2') {
    stepN();
  }
}
