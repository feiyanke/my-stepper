import processing.serial.*; 

Serial myPort;    // The serial port
int x = 0;
int y = 0;

void setup(){
  size(500, 500);
  //background(255,255,255);
  myPort = new Serial(this, "COM6", 115200); 
  myPort.buffer(1);
}

void draw(){
  //background(255,255,255);
  translate(width/2, height/2);
  strokeWeight(2);
  point(x,y);
}

void stepXP() {
  if(x < width/2 - 10) {
    x += 1;
  }
}

void stepXN() {
  if(x > -width/2 + 10) {
    x -= 1;
  }
}

void stepYP() {
  if(y < width/2 - 10) {
    y += 1;
  }
}

void stepYN() {
  if(y > -width/2 + 10) {
    y -= 1;
  }
}

void serialEvent(Serial p) {
  int a = p.read();
  if(a == '1'){
    stepXP();
  } else if(a == '2') {
    stepXN();
  } else if(a == '3'){
    stepYP();
  } else if(a == '4'){
    stepYN();
  }
}
