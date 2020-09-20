import processing.serial.*; 

Serial myPort;    // The serial port
int x = 0;
int y = 0;

void setup(){
  size(1000, 1000);
  //background(255,255,255);
  myPort = new Serial(this, "COM6", 115200); 
  myPort.buffer(1);
}

void draw(){
  //while (myPort.available() > 0) {
  //  String inBuffer = myPort.readString();   
  //  if (inBuffer != null) {
  //    println(inBuffer);
  //  }
  //}
  
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

void mouseDragged(MouseEvent event) {
  int mx = event.getX() - (width/2);
  int my = event.getY() - (height/2);
  
  byte[] buffer = new byte[5];
  buffer[0] = 0;
  buffer[1] = (byte)(mx&0xFF);
  buffer[2] = (byte)((mx>>8)&0xFF);
  buffer[3] = (byte)(my&0xFF);
  buffer[4] = (byte)((my>>8)&0xFF);
  //println(millis()-time);
  //time = millis();
  myPort.write(buffer);
  println("mouse event ", mx, my);
  //println("byte ", hex(buffer[1]), hex(buffer[2]), hex(buffer[3]), hex(buffer[4]));
}
