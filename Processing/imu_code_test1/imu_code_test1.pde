
import processing.serial.*;

Serial myPort;

// Quaternion components
float qw = 1;
float qx = 0;
float qy = 0;
float qz = 0;

void setup() {
  size(900, 700, P3D);

  println(Serial.list());

  myPort = new Serial(this, "COM3", 115200);
  myPort.bufferUntil('\n');
}

void draw() {

  background(25);

  lights();
  directionalLight(255, 255, 255, 0, -1, -1);

  pushMatrix();

  translate(width/2, height/2, 0);

  // Normalize quaternion
  float magnitude = sqrt(
    qw*qw + qx*qx + qy*qy + qz*qz
  );

  if (magnitude > 0) {
    float w = qw / magnitude;
    float x = qx / magnitude;
    float y = qy / magnitude;
    float z = qz / magnitude;

    // Quaternion to rotation matrix
    float angle = 2 * acos(constrain(w, -1, 1));

    float s = sqrt(1 - w*w);

    float axisX = 1;
    float axisY = 0;
    float axisZ = 0;

    if (s > 0.0001) {
      axisX = x / s;
      axisY = y / s;
      axisZ = z / s;
    }

    rotate(angle, axisX, axisY, axisZ);
  }

  drawCube();

  popMatrix();

  drawHUD();
}

void serialEvent(Serial port) {

  String data = port.readStringUntil('\n');

  if (data != null) {

    data = trim(data);

    String[] values = split(data, ',');

    if (values.length == 4) {

      try {
        qw = float(values[0]);
        qx = float(values[1]);
        qy = float(values[2]);
        qz = float(values[3]);
      }
      catch (Exception e) {
        println("Invalid quaternion: " + data);
      }
    }
  }
}

void drawCube() {

  // Satellite body
  fill(100, 160, 220);
  stroke(255);
  strokeWeight(2);

  box(200);

  // X axis - Red
  stroke(255, 50, 50);
  strokeWeight(5);
  line(0, 0, 0, 180, 0, 0);

  // Y axis - Green
  stroke(50, 255, 50);
  line(0, 0, 0, 0, 180, 0);

  // Z axis - Blue
  stroke(50, 100, 255);
  line(0, 0, 0, 0, 0, 180);
}

void drawHUD() {

  hint(DISABLE_DEPTH_TEST);
  camera();

  fill(255);
  textSize(22);
  text("TSAT3A - QUATERNION ORIENTATION", 25, 40);

  textSize(17);

  fill(255, 100, 100);
  text("W: " + nf(qw, 1, 4), 25, 100);

  fill(100, 255, 100);
  text("X: " + nf(qx, 1, 4), 25, 135);

  fill(100, 150, 255);
  text("Y: " + nf(qy, 1, 4), 25, 170);

  fill(255, 200, 100);
  text("Z: " + nf(qz, 1, 4), 25, 205);

  fill(255);
  textSize(14);
  text("Serial Port: COM3", 25, height - 35);

  hint(ENABLE_DEPTH_TEST);
}
