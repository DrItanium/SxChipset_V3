// This program is responsible for parsing commands from the teensy serial connection and turn that into graphics operations
PGraphics pg;
void setup() {
  size(320, 240);
  pg = createGraphics(320, 240);
}

enum DrawingOperation {
  Error('\0', '\0'),
    DrawCircle('D', 'C'),
    FillCircle('F', 'C'),
    DrawTriangle('D', 'T'),
    FillTriangle('F', 'T');
  private final char lo;
  private final char hi;
  DrawingOperation(char a, char b) {
    lo = a;
    hi = b;
  }
};

void draw() {
  pg.beginDraw();
  pg.endDraw();
}
