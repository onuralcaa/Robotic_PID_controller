#include <Servo.h>

Servo sg90;
int pos = 0;

void setup() {
  sg90.attach(13);
}

void loop() {
  
  for (pos = 0; pos <= 180; pos += 1) {
    sg90.write(pos);
    delay(15);
  }

  for (pos = 180; pos >= 0; pos -= 1) {
    sg90.write(pos);
    delay(15);
  }



//sg90.write(90);


}
