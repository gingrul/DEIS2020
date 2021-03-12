//testing of motors' function
#include <RedBot.h>

RedBotMotors motors;

void setup() {
  // put your setup code here, to run once:
   motors.drive(250);
   delay(2000);
   motors.stop();
}

void loop() {
  // put your main code here, to run repeatedly:

}
