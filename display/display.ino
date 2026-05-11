#include <LiquidCrystal_I2C.h>
#include <Wire.h>
// set the lcd address to 0x27 for a 16 chars and 2 line display.
LiquidCrystal_I2C lcd (0x27, 16, 2);
void setup() {
  // put your setup code here, to run once:
lcd.init(); // initialize the LCD
lcd.backlight(); //turn on the backlight and print a message.

}

void loop() {
  // put your main code here, to run repeatedly:
lcd.setCursor (1,0);
lcd.print("Hello Khanal");
delay(3000);
lcd.clear();

lcd.setCursor(1,0);
lcd.print("I want your");
lcd.setCursor(2,1);
lcd.print("Black ass ");
delay(3000);
lcd.clear();

lcd.setCursor(1,0);
lcd.print("Your's Daddy");
delay(3000);
lcd.clear();
}
