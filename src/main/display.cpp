// #include <Adafruit_GFX.h>
// #include <Adafruit_SSD1306.h>
// //BEGIN DISPLAY==================
// #define SCREEN_WIDTH 128 // OLED display width, in pixels
// #define SCREEN_HEIGHT 64 // OLED display height, in pixels
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// #define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// void setup_display() {
//   if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
//     Serial.println(F("SSD1306 allocation failed"));
//     for(;;); // Don't proceed, loop forever
//   }
// }

// void drawStr(int line, const char* name, int t) {
//   char buf[100];

//   display.setCursor(0, line*10);
//   display.write( name );
//   display.setCursor(50,line*10);
//   if (t) display.write( t>0?"+":"-" );
//   display.setCursor(70,line*10);
//   display.write( itoa(t,buf,10) );
// }
//END

// void oled_display() {
//   display.clearDisplay();
//   display.setTextSize(1);      // Normal 1:1 pixel scale
//   display.setTextColor(WHITE); // Draw white text
//   display.setCursor(0, 0);     // Start at top-left corner
//   display.cp437(true);  

//   display.setCursor(20,0);
//   display.write("Balanced Wheel");
 
//   display.setTextColor(WHITE); // Draw white text

//   drawStr(1, "Left_V:", Encoder_Value_Left); 		
//   drawStr(2, "Right_V:", Encoder_Value_Right); 		
//   drawStr(3, "PWM      :", Moto1_PWM); 	
//   drawStr(4, "B_PWM      :", Balance_Pwm); 	
//   drawStr(5, "Angle_B:", Angle_Balance);
//   drawStr(6, "Gyro_B:", Gyro_Balance);
//   display.display();
// }