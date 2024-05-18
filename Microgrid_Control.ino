////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Auburn ELEC 4020 Capstone Design Project Team 12 - DC Microgrid Demonstration
// Author: William McGinnis
// Team Members: Wynn Baker, Justin Daugherty, Tristan Heffner, Luca Giordano, Jonathan Leviner, William McGinnis
// ** Portions of this code adapted from Solarduino's "DC VOLTMETER" and Robin2's Serial Input Basics (Arduino Forum) **
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////* Global Variables *//////////////////////////////////////////////////////
/* 0 - Microgrid Constant Values */
    float vThreshBattery = 12;  //Threshold voltage for battery failsafe
    float Rload = 330;         //Resedential Load Resistance in ohms
/* 1 - DC Voltage Measurement */

    float vArduino1, vArduino2, vArduino3;                // Voltage Arduino sees between 0 - 5V
    float v1Grid, v2Solar, v3Battery;                     // Actual Measured Voltage from Microgrid
    float R1 = 30000.0;                                   // R1 in Voltage Sensor
    float R2 = 7500.0;                                    // R2 in Voltage Sensor
    int rawValueRead1, rawValueRead2, rawValueRead3 = 0;     // Raw analog value read by the arduino (range 0 - 1023) 

/* 2 - LCD Display  */

    #include<LiquidCrystal.h>                   
    LiquidCrystal LCD(8,9,4,5,6,7);             
    unsigned long startMillisLCD;               // start counting time for LCD Display 
    unsigned long currentMillisLCD;             // current counting time for LCD Display 
    const unsigned long periodLCD = 1000;       // LCD refresh rate (ms)
    int displayMode = 0;                        //toggles between different LCD displays
    int displayCounter = 0;                     //secret display counter
    byte customChar0[8] = {
	    0b00011,
      0b00110,
      0b01100,
      0b11111,
      0b00011,
      0b00110,
      0b01100,
      0b00000
      };

/* 3 - LCD Buttons  */
    int buttonRead;                                             //stores value from  analog pin 0 for LCD button value 
    static unsigned long lastButtonPress = 0;                   //keeps track of last button press for debouncing purposes

/* 4 - Serial Read */
    const byte numChars = 32;
    char receivedChars[numChars];   // an array to store the received data
    String rcvString;               // received char array converted to String
    boolean newData = false;


//////////////////////////////////////////////////////* Setup *//////////////////////////////////////////////////////

void setup() {
    voltmeterSetup();
    lcdSetup();
    Serial.begin(9600);   
    dioSetup();
}

void voltmeterSetup() {
    pinMode(A1, INPUT);             /* Declare analog pin 1 as an input*/
    pinMode(A2, INPUT);             /* Declare analog pin 2 as an input*/
    pinMode(A3, INPUT);             /* Declare analog pin 3 as an input*/
}

void lcdSetup() {
    LCD.begin(16,2);                 //Tell Arduino that LCD has 16 columns and 2 rows
    LCD.createChar(0, customChar0);   //Create custom lightning bolt character
    LCD.setCursor(0,0);              //Set LCD to upper left corner of display
    startMillisLCD = millis();       //Initialize millisecond counter for display refresh rate
}

void dioSetup() {
    pinMode(2, OUTPUT);                          
    pinMode(11, OUTPUT);              
    pinMode(12, OUTPUT);              
    pinMode(13, OUTPUT);              
}

//////////////////////////////////////////////////////* Main Loop *//////////////////////////////////////////////////////

void loop() {
  controlDIO();
  voltmeter();
  buttonDetect();
  lcdDisplay();
}

//////////////////////////////////////////////////////* Functions *//////////////////////////////////////////////////////

void voltmeter() {
  /* DC Grid Voltage Measurement (A1)*/
    
    rawValueRead1 = analogRead(A1);        /* Read and collect sensor from analog input pin in raw data (0-1023) values */
    vArduino1 = (rawValueRead1 * 5.0) / 1024.0;         /* Convert the raw data value into 0 - 5V measurable voltage at analog input pin*/
    v1Grid = (vArduino1 / (R2/(R1+R2))) * 1.015;      /* Calculate the expected monitoring voltage in full voltage*/

    /* DC Solar Voltage Measurement (A2)*/
    
    rawValueRead2 = analogRead(A2);        
    vArduino2 = (rawValueRead2 * 5.0) / 1024.0;         
    v2Solar = (vArduino2 / (R2/(R1+R2))) * 1.015;      
    
    /* DC Battery Voltage Measurement (A3)*/
    
    rawValueRead3 = analogRead(A3);       
    vArduino3 = (rawValueRead3 * 5.0) / 1024.0;         
    v3Battery = (vArduino3 / (R2/(R1+R2))) * 1.015;    
    if (v3Battery < vThreshBattery) {
      digitalWrite(11, HIGH);       //Battery failsafe, if vBattery < 12V, turn on grid
      digitalWrite(12, LOW);        //turn off solar
    }   

}

void buttonDetect() {
    buttonRead = analogRead (0);                                //analog value from A0
    if (millis() - lastButtonPress > 200) {                     //debounce 

      /*Right button is pressed */
      if (buttonRead < 60) {
      displayMode++;                                            //next display

        if(displayMode > 2) {                                   //roll over displayMode counter from 2 to 0 
          displayMode = 0;
        }
        lastButtonPress = millis();                             //new most recent button press
      }     

      /* Up button is pressed */
      else if (buttonRead < 200) 
      {   LCD.setCursor(0,0); LCD.print ("Press Left/Right");   //Invalid button press
          LCD.setCursor(0,1); LCD.print ("                ");
          delay(2000);                                          //leave invalid button display up for 2 seconds
      } 

      /* Down button is pressed */
      else if (buttonRead < 400)
    {   LCD.setCursor(0,0); LCD.print ("Press Left/Right");     
        LCD.setCursor(0,1); LCD.print ("                ");
        delay(2000);                                            
    }    

      /* Left button is pressed */
      else if (buttonRead < 600)
      {   
        displayMode--;                                           //previous display
          if(displayMode < 0) {                                  //roll over display from 0 to 3
            displayMode = 3;
        }
        lastButtonPress = millis();

      }            
      /* Select button is pressed */
      else if (buttonRead < 800)
      {   
        LCD.setCursor(0,0); LCD.print ("Press Left/Right");   
        LCD.setCursor(0,1); LCD.print ("                ");
        delay(2000);
      }    
    }
}

void lcdDisplay(){
    currentMillisLCD = millis();  
    if (currentMillisLCD - startMillisLCD >= periodLCD)
      {
        LCD.setCursor(10,1);   //Lightning Bolt characters 
        LCD.write((byte)0);
        LCD.setCursor(12,1);
        LCD.write((byte)0);
        LCD.setCursor(14,1);
        LCD.write((byte)0);

        LCD.setCursor(0,0);   /* Set cursor to first colum 0 and second row 1  */
        switch(displayMode) {
          case 0:
            LCD.print("Grid =          "); 
            LCD.setCursor(0,1);
            LCD.print(v1Grid,4);
            LCD.setCursor(7,1);
            LCD.print(" V");
          break;
          
          case 1:
            LCD.print("Solar =         "); 
            LCD.setCursor(0,1);
            LCD.print(v2Solar, 4);
            LCD.setCursor(7,1);
            LCD.print(" V");
          break;
          
          case 2:
            LCD.print("Battery =       "); 
            LCD.setCursor(0,1);
            LCD.print(v3Battery, 4);
            LCD.setCursor(7,1);
            LCD.print(" V");
          break;
        }
        startMillisLCD = currentMillisLCD ;                                    /* Set the starting point again for next counting time */
      }
}

void controlDIO() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    

    /******************** Receive Serial Input ******************************/
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
            int a_size = sizeof(receivedChars) / sizeof(char);  // convert receivedChars to String
            rcvString = convertToString(receivedChars, a_size);
        }
    }

    /******************** Execute Command **********************************/
    if (newData == true){
     if (rcvString == "toggle solar") {
          if (digitalRead(12) == 1) {   //If grid is on, turn it off to turn solar on (can't have them both on concurrently)
            digitalWrite(12, LOW);
            Serial.println(String("Grid(DIO12) = ") + String(digitalRead(12)));
          }
          digitalWrite(11, !digitalRead(11));  //toggle DIO 11 (Solar)
          Serial.println(String("Solar (DIO 11) = ") + String(digitalRead(11)));
          
          Serial.println();
          Serial.println("*********************************************************************");
          Serial.println();
          newData = false;
      }
      else if (rcvString == "toggle grid") {
          if (digitalRead(11) == 1) { 
            digitalWrite(11, LOW);
            Serial.println(String("Solar (DIO 11) = ") + String(digitalRead(11)));
          }
          digitalWrite(12, !digitalRead(12));  //toggle DIO 12 (Grid)
          Serial.println(String("Grid(DIO 12) = ") + String(digitalRead(12)));

          Serial.println();
          Serial.println("*********************************************************************");
          Serial.println();
          newData = false;
      }
      
      else if (rcvString == "toggle fan") {
          digitalWrite(13, !digitalRead(13));  //toggle DIO 13 (Fan)
          Serial.println(String("Fan (DIO 13) = ") + String(digitalRead(13)));
          Serial.println();
          Serial.println("*********************************************************************");
          Serial.println();
          newData = false;
      }
      else if (rcvString == "status") {  
      
      //Solar Status (Read DIO 11)
        if(digitalRead(11) == 1){
          Serial.println("Solar ON");
        } else {
          Serial.println("Solar OFF");
        }
      //Grid Status (Read DIO 12)
        if(digitalRead(12) == 1){
          Serial.println("Grid ON");
        } else {
          Serial.println("Grid OFF");
        }
      //Load Shed Status (Read DIO 13)
        if(digitalRead(13) == 1){
          Serial.println("Sheddable Load ON");
        } else {
          Serial.println("Sheddable Load OFF");
        }
      //Current Voltage Readings 
      Serial.println(String("Grid Voltage = ") + String(v1Grid, 4) + String("V"));
      Serial.println(String("Solar Voltage = ") + String(v2Solar, 4) + String("V"));
      Serial.println(String("Battery Voltage = ") + String(v3Battery, 4) + String("V"));
      Serial.println();
      Serial.println("*********************************************************************");
      Serial.println();

      newData = false;
      }
    }
}

String convertToString(char* a, int size) {
    int i;
    String s = "";
    for (i = 0; i < size; i++) {
        s = s + a[i];
    }
    return s;
}
