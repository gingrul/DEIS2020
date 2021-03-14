/***********************************************************************

***********************************************************************/

#include <RedBot.h>

RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
RedBotAccel accelerometer;


//int data;  // variable for holding incoming data from PC to Arduino
//int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

// variable used to store the time
unsigned long myTime;

const byte numChars = 20;
char receivedChars[numChars];   // an array to store the received data
char tempChars[numChars];        // temporary array for use when parsing


int leftPower;  // variable for setting the drive power
int rightPower; 

// variables used to store the left and right encoder counts.
int lCount;
int rCount;


boolean newData = false;

void setup(void)
{
    Serial.begin(115200); 
    // Serial.print("Enter in left and right motor power values and click [Send]."); 
    // Serial.print("Separate values with a space or non-numeric character.");
    // Serial.println();
    // Serial.print("Positive values spin the motor CW, and negative values spin the motor CCW.");
    encoder.clearEnc(BOTH);  // Reset the counters.
    
}

void loop(void)
{
    //encoder.clearEnc(BOTH);  // Reset the counters.

    
    recvWithEndMarker();
    
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        myTime=millis();
        motors.leftMotor(leftPower);
        motors.rightMotor(rightPower);
        lCount = encoder.getTicks(LEFT);    // read the left motor encoder
        rCount = encoder.getTicks(RIGHT);   // read the right motor encoder
        
        showParsedData();
        
        newData = false;
    }
//    if(Serial.available() > 0)
//    {
//        //encoder.clearEnc(BOTH);
//        leftPower = Serial.parseInt();  // read in the next numeric value
//        leftPower = constrain(leftPower, -255, 255);  // constrain the data to -255 to +255
//        
//        rightPower = Serial.parseInt();   // read in the next numeric value
//        rightPower = constrain(rightPower, -255, 255);  // constrain the data to -255 to +255
//
//        motors.leftMotor(leftPower);
//
//        motors.rightMotor(rightPower);
//
//        // short delay in between readings/
//        delay(100); 
//
//        // store the encoder counts to a variable.
//        lCount = encoder.getTicks(LEFT);    // read the left motor encoder
//        rCount = encoder.getTicks(RIGHT);   // read the right motor encoder
//        
//        Serial.print(myTime);  // tab
//        Serial.print("_");  // tab
//        Serial.print(leftPower);  // tab
//        Serial.print("_");  // tab
//        Serial.print(rightPower);  // tab
//        Serial.print("_");  // tab
//        Serial.print(lCount);  // tab
//        Serial.print("_");  // tab
//        Serial.println(rCount);  // tab
//        //Serial.print(" ");  // tab
//        Serial.flush(); 
//  
//        int inByte=Serial.read(); //to empty the serial and make it wait until new data
//    } 
//    else
//    {        
//        motors.brake();
//    }
//     
}


// if there is data coming in on the Serial monitor, wait until it received fully   
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
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
        }
    }
}


void showNewData() {
    if (newData == true) {
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        newData = false;
    }
}

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars," ");      // get the first part - the string
    leftPower = atoi(strtokIndx);  // convert this part to an integer
 
    strtokIndx = strtok(NULL, " "); // this continues where the previous call left off
    rightPower= atoi(strtokIndx);     // convert this part to an integer

}

void showParsedData() {
    Serial.print(myTime);
    Serial.print("_");
    Serial.print(leftPower);
    Serial.print("_");
    Serial.print(rightPower);
    Serial.print("_"); 
    Serial.print(lCount); 
    Serial.print("_");  // 
    Serial.println(rCount);  // 
        //Serial.print(" ");  // 
   
}
