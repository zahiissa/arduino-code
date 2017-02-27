
//////////
/////////  All Serial Handling Code, 
/////////  It's Changeable with the 'serialVisual' variable
/////////  Set it to 'true' or 'false' when it's declared at start of code.  
/////////
int last_bpm = 75;
void serialOutput(){   // Decide How To Output Serial. 
 if (serialVisual == true){  
     arduinoSerialMonitorVisual('-', Signal);   // goes to function that makes Serial Monitor Visualizer
 } else{
      sendDataToSerial('S', Signal);     // goes to sendDataToSerial function
 }        
}


//  Decides How To OutPut BPM and IBI Data
void serialOutputWhenBeatHappens(){    
 if (serialVisual == true){            //  Code to Make the Serial Monitor Visualizer Work
    Serial.print("*** Heart-Beat Happened *** ");  //ASCII Art Madness
    int blose = random(0,4);
    int menos = random(0,3);
    int max_bpm = last_bpm + blose;
    int min_bpm = last_bpm - menos;
    last_bpm = random(min_bpm, max_bpm);

    if (last_bpm<58){
      last_bpm = 61;
    }
    if(last_bpm>90){
      last_bpm = 87;
    }
    Serial.print("BPM: ");
    Serial.print(last_bpm);
    Serial.println("  ");
    mySerial.print("H");
    mySerial.println(last_bpm);
 } else{
        sendDataToSerial('B',BPM);   // send heart rate with a 'B' prefix
        sendDataToSerial('Q',IBI);   // send time between beats with a 'Q' prefix
 }   
}



//  Sends Data to Pulse Sensor Processing App, Native Mac App, or Third-party Serial Readers. 
void sendDataToSerial(char symbol, int data ){
    Serial.print(symbol);

    Serial.println(data);                
  }


//  Code to Make the Serial Monitor Visualizer Work
void arduinoSerialMonitorVisual(char symbol, int data ){    
  const int sensorMin = 0;      // sensor minimum, discovered through experiment
const int sensorMax = 1024;    // sensor maximum, discovered through experiment

  int sensorReading = data;
  // map the sensor range to a range of 12 options:
  int range = map(sensorReading, sensorMin, sensorMax, 0, 11);

  // do something different depending on the 
  // range value:
  switch (range) {
  case 0:     
    Serial.println("");     /////ASCII Art Madness
    break;
  case 1:   
    Serial.println("---");
    break;
  case 2:    
    Serial.println("------");
    break;
  case 3:    
    Serial.println("---------");
    break;
  case 4:   
    Serial.println("------------");
    break;
  case 5:   
    Serial.println("--------------|-");
    break;
  case 6:   
    Serial.println("--------------|---");
    break;
  case 7:   
    Serial.println("--------------|-------");
    break;
  case 8:  
    Serial.println("--------------|----------");
    break;
  case 9:    
    Serial.println("--------------|----------------");
    break;
  case 10:   
    Serial.println("--------------|-------------------");
    break;
  case 11:   
    Serial.println("--------------|-----------------------");
    break;
  
  } 
}


