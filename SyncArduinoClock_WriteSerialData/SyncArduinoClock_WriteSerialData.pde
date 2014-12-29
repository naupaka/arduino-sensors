/**
 * SyncArduinoClock. 
 *
 * portIndex must be set to the port connected to the Arduino
 * 
 * The current time is sent in response to request message from Arduino 
 * or by clicking the display window 
 *
 * The time message is 11 ASCII text characters; a header (the letter 'T')
 * followed by the ten digit system time (unix time)
 */
 

import processing.serial.*;
import java.util.Date;
import java.util.Calendar;
import java.util.GregorianCalendar;

PrintWriter output;

public static final short portIndex = 5;  // select the com port, 0 is the first port
public static final char TIME_HEADER = 'T'; //header byte for arduino serial time message 
public static final char TIME_REQUEST = 7;  // ASCII bell character 
public static final char LF = 10;     // ASCII linefeed
public static final char CR = 13;     // ASCII linefeed
Serial myPort;     // Create object from Serial class

void setup() {  
  output = createWriter("sensor_data.txt"); 
  
  size(200, 200);
  println(Serial.list());
  println(" Connecting to -> " + Serial.list()[portIndex]);
  output.println(" Connecting to -> " + Serial.list()[portIndex]);
  myPort = new Serial(this,Serial.list()[portIndex], 9600);
  
  output.flush();
}

void draw()
{
  if ( myPort.available() > 0) {  // If data is available,
    char val = char(myPort.read());         // read it and store it in val
    if(val == TIME_REQUEST){
       long t = getTimeNow();
       sendTimeMessage(TIME_HEADER, t);   
    }
    else
    { 
       if(val == LF)
           ; //igonore
       else if(val == CR) {           
         println();
         output.println();
         output.flush(); // Writes the remaining data to the file
         // output.close(); // Finishes the file 
       }
       else  {
         print(val); // echo everying but time request
         output.print(val); // echo everying but time request
         // output.flush(); // Writes the remaining data to the file
       }
    }
  } 
}

void mousePressed() {  
  sendTimeMessage( TIME_HEADER, getTimeNow());   
}


void sendTimeMessage(char header, long time) {  
  String timeStr = String.valueOf(time);  
  myPort.write(header);  // send header and time to arduino
  myPort.write(timeStr);   
}

long getTimeNow(){
  // java time is in ms, we want secs    
  GregorianCalendar cal = new GregorianCalendar();
  cal.setTime(new Date());
  int	tzo = cal.get(Calendar.ZONE_OFFSET);
  int	dst = cal.get(Calendar.DST_OFFSET);
  long now = (cal.getTimeInMillis() / 1000) ; 
  now = now + (tzo/1000) + (dst/1000); 
  return now;
}
