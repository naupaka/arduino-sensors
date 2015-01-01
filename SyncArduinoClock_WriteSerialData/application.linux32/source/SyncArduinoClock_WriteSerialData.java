import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import java.util.Date; 
import java.util.Calendar; 
import java.util.GregorianCalendar; 
import java.text.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class SyncArduinoClock_WriteSerialData extends PApplet {

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
 
// This processing sketch was originally downloaded from: 
// http://playground.arduino.cc/Code/Time
// on December 29, 2014
// Modified by Naupaka Zimmerman naupaka@gmail.com
// December 30, 2014



// Added these which were missing from the original processing sketch





// For writing to a file
PrintWriter output;
// TimeZone tz;
String dataFolder;

public static final short portIndex = 0;  // select the com port, 0 is the first port
public static final char TIME_HEADER = 'T'; //header byte for arduino serial time message 
public static final char TIME_REQUEST = 7;  // ASCII bell character 
public static final char LF = 10;     // ASCII linefeed
public static final char CR = 13;     // ASCII linefeed
Serial myPort;     // Create object from Serial class

public void setup() {  

  // Choose output file name
  // thanks to example here 
  // http://www.mooduino.co.uk/2010/03/file-output-with-processing.html
  // tz = TimeZone.getDefault();
  DateFormat dfm = new SimpleDateFormat("yyyyMMddHHmm");
  // dfm.setTimeZone(tz);
  String fileName = "/home/pi/greenhouse_data/sensor_data_files/" + dfm.format(new Date()) + "_sensor_data.csv";
  output = createWriter(fileName);
  
  // print to terminal
  // println(Serial.list());
  // println(" Connecting to -> " + Serial.list()[portIndex]);
  
  // write port to output file
  // output.println(" Connecting to -> " + Serial.list()[portIndex]);
  // output.flush();
  
  myPort = new Serial(this,Serial.list()[portIndex], 9600);
}

public void draw()
{
  if ( myPort.available() > 0) {  // If data is available,
    char val = PApplet.parseChar(myPort.read());         // read it and store it in val
    if(val == TIME_REQUEST){
       long t = getTimeNow();
       sendTimeMessage(TIME_HEADER, t);   
    }
    else
    { 
       if(val == LF)
           ; //igonore
       else if(val == CR) {           
         // println();
         output.println();
         output.flush(); // Writes the remaining data to the file
         // output.close(); // Finishes the file, need to reopen again for more use 
       }
       else  {
         // print(val); // echo everying but time request
         output.print(val); // echo everying but time request
       }
    }
  } 
}

// void mousePressed() {  
//  sendTimeMessage( TIME_HEADER, getTimeNow());   
// }


public void sendTimeMessage(char header, long time) {  
  String timeStr = String.valueOf(time);  
  myPort.write(header);  // send header and time to arduino
  myPort.write(timeStr);   
}

public long getTimeNow(){
  // java time is in ms, we want secs    
  GregorianCalendar cal = new GregorianCalendar();
  cal.setTime(new Date());
  int	tzo = cal.get(Calendar.ZONE_OFFSET);
  int	dst = cal.get(Calendar.DST_OFFSET);
  long now = (cal.getTimeInMillis() / 1000) ; 
  now = now + (tzo/1000) + (dst/1000); 
  return now;
}
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "SyncArduinoClock_WriteSerialData" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
