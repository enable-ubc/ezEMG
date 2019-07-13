/* ezEMG switch detection and signal processing algorithm for IGEN 330
 *  
 *  March 22, 2019
 *  
 *  
 * 
 */

// Description of running average signal processing:
//
// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.

// Initializing variables:
// pin declarations
#define inputPin A0
#define buzzerOutput 4
#define ledOutput 7
#define switchPin 5
#define auxOutput 2

#define tipPin A6
#define ring1Pin A5
#define ring2Pin A4
#define sleevePin A3


// variables for checkN //// 
#define dialPin A7    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor 
float N;                // a constant multiplier to set trigger thresholds
////////////////////////////

// threshold variables/////
float low_thresh = 0;
float high_thresh = 0;
///////////////////////////

// time delay constants////
unsigned long previousMillis = 0;
const long printInterval = 15;
////////////////////////////

// variables for detection////
boolean detected = false;
int detectedCount = 0;
int detectedDelay = 50;
//////////////////////////////

// variables for smoothSignal /////////
const int sS_numReadings = 20;
int sS_readings[sS_numReadings];      // the readings from the analog input
int sS_readIndex = 0;                 // the index of the current reading
int sS_total = 0;                     // the running total
int average = 0;                      // the average signal
///////////////////////////////////////

// variables for standardDev /////////
const int sample_size = 50;     // the size of the sample to consider for standard deviation calc
const int sample_lag = 50;      // the standard deviation is calculated for the sample_size values that lag this much behind the current

const int totalReadings = sample_size + sample_lag;   // the total number of values in array for standardDev

int readings[totalReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading

float mean = 0;
float mean_total = 0 ;
float mean_readings[sample_size]; 
int mean_readIndex = 0;
unsigned long sd = 0;
float sd_total = 0;
float sd_readings[sample_size];
int sd_readIndex = 0;
//////////////////////////////////////

// variables for triggerOutput /////////
//const int stall_duration = 1000; 
const int outputDelay = 500;


///////////////////////////////////////

/* setup:
 *  runs once at the beginning to initialize the device settings
 */
void setup() {
  // define output pins and initialize off
  pinMode(buzzerOutput, OUTPUT);
  digitalWrite(buzzerOutput, LOW);
  pinMode(ledOutput, OUTPUT);
  digitalWrite(ledOutput, LOW);
  pinMode(auxOutput, OUTPUT);
  digitalWrite(auxOutput, LOW);
  pinMode(switchPin, INPUT);

  // define aux pins to inputs so that they never send any signal (due to messed up wiring)
  pinMode(tipPin, INPUT);
  pinMode(ring1Pin, INPUT);
  pinMode(ring2Pin, INPUT);
  pinMode(sleevePin, INPUT);

  // initialize serial communication with computer:
  Serial.begin(9600);
  
  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < sS_numReadings; thisReading++) {
    sS_readings[thisReading] = 0;
  }
  for (int thisReading = 0; thisReading < totalReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

/* loop:
 *  main loop that runs continuously
 *    smooth incoming signal using running average
 *    check potentiometer for sensitivity value
 *    calculate mean and standard deviation
 *    calculate high and low thresholds
 *    check if a signal is detected and output accordingly
 */
void loop(){
  smoothSignal();
  checkN();
  standardDev();
  calcThresholds();
  detect();
  
  unsigned long currentMillis = millis(); 
  if (currentMillis - previousMillis >= printInterval){
    previousMillis = currentMillis;
    printToScreen();
  }
}

void printToScreen(){
  Serial.print(average);
  Serial.print(",");
  Serial.print(low_thresh);
  Serial.print(",");
  Serial.print(high_thresh);
  Serial.print(",");
  Serial.println(mean);
}

/* detect:
 *  checks whether a signal was detected and prints the values 
 *  to the serial plotter for visual aid and debugging
 */

void detect(){
    if (!detected) {
      if(average > high_thresh || average < (low_thresh - 1)) {
        detected = true;
        chooseMode();
      }
    }
    else if (detectedCount < detectedDelay) {
        detectedCount++;
      }
      else {
        detected = false;
        detectedCount = 0;
        stopSignaling();
      }
}

/* stopSignaling:
 *  turns off all the components
 */
void stopSignaling(){
  digitalWrite(buzzerOutput, LOW);
  digitalWrite(ledOutput, LOW);
  digitalWrite(auxOutput, LOW);
}

/* calcThreholds:
 *  calculates the high and low thresholds used for detection
 *  uses mean, standard deviation and sensitivity (N)
 */
void calcThresholds(){
  low_thresh = mean - (N * (sd + 1));
  high_thresh = mean + (N * (sd + 1));
}

/* chooseMode:
 *  checks the switch to determine whether to output externally or internally only
 */
void chooseMode(){
  if (digitalRead(switchPin) == HIGH){
    externalOutput();
  }
  else {
    internalOutput();
  }
}

/* internalOutput:
 *  signals only the internal components; the buzzer and light
 */
void internalOutput(){
   //digitalWrite(buzzerOutput, HIGH);
   digitalWrite(ledOutput, HIGH);
}

/* externalOutput:
 *  signals all of the buzzer, light and aux output
 */
void externalOutput(){
  digitalWrite(buzzerOutput, HIGH);
  digitalWrite(ledOutput, HIGH);
  digitalWrite(buzzerOutput, LOW);
  digitalWrite(auxOutput, HIGH);
  analogRead(sleevePin);
  analogRead(ring2Pin);
  delay(3000);
  digitalWrite(auxOutput, LOW);
  analogRead(sleevePin);
  analogRead(ring2Pin);
}

/* smoothSignal: 
 *  smooths the signal by calculating a running average 
 */

void smoothSignal() {
  // subtract the last reading:
  sS_total = sS_total - sS_readings[sS_readIndex];
  // read from the sensor:
  sS_readings[sS_readIndex] = analogRead(inputPin);
  // add the reading to the total:
  sS_total = sS_total + sS_readings[sS_readIndex];
  // advance to the next position in the array:
  sS_readIndex = sS_readIndex + 1;

  // if we're at the end of the array...
  if (sS_readIndex >= sS_numReadings) {
    // ...wrap around to the beginning:
    sS_readIndex = 0;
  }

  // calculate the average:
  average = sS_total / sS_numReadings;
  // send it to the computer as ASCII digits
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  //float voltage = average * (5.0 / 1023.0);
}

/* checkN:
 *  checks the potentiometer to determine sensitivity of device.
 *  
 *  uses the value read to calculate a multiplier which scales the low and high thresholds accordingly
 */
void checkN(){ 
  sensorValue = analogRead(dialPin); 
  // makes max N value ****** and min 3.1
  N = (sensorValue / 200.0) + 0.5;
}

void standardDev() { 
  // store the current value of average:
  readings[readIndex] = average;
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= totalReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // sum a total for mean calc
  int j = mean_readIndex;
    for(int count=0; count < sample_size; count++) {

      if(j >= totalReadings) { 
          j=0;
        }
      
      // subtract the last reading:
      mean_total = mean_total - mean_readings[mean_readIndex];
      // read from the sensor:
      mean_readings[mean_readIndex] = readings[j];
      // add the reading to the total:
      mean_total = mean_total + mean_readings[mean_readIndex];
      // advance to the next position in the array:
      mean_readIndex = mean_readIndex + 1;
    
      if(mean_readIndex >= sample_size) { 
         mean_readIndex = 0;
      }
  
      j++;
    }
    
  // calc the mean
  mean = mean_total / sample_size;


  // sum a total for sd calc
  int i = readIndex;
  for(int count=0; count < sample_size; count++) {

    if(i >= totalReadings) { 
      i=0;
    }

    // subtract the last reading:
    sd_total = sd_total - sd_readings[sd_readIndex];
    // read from the sensor:
    sd_readings[sd_readIndex] = sq(abs(readings[i] - mean));
    // add the reading to the total:
    sd_total = sd_total + sd_readings[sd_readIndex];
    // advance to the next position in the array:
    sd_readIndex = sd_readIndex + 1;

    if(sd_readIndex >= sample_size) { 
      sd_readIndex=0;
    }

    i++;
  }
  
  // calc the sd
  sd = abs(sqrt( sd_total / sample_size ));

}
