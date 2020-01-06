
// Define Right Sensor Pins Here
#define ProximitySensor 5      
#define HallSensor 1
#define MagPin 5    // MOSFET Pin Controlling Electromagnet

#define ProximitySamples 20   // Define Samples for Average Filter
#define HallSamples 50

int ProximityRaw;
int HallRaw;

int HallFeedback;            
int ProximityFeedback;

int Threshold;

bool MagFlag;                 // Flag for gripper state

void setup()
{
  
  Serial.begin (115200);
  pinMode (ProximitySensor, OUTPUT) ;
  pinMode (HallSensor, OUTPUT) ;
  pinMode (MagPin, OUTPUT);
  digitalWrite(MagPin, LOW);
}

void DebugWink()
{ 
  // Blinks LED - Used for Debuging Only
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(1000);
  }
}

void ResetHallThreshold()
{
  Threshold = 0;
}

void HallCalibrate()
// Calibrates Hall Sensor
{
  
  int HallAvg = 0;
  int SensorVal;
 
  for (int i = 0; i < HallSamples; i++)
  {
    SensorVal = analogRead(HallSensor);
    HallAvg = HallAvg + SensorVal;
  }

  Threshold = abs(HallAvg / HallSamples);
  
}


void EstimateFeedback()
{
  int PRaw, HRaw;
  
  ProximityRaw = 0;
  HallRaw = 0;


  //--------------------------PROXIMITY--------------------------------/
  for (int p = 0; p < ProximitySamples; p++)
  {
    PRaw = analogRead(ProximitySensor);
    ProximityRaw = ProximityRaw + PRaw;
  }
  ProximityRaw = abs(ProximityRaw / ProximitySamples);  // Average Samples
  
  //---------------------------HALL-----------------------------------/
  for (int h = 0; h < HallSamples; h++)
  {
    HRaw = analogRead(HallSensor);
    HallRaw = HallRaw + HRaw;
  }
  HallRaw = abs(HallRaw / HallSamples);

  // Feedback
  //------------------------------------------------------------------/
  if (ProximityRaw < 500)
  {
    ProximityFeedback = 1;
  }
  else
  {
    ProximityFeedback = 0;
  }
  //------------------------------------------------------------------/
  if (HallRaw < Threshold)
  {
    HallFeedback = 1;
  }

  else
  {
    HallFeedback = 0;
  }


}


void GripperProtocol(int ProxRaw, int HallRaw, int ProxFeedback, int HallFeedback, bool TotalFeedback, bool Flag)
{

  uint8_t crc = (uint8_t) 'b';
  uint8_t payload_size = 7;

  byte bytes[6];

  bytes[0] = ProxRaw;        // Raw Prox Data
  bytes[1] = HallRaw;        // Raw Hall Data

  bytes[2] = ProxFeedback;   // Feedback Estimate from Inductive Proximity Sensor
  bytes[3] = HallFeedback;   // Feedback Estimate from Hall Sensor
  
  bytes[4] = TotalFeedback;  // Total Feedback Estimate

  bytes[5] = Flag;           // Magnet State  

  bytes[6] = 0x23;           // EndMessage
    
  Serial.write('b');
  Serial.write(payload_size + 1); // Add a byte for Message ID

  // Write Data on Serial
  Serial.write(0x43);        // Message ID "32" - Used to distinguish msg from gripper vs. other instruments that use same protocol
  Serial.write(bytes[0]);    // Raw Feedback Proxmity Sensor
  Serial.write(bytes[1]);    // Raw Feedback Hall Sensor
  Serial.write(bytes[2]);    // Estimated Feedback Proximity Sensor
  Serial.write(bytes[3]);    // Estimated Feedback Hall Sensor
  Serial.write(bytes[4]);    // Total Estimated Feedback
  Serial.write(bytes[5]);    // Magnet State
  Serial.write(bytes[6]);    
  
  crc += 0x43 + payload_size + 1 + (uint8_t) bytes[0] + (uint8_t) bytes[1] + (uint8_t) bytes[2] + (uint8_t) bytes[3] + (uint8_t) bytes[4] + (uint8_t) bytes[5] + (uint8_t) bytes[6];
  Serial.write(crc);

  // Debug Msg
  //Serial.println((String)"b"+payload_size+bytes[0]+bytes[1]+bytes[2]+bytes[3]+bytes[4]+bytes[5]+bytes[6]+crc);
}

uint8_t Ash_Read_Single()
{
  if (Serial.available() > 3)
  {

    uint8_t crc = 0;
    uint8_t tmp_in;
    uint8_t cmd;
    
    tmp_in = Serial.read();

    // Message Start
    if (tmp_in == 'b')
    {
      crc += tmp_in;
      tmp_in = Serial.read();
      crc += tmp_in;
      
      tmp_in = Serial.read();
      cmd = tmp_in;

      crc += tmp_in;
      if (crc == Serial.read())
      {
        return cmd;
      }
    }
  }
  return 255;
}

void Switch_Mags(bool state)
// Function to operate the MOSFET Controlling Electromagnet
{
  
  if (state == true)
  {
    digitalWrite(MagPin, HIGH); // Turning on Electromagnet
    delay(10);
    MagFlag = true;
  }

  if (state == false)
  {
    digitalWrite(MagPin, LOW); // Turning off Electromagnet
    delay(10);
    MagFlag = false;
  }
}

void loop()
{
  // Read from Serial
  uint8_t mag_state = Ash_Read_Single(); 

  // Function to Toggle Magnets Based on Incoming Message
  if (mag_state == 0x40)
  {
    Switch_Mags(true); 
    
    HallCalibrate();
    delay(50);
    HallCalibrate();
   
  }
  else if (mag_state == 0x41)
  {
    Switch_Mags(false);
    
    HallCalibrate();
    delay(5);
    HallCalibrate();
    delay(5);
  }


  EstimateFeedback();
  GripperProtocol(ProximityRaw, HallRaw, ProximityFeedback, HallFeedback, (ProximityFeedback || HallFeedback), MagFlag);

}
