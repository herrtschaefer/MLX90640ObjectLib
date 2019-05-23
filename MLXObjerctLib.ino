#include "MLX90640.h"  
#define MLX_SDA 18
#define MLX_SCL 19
#define TA_SHIFT 8
MLX90640 sensor(MLX_SDA,MLX_SCL);

void setup(){
    Serial.begin(115200);
    sensor.begin(-1);
    sensor.setTASHIFT(TA_SHIFT);
  }
void loop() {
    sensor.getTemp();
    Serial.println(sensor.Data[0],2);
    Serial.println(sensor.Data[10],2);
    Serial.println(sensor.Data[100],2);
    Serial.println(sensor.Data[600],2);

    Serial.println();
  }




































////BOARD NODE-MCU32S
//#define MLX_SDA 18
//#define MLX_SCL 19
//#define TA_SHIFT 8  //Used just in this file currently ->Move into Object
//// Default shift for MLX90640 in open air
//const byte address = 0x33;          // Default 7-bit unshifted address of the MLX90640
//static float Data[768];       // Sensor Data
//parameters mlx90640;            // Parameter Struct
//
///*      Function_Dummys
// * For description see Bottom
// */
//boolean isConnected();               
//void    sensorInitialize();         
//void    serialInitialize(int baud);
//
//
//void setup() {
//  serialInitialize(115200); // start Serial communication
//  sensorInitialize();       // Initialize sensor and extract parameters
//  
//}
//
//void loop() {
//
//  getFrame();
//
//}
//
//void sensorInitialize() {           // starts communication and extract Parameters 
//
//  Wire.begin(MLX_SDA, MLX_SCL);   // Sensor Bus (SDA , SCL)
//
//  // Connect to sensor
//  if (isConnected() == false)
//  {
//    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
//    while (1);
//  }
//  Serial.println("MLX90640 online!");
//
//  //Get device parameters - We only have to do this once
//  int status = 1 ;
//  uint16_t eeMLX90640[832];
//  while (status != 0) {                     //Added
//    status = sensor.DumpEE(address, eeMLX90640);
//    if (status != 0)
//      Serial.println("Failed to load system parameters");
//
//    status = sensor.ExtractParameters(eeMLX90640, &mlx90640);
//    if (status != 0)
//      Serial.println("Parameter extraction failed");
//
//    //Once params are extracted, we can release eeMLX90640 array
//  }
//}
//void getFrame() {                   // stores Frame in Data
//  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
//  {
//    uint16_t mlx90640Frame[834];
//    int status = sensor.GetFrameData(address, mlx90640Frame);
//    if (status < 0)
//    {
//      Serial.print("GetFrame Error: ");
//      Serial.println(status);
//    }
//
//    float vdd = sensor.GetVdd(mlx90640Frame, &mlx90640);
//    float Ta = sensor.GetTa(mlx90640Frame, &mlx90640);
//
//    float tr = Ta - TA_SHIFT; //Reflected Dataerature based on the sensor ambient Dataerature
//    float emissivity = 0.95;
//
//    sensor.CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, Data);
//  }
//}
//boolean isConnected(){              // true if it detects Sensor on I2C Bus
//  Wire.beginTransmission((uint8_t)address);
//  if (Wire.endTransmission() != 0)
//    return (false); //Sensor did not ACK
//  return (true);
//}
//void serialInitialize(int baud){    // start Serial communication
//  Serial.begin(baud);
//  while (!Serial); //Wait for user to open terminal
//  Serial.println("MLX90640 IR Array Test");
//
//}
