#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Arduino.h>
#include <SparkFunLSM6DSO.h>

#define SWITCH_PIN 17      // Pin for switch
#define STEP_THRESHOLD 1.2 // Adjust based on calibration
#define STEP_DELAY 200     // Time delay for step detection in milliseconds
#define SAMPLE_COUNT 50    // Number of samples to use for intensity calculation
#define SAMPLE_DELAY_MS 20 // Delay between each sample for intensity calculation

const char *ssid = "NeelPhone";
const char *password = "test1111";

const char *AWS_IP = "44.207.6.203"; // Server IP
const char *serverPath = "/";
const int serverPort = 5000;

// Step counter variables
int stepCount = 0;
unsigned long lastStepTime = 0;

// Intensity calculation variables
float ax[SAMPLE_COUNT], ay[SAMPLE_COUNT], az[SAMPLE_COUNT];
float gx[SAMPLE_COUNT], gy[SAMPLE_COUNT], gz[SAMPLE_COUNT];

// Accelerometer and Gyroscope object
LSM6DSO myIMU;

// Button state variables
bool intensityMeasuring = false;
// unsigned long lastButtonPress = 0;
unsigned long debounceDelay = 300; // debounce delay in milliseconds
// unsigned long prevButtonState = 0;
// Function declarations
float calculateRMS(float x[], float y[], float z[]);
String getIntensityLevel(float intensity);

void sendDataToAWS(int stepCount, float intensity)
{
  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());
  if (WiFi.status() == WL_CONNECTED)
  { // Check Wi-Fi connection
    HTTPClient http;
    Serial.print("CONNECTED TO CLOUD");

    // Construct the URL
    String serverURL = "http://" + String(AWS_IP) + ":" + String(serverPort) + serverPath;
    String payload = "{\"steps\":" + String(stepCount) + ",\"intensity\":" + String(intensity) + "}";

    // Configure HTTP request
    http.begin(serverURL.c_str());
    http.addHeader("Content-Type", "application/json"); // Specify content type as JSON

    // Send HTTP POST request
    int httpResponseCode = http.POST(payload);

    if (httpResponseCode > 0)
    {
      // Success
      String response = http.getString();
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      Serial.print("Server response: ");
      Serial.println(response);
    }
    else
    {
      // Failed to send data
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // End HTTP connection
  }
  else
  {
    Serial.println("WiFi Not Connected.");
  }
  // Serial.print("CONNECTION FAILED!!!!!!!!!!!!!!!!!");
  // if (WiFi.status() == WL_CONNECTED)
  // {
  //   Serial.println("WiFi Connected.");
  // }
  // else
  // {
  //   Serial.println("WiFi Not Connected.");
  // }
}

void setup()
{
  // Serial setup
  Serial.begin(9600);
  Wire.begin(21, 22);
  WiFi.begin(ssid, password);
  // Initialize button
  pinMode(SWITCH_PIN, INPUT); // Assuming the button is wired to GND
  // digitalWrite(SWITCH_PIN, LOW);     // Ensure button is not activated initially

  Serial.print("WiFi status: ");
  Serial.println(WiFi.status());

  // Initialize the IMU
  if (myIMU.begin())
  {
    Serial.println("IMU Ready.");
  }
  else
  {
    Serial.println("IMU Initialization Failed.");
    while (1)
      ;
  }

  myIMU.initialize(BASIC_SETTINGS);
}

void loop()
{
  // Read button state (with debounce)
  int switchState = digitalRead(SWITCH_PIN);
  // Serial.print("button state = ");
  // Serial.println(buttonState);
  if (switchState == HIGH)
  { // if (low == pressed) and in time
    // lastButtonPress = millis();
    // prevButtonState = buttonState;

    intensityMeasuring = true; // Toggle intensity measuring to True
    // Serial.println("Workout intensity measuring started.");
  }

  else
  {
    // Serial.println("Workout intensity not being measured.");
    // delay(500);
    intensityMeasuring = false; //Toggle intensity measuring to False

  }

    // Read accelerometer data
  float accelX = myIMU.readFloatAccelX();
  float accelY = myIMU.readFloatAccelY();
  float accelZ = myIMU.readFloatAccelZ();

  // Read gyroscope data
  float gyroX = myIMU.readFloatGyroX();
  float gyroY = myIMU.readFloatGyroY();
  float gyroZ = myIMU.readFloatGyroZ();

  // Calculate the magnitude of acceleration
  float accelMagnitude = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));

  // Step detection based on acceleration magnitude
  unsigned long currentTime = millis();
  if ((currentTime - lastStepTime) > STEP_DELAY && accelMagnitude > STEP_THRESHOLD)
  {
    stepCount++;
    lastStepTime = currentTime;

    // Print step count to Serial Monitor
    Serial.print("Step Detected! Total Steps: ");
    Serial.println(stepCount);
    sendDataToAWS(stepCount, 0); // Send step count; intensity = 0 for now
  }

  // Collect data for intensity calculation only if measuring intensity
  if (intensityMeasuring)
  {
    for (int i = 0; i < SAMPLE_COUNT; i++)
    {
      ax[i] = myIMU.readFloatAccelX();
      ay[i] = myIMU.readFloatAccelY();
      az[i] = myIMU.readFloatAccelZ();

      gx[i] = myIMU.readFloatGyroX();
      gy[i] = myIMU.readFloatGyroY();
      gz[i] = myIMU.readFloatGyroZ();

      delay(SAMPLE_DELAY_MS);
    }

    // Calculate RMS (Root Mean Square) for accelerometer data
    float accelRMS = calculateRMS(ax, ay, az);

    // Calculate RMS for gyroscope data (angular velocity)
    float gyroRMS = calculateRMS(gx, gy, gz);

    // Calculate workout intensity
    float intensity = (accelRMS + gyroRMS) / 2.0; // Combine both accelerometer and gyroscope RMS

    // Determine intensity level
    String intensityLevel = getIntensityLevel(intensity);

    // Output intensity to Serial Monitor
    Serial.print("Workout Intensity: ");
    Serial.print(intensity);
    Serial.print(" m/s^2 (");
    Serial.print(intensityLevel);
    Serial.println(")");
    sendDataToAWS(stepCount, intensity); // Send step count and calculated intensity
  }

  delay(100); // Small delay for stability
}

// Function to calculate RMS for a set of data (accelerometer or gyroscope)
float calculateRMS(float x[], float y[], float z[])
{
  float sumSquares = 0.0;
  for (int i = 0; i < SAMPLE_COUNT; i++)
  {
    sumSquares += pow(x[i], 2) + pow(y[i], 2) + pow(z[i], 2);
  }
  return sqrt(sumSquares / SAMPLE_COUNT);
}

// Function to categorize intensity level
String getIntensityLevel(float intensity)
{
  if (intensity < 4.0)
  {
    return "Low";
  }
  else if (intensity > 5.0 && intensity < 30)
  {
    return "Medium";
  }
  else
  {
    return "High";
  }
}
