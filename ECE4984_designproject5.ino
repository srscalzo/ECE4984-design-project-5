/*
  Virginia Tech
  ECE 4984/5984 - Fall 2025
  Design Project 5
  October 20, 2025

  Izzy Burley and Sam Scalzo

  Bluetooth Low Energy (BLE) Client Consuming Temperature Data from a Remote Server
  and Displaying Temperature

  Acknowledgments:
  - "Bluetooth Overview" and other wiki pages at Seeed Studio:
    https://wiki.seeedstudio.com/Wio-Terminal-Bluetooth-Overview/.

  - Seeed Arduino rpcBLE library: https://github.com/Seeed-Studio/Seeed_Arduino_rpcBLE.

  - This code build from the BLE_client.ino example from the Seeed Arduino library:
    https://github.com/Seeed-Studio/Seeed_Arduino_rpcBLE/tree/master/examples/BLE_client.

  - General use of the Wio Terminal based on examples from Wio Terminal
    Classroom, Lakshantha Dissanayake (lakshanthad), at:
    https://github.com/lakshanthad/Wio_Terminal_Classroom_Arduino.
*/


// Include header files. Based on experience with the server, the two BLE header
// files are included before the others.
#include <rpcBLEDevice.h> // rpcBLE device header file
#include <BLEScan.h> // rpcBLE scanner header file
#include <BLEAdvertisedDevice.h> // rpcBLE advertised device header file
#include "TFT_eSPI.h" // TFT LCD header file
#include "Free_Fonts.h" // Free fonts header 

// Set debug. Comment out to remove debug code.
#define DEBUG

// Define shared measure for screen layout. This is the height of the bars at the
// top and bottom of the screen for data source and device state, respectively.
#define STATUS_HEIGHT 30

// Define delay time for displays in seconds.
#define DELAY_STATEPAUSE  2   // Delay time to see state where needed

// Define BLE scan duration in seconds. 5 seconds is generally a good value, but
// while debugging, the client will likely miss the server advertisement when
// resetting since the server takes some time to figure out that it is now
// disconnected. 10 seconds seems to work well for this situation.
#define BLE_SCAN_DURATION 10

// Instantiate the TFT for display.
TFT_eSPI tft;

// Declare client state values and global client state varialble.
#define STATE_INIT      0     // Initializing
#define STATE_SCAN      1     // Scanning
#define STATE_SCANFAIL  2     // Scan failed
#define STATE_SERVFND   3     // Server found
#define STATE_CONN      4     // Connected
#define STATE_ACTIVE    5     // Connection active
#define STATE_CONNFAIL  6     // Connection failed
#define STATE_DISCONN   7     // Disconnected
static int clientState = STATE_INIT;

// Declare global for time that the scan is considered to have failed. This
// time will be set to be a bit past the current clock plus the scan duration.
unsigned long scanFailedTime = 0;


// ***** Initializations for BLE Follow *****

// Define custome UUID values used at the BLE server. These values must match those
// used on the server. In this program, we just use the service UUID and the
// characteristic UUID.
#define SENSOR_SERVICE_UUID "019971b3-31c5-73de-8e9e-e47c22fc0c87"
#define TEMP_CHARACTERISTIC_UUID "019971b3-31c5-73de-8e9e-e47c22fc0c88"
// #define TEMP_DESCRIPTOR_UUID "019971b3-31c5-73de-8e9e-e47c22fc0c89"
#define HUMID_CHARACTERISTIC_UUID "019971b3-31c5-73de-8e9e-e47c22fc0c8a"

// Convert these UUIDs from character strings to BLEUUID types.
static BLEUUID serviceUUID(SENSOR_SERVICE_UUID);
static BLEUUID tempCharUUID(TEMP_CHARACTERISTIC_UUID);
static BLEUUID humidCharUUID(HUMID_CHARACTERISTIC_UUID);


// Declare global variables for the pointers to the temperature characteristics at
// the remote endpoint (the server) and for the server device
// which has advertised itself.
static BLERemoteCharacteristic* pRemoteTempCharacteristic;
static BLEAdvertisedDevice* myDevice;
static BLERemoteCharacteristic* pRemoteHumidCharacteristic;


// In this client, we restrict the server of interest to a particular BLE device
// which is identified by the 48-bit MAC address of the server's BLE interface. An
// easy way to discover this address is from a device scan by the Nordic nRF Connect
// for Mobile application.
//
// Note that byte order as stored here is reversed from what is diplayed for the BLE
// MAC address. For example, address "2C:F7:F1:0A:18:7D" would be initialized as:
// uint8_t bd_addr[6] = {0x7d, 0x18, 0x0a, 0xf1, 0xf7, 0x2c};
//
// Define the BLE device address of interest and convert it to a BLEAddress type. Be
// sure that bd_addr this is the MAC address of the target server. Be careful of byte
// ordering.
//
// ** UPDATE THIS VALUE TO MATCH YOUR SERVER DEVICE. **
uint8_t bd_addr[6] = {0xc2, 0xa5, 0x1f, 0xf1, 0xf7, 0x2c};  // Kit 31 BLE MAC address
BLEAddress TempServer(bd_addr);

// Define the callback routine for new temperature data from the server from a notification. Display
// the temperature value. The callback parameters provide a pointer to the remote characteristic,
// the data (as bytes) and the length of the data (number of bytes).
static void notifyTempCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // Convert value to null terminated string and display it as temperature.
    char valueBuffer[8];
    int j;
    for (j=0; j<length; ++j) {
      valueBuffer[j] = (char) *pData;
      ++pData;
    }
    valueBuffer[length] = NULL;
    displayTempText(valueBuffer);

    #ifdef DEBUG
    Serial.print("STATUS: Notify callback for temperature characteristic: ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print(" * Temperature: ");
    Serial.println(valueBuffer);
    #endif
}

// Define the callback routine for new humidity data from the server from a notification. Display
// the humidity value. The callback parameters provide a pointer to the remote characteristic,
// the data (as bytes) and the length of the data (number of bytes).
static void notifyHumidCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // Convert value to null terminated string and display it as humidity.
    char valueBuffer[8];
    int j;
    for (j=0; j<length; ++j) {
      valueBuffer[j] = (char) *pData;
      ++pData;
    }
    valueBuffer[length] = NULL;
    displayHumid(valueBuffer);

    #ifdef DEBUG
    Serial.print("STATUS: Notify callback for humidity characteristic: ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print(" * Humidity: ");
    Serial.println(valueBuffer);
    #endif
}


// Define callback routines for the client device. These are called on connect and on
// disconnect. The callback parameter provides a pointer to the client connected to
// the server.
class MyClientCallback : public BLEClientCallbacks {

  void onConnect(BLEClient* pclient) {  
    // onConnect is just for debugging.    
    #ifdef DEBUG
    Serial.println("STATUS: onConnect called");
    #endif
  }

  void onDisconnect(BLEClient* pclient) {
    // Set state to show client is not connected. Update state and display.
    // Delay a little to allow previous state to be visible.
    clientState = STATE_DISCONN;
    delay(DELAY_STATEPAUSE*1000);
    displayState("Disconnected");

    #ifdef DEBUG
    Serial.println("STATUS: onDisconnect called");
    #endif
  }
};


// Define callback routine for scanning. onResult() will be called for each advertised
// device found. The onResult() callback routine will check if the address of the
// advertised device mataches the target for this client. The callback parameter
// provides a pointer to the advertised device that has been found.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks { 

  void onResult(BLEAdvertisedDevice advertisedDevice) {
    #ifdef DEBUG
    Serial.print("STATUS: BLE advertised device is ");
    Serial.println(advertisedDevice.toString().c_str());
    #endif
	  
    // For this device, check if its 48-bit BLE MAC address matches the target server's MAC
    // address. If it does, then stop scanning, set myDevice to the device that is found, and
    // set the client state to STATE_SERVFND (server found) to indicate that a connection is
    // needed.
    if (memcmp(advertisedDevice.getAddress().getNative(),TempServer.getNative(), 6) == 0) {
      #ifdef DEBUG
      Serial.print("STATUS: Temperature server device found - ");
      Serial.println(advertisedDevice.toString().c_str());
      #endif

      // Stop the scan.
      BLEDevice::getScan()->stop();

      // Set myDevice to point to this advertised device.
      myDevice = new BLEAdvertisedDevice(advertisedDevice);

      // Set new client state that this is the device to which to connect and display.
      clientState = STATE_SERVFND;
      displayState("Server Found");
    }
  }

};


/*
  Main Routines
*/

// Setup function.
void setup() {

  #ifdef DEBUG
  Serial.begin(115200);
//  while(!Serial){};
  #endif

  // Initialize the display by starting the TFT LCD and setting screen rotation.
  // The screen rotation is set for "landscape" mode with the USB connector at the
  // bottom of the screen.
  tft.begin();
  tft.setRotation(3);
  tft.backlight();

  screenHome();
  delay(3000);
  // Display blank area for data source. Draw the area for displaying temperature.
  displaySource("");
  drawTemp();

  // Indicate that the device is initializing and delay by DELAY_STATEPAUSE seconds
  // to keep the state visible.
  clientState = STATE_INIT;
  displayState("Initializing");
  delay(DELAY_STATEPAUSE*1000);

  #ifdef DEBUG
  Serial.println("STATUS: Starting BLE temperature client");
  #endif
  BLEDevice::init("");

  // Retrieve a scanner and set the callback routine to use to be informed when a
  // new device is detected. Specify active scanning and start the scan to run for
  // BLE_SCAN_DURATION seconds. The scan interval and scan window times here seem
  // to work well, but other values might also work well.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);

  // Update state and display.
  clientState = STATE_SCAN;
  displayState("Scanning");

  // Start scanning.
  pBLEScan->start(BLE_SCAN_DURATION, false);

  // Set a time past the end of the scan to know that the scan has failed to
  // find an advertising server of interest.
  scanFailedTime = millis() + (BLE_SCAN_DURATION * 1000) + 1000;
}


// Main loop function.
void loop() {
	
  // If the state is STATE_SERVFND (server found), then the client has scanned and found
  // a BLE server that has the target MAC address. Attempt to have the client connect to
  // the advertised device. The call to connectToServer() returns true if the client
  // connects and finds the target service and characteristic and can read data and
  // receive notifications for the characteristic. Otherwise, it returns false. Update
  // the state and state message based on success or failure.
  if (clientState == STATE_SERVFND) {

    if (connectToServer()) {
      #ifdef DEBUG
      Serial.println("STATUS: Client connected to remote BLE server");
      #endif

      // Update state and display.
      clientState = STATE_ACTIVE;
      displayState("Connection Active");
    }
    else {
      #ifdef DEBUG
      Serial.println("ERROR: Client failed to connect to the server, so stopping");
      #endif

      // Update state and display;
      clientState = STATE_CONNFAIL;
      displayState("Connection Failed");
    }

  }

  // If the client is in the STATE_SCAN (scanning) state and the scan duration has been
  // exceeded, the the scan has failed to find a server with the target address. The
  // client updates the state and displays the state.
  else if (clientState == STATE_SCAN && millis() > scanFailedTime) {
    #ifdef DEBUG
    Serial.println("ERROR: Scanning ended without finding a target device, so stopping");
    #endif

    // Update state and display.
    clientState = STATE_SCANFAIL;
    displayState("Scan Failed");
  }

  // Wait a bit for continuing loop().
  delay(1000);
}


/*
  Helper Routines
*/
void screenHome() {
  tft.setFreeFont(FSS12);
  tft.setTextColor(TFT_WHITE);
  tft.fillScreen(TFT_DARKGREEN);
  tft.setTextDatum(MC_DATUM); 
  tft.drawString("Virginia Tech", 50, 50);
  tft.drawString("ECE 4984", 50, 75);
  tft.drawString("Fall 2025", 50, 100);
  tft.drawString("Project 5", 50, 125);
  
  // Personal info
  tft.setFreeFont(FF1);
  tft.drawString("Izzy Burley and Samuel Scalzo", 50, 170);
  tft.drawString("izzyburley@vt.edu", 50, 200);
  tft.drawString("srscalzo@vt.edu", 50, 220);
}

// Connect to the server once an advertised device is found that matches the target
// 48-bit BLE MAC address. This routine will:
// - Connect to the server device (mydevice)
// - Verify that the server has the right service (based on UUID)
// - Verify that the service can provide the temperature characteristic via a read and,
//   if so, update the temperature display
// - Verify if the service can provide notifications of the temperature characteristic
//   and, if so, register for notifications
// The routine returns false (failure) if any of the verifications fail. Otherwise,
// the routine returns true.
bool connectToServer() {
  #ifdef DEBUG
  Serial.print("STATUS: Connecting to ");
  Serial.println(myDevice->getAddress().toString().c_str());
  #endif

  // Create a client object which allows access to the server attributes. Set
  // callback routines for the client (onConnect and onDisconnect).
  BLEClient*  pClient  = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
	
  // Actually connect to the remote server as found from scanning to have the target
  // MAC address. Exit with error if unsuccessful.
  if (pClient->connect(myDevice)) {
    #ifdef DEBUG
    Serial.println("STATUS: Connected to server, but not verified service, etc.");
    #endif   

    // Set state to show that client is now connected. Update state and display.
    // Delay a bit to allow previous state to be visible.
    clientState = STATE_CONN;
    displayState("Connected");
  }
  else {
    #ifdef DEBUG
    Serial.println("ERROR: Connect to server failed");
    #endif

    return(false);
  }

  // Obtain a reference to the sensor service at the remote BLE server. Disconnect
  // and fail (return false) if the pointer for the remote service is null.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  
  if (pRemoteService == nullptr) {
    #ifdef DEBUG
    Serial.print("ERROR: Failed to find service with UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    #endif

    // Disconnect and return false.
    pClient->disconnect();
    return false;
  }

  // Display the data source (the name of the server device). Note that the call
  // to myDevice-getName() returns a string of type std::string. The c_str()
  // method will turn this into a C style null-terminated string.
  char sourceBuffer[32];
  strcpy(sourceBuffer, myDevice->getName().c_str());
  displaySource(sourceBuffer);

  #ifdef DEBUG  
  Serial.print("STATUS: Connected to server ");
  Serial.print(sourceBuffer);
  Serial.print(" with service UUID ");
  Serial.println(serviceUUID.toString().c_str());
  #endif

  // Obtain a reference to the temperature characteristic in the service of the
  // remote BLE server. Exit with failure if the client cannot find the
  // characteristic.
  pRemoteTempCharacteristic = pRemoteService->getCharacteristic(tempCharUUID);
  pRemoteHumidCharacteristic = pRemoteService->getCharacteristic(humidCharUUID);

  if (pRemoteTempCharacteristic == nullptr) {
    #ifdef DEBUG
    Serial.print("ERROR: Failed to find temperature characteristic UUID ");
    Serial.println(tempCharUUID.toString().c_str());
    #endif

    // Disconnect and return false.
    pClient->disconnect();
    return false;
  }
  else {
    #ifdef DEBUG
    Serial.print("STATUS: Found temperature characteristic UUID ");
    Serial.println(tempCharUUID.toString().c_str());
    #endif
  }

  // Read the value of the temperature characteristic and display it. If the
  // client cannot read the characteristic, then fail.
  if(pRemoteTempCharacteristic->canRead()) {
    #ifdef DEBUG
    Serial.println("STATUS: Can read temperature characteristic");
    #endif

    // Get the temperature string and display it.
    std::string value = pRemoteTempCharacteristic->readValue();
    char tempBuffer[8];
    strcpy(tempBuffer, value.c_str());
    displayTempText(tempBuffer);

    // Get the humidity string and display it.
    std::string humidValue = pRemoteHumidCharacteristic->readValue();
    char humidBuffer[8];
    strcpy(humidBuffer, humidValue.c_str());
    displayHumid(humidBuffer);

    #ifdef DEBUG
    Serial.print("STATUS: Temperature is ");
    Serial.println(tempBuffer);
    #endif
  }
  else {
    #ifdef DEBUG
    Serial.println("ERROR: Cannot read temperature characteristic");
    #endif

    // Disconnect and return false.
    pClient->disconnect();
    return false;
  }
    
  // Register for notifications for the temperature characteristic. Fail
  // if the client cannot receive notifications for this characteristic.
  if(pRemoteTempCharacteristic->canNotify()) {
    #ifdef DEBUG
    Serial.println("STATUS: Can use notifications for temperature");
    #endif

    pRemoteTempCharacteristic->registerForNotify(notifyTempCallback);
  }
  else {
    #ifdef DEBUG
    Serial.println("ERROR: Notifications not supported for temperature");
    #endif

    // Disconnect and return false.
    pClient->disconnect();
    return false;
  }  

  // Register for notifications for the humidity characteristic. Fail
  // if the client cannot receive notifications for this characteristic.
  if(pRemoteHumidCharacteristic->canNotify()) {
    #ifdef DEBUG
    Serial.println("STATUS: Can use notifications for humidity");
    #endif

    pRemoteHumidCharacteristic->registerForNotify(notifyHumidCallback);
  }
  else {
    #ifdef DEBUG
    Serial.println("ERROR: Notifications not supported for humidity");
    #endif

    // Disconnect and return false.
    pClient->disconnect();
    return false;
  }  

  // Return success (true).
  return true;
}


// Display the server state at the bottom of the screen by displaying the
// provided string.
void displayState(char* stateMessage) {
  tft.setFreeFont(FSS9);
  tft.setTextColor(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.fillRect(0, TFT_WIDTH-STATUS_HEIGHT, TFT_HEIGHT, STATUS_HEIGHT, TFT_ORANGE);
  tft.drawString(stateMessage, TFT_HEIGHT/2, TFT_WIDTH-STATUS_HEIGHT+12);
}


// Draw the screen area for displaying temperature.
void drawTemp() {
  tft.setTextDatum(MC_DATUM);

  tft.fillRect(0, STATUS_HEIGHT, TFT_HEIGHT, TFT_WIDTH-(2*STATUS_HEIGHT), TFT_LIGHTGREY);
  tft.fillRect(5, STATUS_HEIGHT+5, (TFT_HEIGHT/2)-8, TFT_WIDTH-(2*STATUS_HEIGHT)-10, TFT_DARKGREEN);
  tft.fillRect((TFT_HEIGHT/2)+3, STATUS_HEIGHT+5, (TFT_HEIGHT/2)-8, TFT_WIDTH-(2*STATUS_HEIGHT)-10, TFT_DARKGREEN);
  
  tft.setFreeFont(FSS9);
  tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("Temperature", TFT_HEIGHT/4+1, STATUS_HEIGHT+18);
  tft.drawString("Fahrenheit", TFT_HEIGHT/4, TFT_WIDTH-STATUS_HEIGHT-20);
  //Added Humidity
  tft.drawString("Humidity", 3*TFT_HEIGHT/4, STATUS_HEIGHT+18);
  tft.drawString("Percent", 3*TFT_HEIGHT/4, TFT_WIDTH-STATUS_HEIGHT-20);
}


// Display the provided temperature.
void displayTempText(char* displayTemp) {
  // Fill in the display area for the temperature to erase old value.
  tft.fillRect(6, (TFT_WIDTH/2)-25, (TFT_HEIGHT/2)-10, 42, TFT_DARKGREEN);
  
  // Display temperature value.
  tft.setFreeFont(FSSB24);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(displayTemp, (TFT_HEIGHT/4)-2, (TFT_WIDTH/2)-7);
}

// Display the provided humidity value
// Display the provided humidity value (string)
void displayHumid(char* displayHumid) {
  // Fill in the display area to erase old values
  tft.fillRect((TFT_HEIGHT/2)+4, (TFT_WIDTH/2)-25, (TFT_HEIGHT/2)-10, 42, TFT_DARKGREEN);

  // Display humidity string
  tft.setFreeFont(FSSB24);
  tft.setTextColor(TFT_WHITE);
  tft.drawString(displayHumid, 3*TFT_HEIGHT/4, (TFT_WIDTH/2)-7);
}


// Display the data source at the top of the screen by displaying the provided
// string.
void displaySource(char* sourceMessage) {
  tft.setFreeFont(FSSB9);
  tft.setTextColor(TFT_DARKGREEN);
  tft.setTextDatum(MC_DATUM);
  tft.fillRect(0, 0, TFT_HEIGHT, STATUS_HEIGHT, TFT_LIGHTGREY);
  tft.drawString(sourceMessage, TFT_HEIGHT/2, STATUS_HEIGHT-14);
}
