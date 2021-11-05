/*******************************************
 * Takes a picture on startup,             *
 * saves it to a memory card,              *
 * connects or builds the mesh,            *
 * sends a json object to a specific node. *
 *******************************************/

#include <Arduino.h>

#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/experimental/micro/kernels/micro_ops.h>
#include <tensorflow/lite/experimental/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/experimental/micro/micro_error_reporter.h>
#include <tensorflow/lite/experimental/micro/micro_interpreter.h>
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include <painlessMesh.h>
#include <painlessmesh/base64.hpp>
#include "painlessmesh/plugin.hpp"

#include <EEPROM.h>
#include "esp_camera.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"

// number of bytes we want to access
#define   EEPROM_SIZE     1 

// assign names to pin numbers
#define   PWDN_GPIO_NUM   32
#define   RESET_GPIO_NUM  -1
#define   XCLK_GPIO_NUM   0
#define   SIOD_GPIO_NUM   26
#define   SIOC_GPIO_NUM   27
#define   Y9_GPIO_NUM     35
#define   Y8_GPIO_NUM     34
#define   Y7_GPIO_NUM     39
#define   Y6_GPIO_NUM     36
#define   Y5_GPIO_NUM     21
#define   Y4_GPIO_NUM     19
#define   Y3_GPIO_NUM     18
#define   Y2_GPIO_NUM     5
#define   VSYNC_GPIO_NUM  25
#define   HREF_GPIO_NUM   23
#define   PCLK_GPIO_NUM   22

#define   MESH_PREFIX     "SmartForestMesh"
#define   MESH_PASSWORD   "SWORDFISH_4711"
#define   MESH_PORT       5555

// Identify the destination node's ID with mesh.getNodeId()
#define   DEST_NODE       3177562153

// Custom package for transmission over the mesh network
class PictureDataPackage : public painlessmesh::plugin::SinglePackage {
 public:
  String pictureName;
  String isADeer;

  // Each package has to be identified by a unique ID
  // Values <30 are reserved for default messages, 
  // so using 31 for this package
  PictureDataPackage() : painlessmesh::plugin::SinglePackage(31) {}

  // Convert json object into a PictureDataPackage
  PictureDataPackage(JsonObject jsonObj) : painlessmesh::plugin::SinglePackage(jsonObj) {
    pictureName = jsonObj["pictureName"].as<String>();
    isADeer = jsonObj["isADeer"].as<String>();
  }

  // Convert PictureDataPackage to json object
  JsonObject addTo(JsonObject &&jsonObj) const {
    jsonObj = painlessmesh::plugin::SinglePackage::addTo(std::move(jsonObj));
    jsonObj["pictureName"] = pictureName;
    jsonObj["isADeer"] = isADeer;

    return jsonObj;
  }
  
  // Memory to reserve for converting this object to json
  size_t jsonObjectSize() const {
    return JSON_OBJECT_SIZE(noJsonFields + 2)
            + round(1.1*pictureName.length() 
                    + 1.1*isADeer.length());

  }

  bool isInitialized() {
    return !pictureName.isEmpty() && !isADeer.isEmpty(); //&& !pictureBase64.isEmpty();
  }
};

Scheduler userScheduler; 
painlessMesh mesh;
PictureDataPackage pictureData; // TODO: Replace with QueueArray
int pictureNumber = 0;

/*  USER TASKS  */
void sendReport();
Task taskSendReport(TASK_SECOND * 60, TASK_FOREVER, &sendReport);
void sendReport() {
  if (!pictureData.isInitialized()) {
    Serial.println("pictureData is not initialized!");
  }
  
  if (mesh.sendPackage(&pictureData)) {
    Serial.println("Transmission of package was successful!");
    taskSendReport.disable();
  } else {
    Serial.println("Failed to send package!");
    Serial.printf("Will try again in 60 seconds.\n\n");
  }
}

void takePicture();
Task taskTakePicture(TASK_SECOND * 30, TASK_ONCE, &takePicture);
void takePicture() {
    // Taking the picture and saving the result to frameBuffer
  Serial.println("Starting to take a picture.");
  camera_fb_t * frameBuffer = NULL;
  frameBuffer = esp_camera_fb_get();
  if(!frameBuffer) {
    Serial.println("Camera capture failed!");
    return;
  }
  
  // increment for ascending picture numbers  
  pictureNumber = EEPROM.read(0) + 1;
  
  // saving the picture to the sd card
  String path = "/" + String(mesh.getNodeId()).substring(6) + "_" + String(pictureNumber) +".jpg";
  fs::FS &fs = SD_MMC;  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file in writing mode!");
  } else {
    file.write(frameBuffer->buf, frameBuffer->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n\n", path.c_str());
    EEPROM.write(0, pictureNumber); // update picture number in EEPROM
    EEPROM.commit();
  }
  file.close();
  
  // Turns off the on-board LED connected to GPIO 4
  pinMode(GPIO_NUM_4, OUTPUT);
  digitalWrite(GPIO_NUM_4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);
  
  // Building the package
  pictureData.from = mesh.getNodeId();
  pictureData.dest = DEST_NODE;  // mac address of master node
  pictureData.pictureName = path.substring(1);
  pictureData.isADeer = "Yes"; // put tensorflow stuff here later

  // freeing the space after building the package
  esp_camera_fb_return(frameBuffer);
  
  taskSendReport.enableIfNot();
  taskTakePicture.disable();
}

void initializeCamAndStorage();
Task taskInitializeCamAndStorage(TASK_SECOND * 30, TASK_ONCE, &initializeCamAndStorage);
void initializeCamAndStorage() {
  // configuring the camera
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; 
  
  // configuring picture properties
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  
  // Mounting the sd card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed!");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD Card attached!");
    return;
  }

  // initialize EEPROM for updating pictureNumber
  EEPROM.begin(EEPROM_SIZE);

  taskTakePicture.enable();
  taskInitializeCamAndStorage.disable();
}
/*  END OF USER TASKS */

// Needed for painless library
void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection, nodeId = %u\n", nodeId);
}
void changedConnectionCallback() {
  Serial.printf("Changed connections\n\n");
}
void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n\n", mesh.getNodeTime(),offset);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);

  // starting the mesh
  mesh.setDebugMsgTypes( ERROR | STARTUP ); // set before init() to see startup messages
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // How to handle a package of type 31
  mesh.onPackage(31, [](painlessmesh::protocol::Variant variant) {
    auto package = variant.to<PictureDataPackage>(); 
    Serial.printf("Node %u has taken the picture %s.\n", package.from, package.pictureName.c_str());
    Serial.printf("Did it find a deer? -> %s.\n\n", package.isADeer.c_str());
    return true;
  });

  Serial.printf("The ID of this node is %u \n\n", mesh.getNodeId());

  // use this instead of adding more actions to setup() or loop()
  userScheduler.addTask(taskInitializeCamAndStorage);
  userScheduler.addTask(taskTakePicture);
  userScheduler.addTask(taskSendReport);
  
  taskInitializeCamAndStorage.enableIfNot();
  taskTakePicture.disable();
  taskSendReport.disable();
}

void loop() {
  mesh.update();
}
