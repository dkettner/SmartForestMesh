/****************************************************
 * Connects to or builds the mesh,                  *
 * takes a picture once every two minutes,          *
 * saves it to a memory card,                       *
 * makes a report about the picture,                *
 * puts the report in a queue,                      *
 * tries to send the next report once every minute. *
 * Logs its uptime once every 15 minutes.           *
 ****************************************************/

#include <Arduino.h>
#include <cppQueue.h>
#include <painlessMesh.h>

#include <TensorFlowLite_ESP32.h>
#include <tensorflow/lite/experimental/micro/kernels/micro_ops.h>
#include <tensorflow/lite/experimental/micro/micro_mutable_op_resolver.h>
#include <tensorflow/lite/experimental/micro/micro_error_reporter.h>
#include <tensorflow/lite/experimental/micro/micro_interpreter.h>
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include <EEPROM.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/rtc_io.h"

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
#define   PIR_SENSOR_PIN  16

// index handling with EEPROM
#define   EEPROM_SIZE             8
#define   PICTURE_INDEX_ADDRESS   0
#define   UPTIME_INDEX_ADDRESS    4

// directory paths
#define   PICTURES_PATH     "/pictures"
#define   REPORTS_PATH      "/reports"
#define   UPTIME_LOGS_PATH  "/uptimeLogs"
#define   ERROR_LOGS_PATH   "/errorLogs"

// mesh network
#define   MESH_PREFIX       "SmartForestMesh"
#define   MESH_PASSWORD     "SWORDFISH_4711"
#define   MESH_PORT         5555
#define   DEST_NODE         3177562153        // Identify with mesh.getNodeId()

#define   QUEUE_SIZE        10

// custom package for transmission over the mesh network
class PictureReportPackage : public painlessmesh::plugin::SinglePackage {
 public:
  unsigned long pictureIndex;
  float deerProbability;

  // Each package has to be identified by a unique ID
  // Values <30 are reserved for default messages, 
  // so using 31 for this package
  PictureReportPackage() : painlessmesh::plugin::SinglePackage(31) {}

  // Convert json object into a PictureReportPackage
  PictureReportPackage(JsonObject jsonObj) : painlessmesh::plugin::SinglePackage(jsonObj) {
    pictureIndex = jsonObj["pictureIndex"].as<unsigned long>();
    deerProbability = jsonObj["deerProbability"].as<float>();
  }

  // Convert PictureReportPackage to json object
  JsonObject addTo(JsonObject &&jsonObj) const {
    jsonObj = painlessmesh::plugin::SinglePackage::addTo(std::move(jsonObj));
    jsonObj["pictureIndex"] = pictureIndex;
    jsonObj["deerProbability"] = deerProbability;

    return jsonObj;
  }
  
  // Memory to reserve for converting this object to json
  size_t jsonObjectSize() const {
    return JSON_OBJECT_SIZE(noJsonFields + 2)
            + round(1.1*sizeof(pictureIndex)
                    + 1.1*sizeof(deerProbability));

  }

  String getFullPictureName() {
    return String(this->from % 1000) + "_" + String(pictureIndex) + ".jpg";
  }

  String getFullReportName() {
    return String(this->from % 1000) + "_" + String(pictureIndex) + ".log";
  }

  String getFullErrorName() {
    return String(this->from % 1000) + "_" + String(pictureIndex) + ".err";
  }
};

painlessMesh mesh;
Scheduler userScheduler; 
cppQueue reportQueue(sizeof(PictureReportPackage), QUEUE_SIZE, FIFO);
String directories[] = {
  PICTURES_PATH,
  REPORTS_PATH,
  UPTIME_LOGS_PATH,
  ERROR_LOGS_PATH
};
String uptimeLogPath;

/*  USER TASKS  */
void sendReport();
Task taskSendReport(TASK_SECOND * 60, TASK_FOREVER, &sendReport);
void sendReport() {
  if (reportQueue.isEmpty()) {
    Serial.println("taskSendReport: Queue is empty, nothing to send.");
    return;
  }
  
  PictureReportPackage oldestReport;
  reportQueue.peek(&oldestReport);
  if (mesh.sendPackage(&oldestReport)) {
    Serial.printf("taskSendReport: Transmission of report %s was successful.\n", oldestReport.getFullReportName().c_str());
    reportQueue.drop();
  } else {
    Serial.printf("taskSendReport: Failed to send report %s!\n", oldestReport.getFullReportName().c_str());
  }
}

void takePicture();
Task taskTakePicture(TASK_SECOND * 120, TASK_FOREVER, &takePicture);
void takePicture() {
  Serial.println("taskTakePicture: Starting to take a picture.");
  fs::FS &fs = SD_MMC;
  unsigned long nextPictureIndex = EEPROM.readULong(PICTURE_INDEX_ADDRESS);
  EEPROM.writeULong(PICTURE_INDEX_ADDRESS, nextPictureIndex + 1);   // increment picture number in EEPROM
  EEPROM.commit(); // EEPROM.end(); ???

  // Take the picture
  camera_fb_t * frameBuffer = NULL;
  frameBuffer = esp_camera_fb_get();
  if(!frameBuffer) {
    Serial.println("taskTakePicture: Camera capture failed!");
    return;
  }
  
  // Save the picture to the sd card
  String path = String(PICTURES_PATH) + "/" + String(mesh.getNodeId()).substring(7) + "_" + String(nextPictureIndex) + ".jpg";
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file) {
    Serial.println("taskTakePicture: Failed to open file in writing mode!");
  } else {
    file.write(frameBuffer->buf, frameBuffer->len);   // payload (image), payload length
    Serial.printf("taskTakePicture: Saved picture to path: %s\n", path.c_str());
  }
  file.close();
  
  // Build the new report
  // TODO: put report handling in separate function
  PictureReportPackage newReport;
  newReport.from = mesh.getNodeId();
  newReport.dest = DEST_NODE;
  newReport.pictureIndex = nextPictureIndex;
  newReport.deerProbability = 0.5; // put tensorflow stuff here later

  // Save Report to SD card
  String newReportLogPath = String(REPORTS_PATH) + "/" + newReport.getFullReportName();
  File newReportLog = fs.open(newReportLogPath, FILE_WRITE);
  if (!newReportLog) {
    Serial.printf("taskTakePicture: Could not create %s!\n", newReportLogPath.c_str());
  } else {
    // TODO: Use proper json objects
    newReportLog.printf("Report about %s\n\n", newReport.getFullPictureName().c_str());
    newReportLog.printf("from: %zu\n", newReport.from);
    newReportLog.printf("dest: %zu\n", newReport.dest);
    newReportLog.printf("pictureIndex: %lu\n", newReport.pictureIndex);
    newReportLog.printf("deerProbability: %.2f\n", newReport.deerProbability);
    newReportLog.close();

    Serial.printf("taskTakePicture: Saved report to path: %s\n", newReportLogPath.c_str());
  }

  // Return if no deer was found
  if (newReport.deerProbability < 0.5) {
    Serial.printf("taskTakePicture: No deer found on %s.\n", newReport.getFullPictureName().c_str());
    Serial.println("taskTakePicture: Report will not get pushed to queue.");
    esp_camera_fb_return(frameBuffer);
    return;
  }

  // Drop oldest report if queue is full and create error log.
  if (reportQueue.isFull()) {
    Serial.println("taskTakePicture: Queue is full.");

    PictureReportPackage oldestReport;
    reportQueue.pop(&oldestReport);
    Serial.printf("taskTakePicture: Dropped the oldest report: %s\n", oldestReport.getFullReportName().c_str());
    String newErrorLogPath = String(ERROR_LOGS_PATH) + "/" + oldestReport.getFullErrorName();
    File newErrorLog = fs.open(newErrorLogPath.c_str(), FILE_WRITE);
    if (!newErrorLog) {
      Serial.printf("taskTakePicture: Could not create %s!\n", newErrorLogPath.c_str());
    } else {
      Serial.printf("taskTakePicture: Saved error log to path: %s\n", newErrorLogPath.c_str());
      newErrorLog.close();
    }
  }

  // Push new report to queue
  reportQueue.push(&newReport);
  Serial.println("taskTakePicture: Pushed report to queue.");

  // Cleanup
  esp_camera_fb_return(frameBuffer);
  digitalWrite(GPIO_NUM_4, LOW);
}

void takePicturePIR();
Task taskTakePicturePIR(TASK_SECOND * 15, TASK_FOREVER, &takePicturePIR);
void takePicturePIR() {
  // Return if no movement is being detected
  if (digitalRead(PIR_SENSOR_PIN) == HIGH) {   // TODO: Check if HIGH or LOW
    return;
  }
  
  Serial.println("taskTakePicturePIR: Starting to take a picture.");
  fs::FS &fs = SD_MMC;
  unsigned long nextPictureIndex = EEPROM.readULong(PICTURE_INDEX_ADDRESS);
  EEPROM.writeULong(PICTURE_INDEX_ADDRESS, nextPictureIndex + 1);   // increment picture number in EEPROM
  EEPROM.commit(); // EEPROM.end(); ???

  // Taking the picture
  camera_fb_t * frameBuffer = NULL;
  frameBuffer = esp_camera_fb_get();
  if(!frameBuffer) {
    Serial.println("taskTakePicturePIR: Camera capture failed!");
    return;
  }
  
  // Saving the picture to the sd card
  String path = String(PICTURES_PATH) + "/" + String(mesh.getNodeId()).substring(7) + "_" + String(nextPictureIndex) + ".jpg";
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file) {
    Serial.println("taskTakePicturePIR: Failed to open file in writing mode!");
  } else {
    file.write(frameBuffer->buf, frameBuffer->len);   
    Serial.printf("taskTakePicturePIR: Saved picture to path: %s\n", path.c_str());
  }
  file.close();
  
  // Build the new report
  PictureReportPackage newReport;
  newReport.from = mesh.getNodeId();
  newReport.dest = DEST_NODE;
  newReport.pictureIndex = nextPictureIndex;
  newReport.deerProbability = 0.5; // put tensorflow stuff here later

  // Save Report to SD card
  String newReportLogPath = String(REPORTS_PATH) + "/" + newReport.getFullReportName();
  File newReportLog = fs.open(newReportLogPath, FILE_WRITE);
  if (!newReportLog) {
    Serial.printf("taskTakePicturePIR: Could not create %s!\n", newReportLogPath.c_str());
  } else {
    // TODO: Use proper json objects
    newReportLog.printf("Report about %s\n\n", newReport.getFullPictureName().c_str());
    newReportLog.printf("from: %zu\n", newReport.from);
    newReportLog.printf("dest: %zu\n", newReport.dest);
    newReportLog.printf("pictureIndex: %lu\n", newReport.pictureIndex);
    newReportLog.printf("deerProbability: %.2f\n", newReport.deerProbability);
    newReportLog.close();

    Serial.printf("taskTakePicture: Saved report to path: %s\n", newReportLogPath.c_str());
  }

  // Return if no deer was found
  if (newReport.deerProbability < 0.5) {
    Serial.printf("taskTakePicturePIR: No deer found on %s.\n", newReport.getFullPictureName().c_str());
    Serial.println("taskTakePicturePIR: Report will not get pushed to queue.");
    esp_camera_fb_return(frameBuffer);
    return;
  }

  // Drop oldest report if queue is full and create error log.
  if (reportQueue.isFull()) {
    Serial.println("taskTakePicturePIR: Queue is full.");

    PictureReportPackage oldestReport;
    reportQueue.pop(&oldestReport);
    Serial.printf("taskTakePicturePIR: Dropped the oldest report: %s\n", oldestReport.getFullReportName().c_str());
    String newErrorLogPath = String(ERROR_LOGS_PATH) + "/" + oldestReport.getFullErrorName();
    File newErrorLog = fs.open(newErrorLogPath.c_str(), FILE_WRITE);
    if (!newErrorLog) {
      Serial.printf("taskTakePicturePIR: Could not create %s!\n", newErrorLogPath.c_str());
    } else {
      Serial.printf("taskTakePicturePIR: Saved error log to path: %s\n", newErrorLogPath.c_str());
      newErrorLog.close();
    }
  }

  // Push new report to queue
  reportQueue.push(&newReport);
  Serial.println("taskTakePicture: Pushed report to queue.");

  // Cleanup
  esp_camera_fb_return(frameBuffer);
  digitalWrite(GPIO_NUM_4, LOW);
}

void logUptime();
Task taskLogUptime(TASK_MINUTE * 10, TASK_FOREVER, &logUptime);
void logUptime() {
  fs::FS &fs = SD_MMC;
  File uptimeLog = fs.open(uptimeLogPath.c_str(), FILE_APPEND);
  if(!uptimeLog) {
    Serial.printf("taskInitializeStorage: Failed to open %s!\n", uptimeLogPath.c_str());
  } else {
    float newUptime = (float) millis() / 60000;   // uptime in minutes
    String newUptimeEntry = String(newUptime) + " min" ;
    uptimeLog.println(newUptimeEntry.c_str());
    uptimeLog.close();

    Serial.printf("taskLogUptime: Appended new uptime: %.2f min.\n", newUptime);
  }
}


void resetEeprom(int range) {
  for (int eepromAddress = 0; eepromAddress < range; eepromAddress++) {
    EEPROM.write(eepromAddress, 0);
  }
  EEPROM.commit();
  return;
}
void initializeStorage();
Task taskInitializeStorage(TASK_SECOND * 30, TASK_ONCE, &initializeStorage);
void initializeStorage() {
  // Initializing EEPROM
  EEPROM.begin(EEPROM_SIZE);
  Serial.println("taskInitializeStorage: Initialized EEPROM.");
  // resetEeprom(EEPROM_SIZE); // uncomment if needed

  // Mounting the sd card
  if (!SD_MMC.begin()) {
    Serial.println("taskInitializeStorage: SD card mount failed!");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("taskInitializeStorage: No SD card attached!");
    return;
  }
  Serial.println("taskInitializeStorage: SD card mount was successful.");
  
  // Creating directories
  Serial.println("taskInitializeStorage: Starting to create nonexistent directories.");
  fs::FS &fs = SD_MMC;
  for (String currentDirectory: directories) {
    if (!fs.exists(currentDirectory.c_str())) {
      if (fs.mkdir(currentDirectory.c_str())) {
        Serial.printf("taskInitializeStorage: Created directory: %s \n", currentDirectory.c_str());
      } else {
        Serial.printf("taskInitializeStorage: Could not create directory: %s !\n", currentDirectory.c_str());
        // Try again?
      }
    } else {
      Serial.printf("taskInitializeStorage: Directory %s already exists.\n", currentDirectory.c_str());
    }
  }

  // Creating new uptimeLogPath 
  unsigned long nextUptimeIndex = EEPROM.readULong(UPTIME_INDEX_ADDRESS);
  EEPROM.writeULong(UPTIME_INDEX_ADDRESS, nextUptimeIndex + 1);
  EEPROM.commit(); // EEPROM.end(); ?
  String newUptimelogPath = String(UPTIME_LOGS_PATH) + "/ut" + String(nextUptimeIndex) + ".log";

  // Creating new uptimeLog (the actual file)
  File newUptimeLog = fs.open(newUptimelogPath.c_str(), FILE_WRITE);
  if(!newUptimeLog) {
    Serial.printf("taskInitializeStorage: Failed to create %s!\n", newUptimelogPath.c_str());
  } else {
    Serial.printf("taskInitializeStorage: Saved uptime log to path: %s\n", newUptimelogPath.c_str());
    uptimeLogPath = newUptimelogPath;
  }
  newUptimeLog.close();

  // Next state
  taskTakePicture.enableIfNot();
  taskLogUptime.enableIfNot();
  taskInitializeStorage.disable();
}

void initializeCamera();
Task taskInitializeCamera(TASK_SECOND * 30, TASK_ONCE, &initializeCamera);
void initializeCamera() {
  // GPIO 4 is soldered to SD card and LED flash
  // This fixes current drops which causes SD problems (somehow)
  pinMode(GPIO_NUM_4, OUTPUT);   
  digitalWrite(4, LOW);
  rtc_gpio_hold_dis(GPIO_NUM_4);

  // Prepare for readings from PIR sensor
  pinMode(PIR_SENSOR_PIN, INPUT);
  
  Serial.println("taskInitializeCamera: Configuring camera and picture properties.");
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
  if (psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 1;  // = 2 was unstable and led to errors 
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Init Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("taskInitializeCamera: Camera init failed with error 0x%x!", err);
    return;
  }

  Serial.println("taskInitializeCamera: Finished configuration.");

  // Next state
  taskInitializeStorage.enableIfNot();
  taskInitializeCamera.disable();
}
/*  END OF USER TASKS */

// Needed for painless library
void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("mesh: New connection with node %u.\n", nodeId);
}
void changedConnectionCallback() {
  Serial.println("mesh: Changed connections.");
}
void nodeTimeAdjustedCallback(int32_t offset) {
  // Uncomment if needed.
  // Serial.printf("mesh: Adjusted time %u, offset = %d.\n", mesh.getNodeTime(), offset);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // disable brownout detector
  Serial.begin(115200);

  // starting the mesh
  mesh.setDebugMsgTypes( ERROR | STARTUP );
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  // How to handle a package of type 31
  mesh.onPackage(31, [](painlessmesh::protocol::Variant variant) {
    auto package = variant.to<PictureReportPackage>(); 
    Serial.printf("mesh: Node %zu has taken the picture %s.\n", package.from, package.getFullPictureName().c_str());
    Serial.printf("mesh: Deer probability: %.2f\n", package.deerProbability);
    return true;
  });

  Serial.printf("\nmesh: The ID of this node is %zu.\n", mesh.getNodeId());

  // use this instead of adding more actions to setup() or loop()
  userScheduler.addTask(taskInitializeCamera);
  userScheduler.addTask(taskInitializeStorage);
  userScheduler.addTask(taskTakePicture);
  userScheduler.addTask(taskSendReport);
  userScheduler.addTask(taskLogUptime);
  
  // Next state
  taskTakePicture.disable();
  taskLogUptime.disable();
  taskInitializeStorage.disable();
  taskInitializeCamera.enableIfNot();
  taskSendReport.enableIfNot();   // race conditions?
}

void loop() {
  mesh.update();
}
