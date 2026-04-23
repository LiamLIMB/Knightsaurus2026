#include "driver/spi_slave.h"
#include <Bluepad32.h>

#define PIN_MOSI 23
#define PIN_MISO 19
#define PIN_SCLK 18
#define PIN_CS   5

// DMA-safe buffers (must be 32-bit aligned)
DMA_ATTR uint8_t recvbuf[4];
DMA_ATTR uint8_t sendbuf[4];

spi_slave_transaction_t t;



ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
      }
    }

    if (!foundEmptySlot) {
      Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

    if (!foundController) {
      Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
  /*
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
  */
}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...

   //== XBOX A or PS5 X Button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
    // code for when A button is pushed
    Serial.println("A Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'O';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);    
  }
  if (ctl->buttons() != 0x0001) {
    // code for when A button is released
  }

  //== XBOX X or PS5 Square Button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
    // code for when X button is pushed
    Serial.println("X Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'X';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);    

  }
  if (ctl->buttons() != 0x0004) {
  // code for when X button is released
  }

  //== XBOX Y or PS5 Triangle Button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
    // code for when Y button is pushed
    Serial.println("Y Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'Y';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);    
  }
  if (ctl->buttons() != 0x0008) {
    // code for when Y button is released
  }

  //== XBOX B or PS5 Circle Button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
    // code for when B button is pushed
    Serial.println("B Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'Z';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);    
  }
  if (ctl->buttons() != 0x0002) {
    // code for when B button is released
  }
  
  static bool upWasPressed = false;
  bool upPressed = (ctl->dpad() & 0x01);

  if (upPressed && !upWasPressed) {
      sendbuf[0] = 'B';   // UP pressed
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  if (!upPressed && upWasPressed) {
      sendbuf[0] = 'V';   // UP released
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  upWasPressed = upPressed;

  static bool downWasPressed = false;
  bool downPressed = (ctl->dpad() & 0x02);

  if (downPressed && !downWasPressed) {
      sendbuf[0] = 'C';   // DOWN pressed
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  if (!downPressed && downWasPressed) {
      sendbuf[0] = 'Z';   // DOWN released
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  downWasPressed = downPressed;


  // //== XBOX Dpad LEFT button = 0x08 ==//
  // if (ctl->dpad() & 0x08) {
  //   // code for when dpad left button is pushed
  //   Serial.println("DPAD Left Pushed");
  //   memset(&t, 0, sizeof(t));
  //   memset(recvbuf, 0, sizeof(recvbuf));

  //   sendbuf[0] = 'D';  // ESP32 will send this to the UNO

  //   t.length = 8;               // 1 byte
  //   t.tx_buffer = sendbuf;
  //   t.rx_buffer = recvbuf;

  //   spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  // }

  // ===== LEFT DPAD STATE TRACKING =====
  static bool leftWasPressed = false;
  bool leftPressed = (ctl->dpad() & 0x08);

  if (leftPressed && !leftWasPressed) {
      // LEFT just pressed
      Serial.println("DPAD Left Pressed");
      sendbuf[0] = 'D';
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  if (!leftPressed && leftWasPressed) {
      // LEFT just released
      Serial.println("DPAD Left Released");
      sendbuf[0] = 'K';
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  leftWasPressed = leftPressed;

  // ===== RIGHT DPAD STATE TRACKING =====
  static bool rightWasPressed = false;
  bool rightPressed = (ctl->dpad() & 0x04);

  if (rightPressed && !rightWasPressed) {
      // RIGHT just pressed
      Serial.println("DPAD Right Pressed");
      sendbuf[0] = 'E';
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  if (!rightPressed && rightWasPressed) {
      // RIGHT just released
      Serial.println("DPAD Right Released");
      sendbuf[0] = 'L';
      spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }

  rightWasPressed = rightPressed;


  //== XBOX RB trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
    // code for when RB button is pushed
    Serial.println("RB Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'F';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);



  }
  if (ctl->buttons() != 0x0020) {
    // code for when RB button is released
  }

  //== XBOX RT trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    // code for when RT button is pushed
    Serial.println("RT Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'G';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);


  }
  if (ctl->buttons() != 0x0080) {
    // code for when RT button is released
  }

  //== XBOX LB trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when LB button is pushed
    Serial.println("LB Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'H';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);

  }
  if (ctl->buttons() != 0x0010) {
    // code for when LB button is released
  }

  //== XBOX LT trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    // code for when LT button is pushed
    Serial.println("LT Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'I';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);


  }
  if (ctl->buttons() != 0x0040) {
    // code for when LT button is released
  }

  //== XBOX Screenshot Button = 0x020 ==//
  if (ctl->miscButtons() & 0x02) {
    // code for when Screenshot button is pushed
    Serial.println("Screenshot Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'J';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }
  if (ctl->buttons() != 0x02) {
    // code for when LT button is released
  }

  //== XBOX Start Button = 0x040 ==//
  if (ctl->miscButtons() & 0x04) {
    // code for when Screenshot button is pushed
    Serial.println("Start Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = 'M';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }
  if (ctl->buttons() != 0x02) {
    // code for when LT button is released
  }

  //== R3 = 0x0200 ==//
  if (ctl->buttons() == 0x0200) {
    // code for when R3 button is pushed
    Serial.println("R3 Pushed");
    memset(&t, 0, sizeof(t));
    memset(recvbuf, 0, sizeof(recvbuf));

    sendbuf[0] = '?';  // ESP32 will send this to the UNO

    t.length = 8;               // 1 byte
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
  }
  if (ctl->buttons() != 0x02) {
    // code for when LT button is released
  } 


  // Uncomment if we want joystick capabilities.
  /*
  //== LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -25) {
    // code for when left joystick is pushed up
    }

  //== LEFT JOYSTICK - DOWN ==//
  if (ctl->axisY() >= 25) {
    // code for when left joystick is pushed down
  }

  //== LEFT JOYSTICK - LEFT ==//
  if (ctl->axisX() <= -25) {
    // code for when left joystick is pushed left
  }

  //== LEFT JOYSTICK - RIGHT ==//
  if (ctl->axisX() >= 25) {
    // code for when left joystick is pushed right
  }

  //== LEFT JOYSTICK DEADZONE ==//
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
    // code for when left joystick is at idle
  }

  //== RIGHT JOYSTICK - X AXIS ==//
  if (ctl->axisRX()) {
    // code for when right joystick moves along x-axis
  }

  //== RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
  // code for when right joystick moves along y-axis
  }

  */  

  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
         processGamepad(myController);
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}


void setup() {
  Serial.begin(115200);

  spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_MOSI,
    .miso_io_num = PIN_MISO,
    .sclk_io_num = PIN_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4
  };

  spi_slave_interface_config_t slvcfg = {
    .spics_io_num = PIN_CS,
    .flags = 0,
    .queue_size = 1,
    .mode = 0
  };

  //Serial.println("Test");
  spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);

  Serial.println("ESP32 SPI Slave Ready");

    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);

}

void loop() {

  
  memset(&t, 0, sizeof(t));
  memset(recvbuf, 0, sizeof(recvbuf));

  sendbuf[0] = 'A';  // ESP32 will send this to the UNO

  t.length = 8;               // 1 byte
  t.tx_buffer = sendbuf;
  t.rx_buffer = recvbuf;

  spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);

  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  //delay(150);

}