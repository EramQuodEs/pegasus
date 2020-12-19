/* House Keeping
   The device starts at OTA check.
   If it finds new fw, it updates the code and restarts. If no new code, it propagates to ss_InitMeasure.
   In this state all the peripherals are started, along with their threads.
   After that the meausring state is introduced.
   In this state the peripherals are checked for errors, miscommunications and if problems are met
   possible solutions, ie. reconnections, resets etc is applied.
   After the peripheral check is done, an idle state is set in which the device checks for wifi signal and location.
   If the locations, based on
   the GPS signal is found to be near the base, or the wifi SSID of the base is read,
   then the device stop the measuring sequence and starts
   the OTA and the DATA flush loops.
     //  1. ss_OTAUpdate,       //OTA Update check
     //  2. ss_DataFlush,       //Transfer orphan data
     //  3. ss_InitMeasure,     //Init meas system
     //  4. ss_Measure,         //Meas
     //  5. ss_Idle             //Check Wifi and select route
*/
void MWD_routine( void * parameter) {
  for (;;) {
    MWD_houseKeeping();
    delay(25000);
  }
}
void MWD_houseKeeping() {
  //Cehck for SD clogging
  //  Serial.println(String(temprature_sens_read() - 32) / 1.8);

  //Serial.print("espState: ");  Serial.println(espState);
  //this thread checks for peripheral timeouts, wifi presense and reroute code execution MWD=Manage watchdogs
  //=========================================================================================================================System Initialization for Measurements================================================================================================
  if ((espState == ss_InitMeasure) && (init_finished == false)) {
    Serial.println("Initializing main peripherals");
    if (WS_alive) {
      webSocket.sendTXT(":: Initializing main peripherals");
    }
    //Serial.println("****apong*****");
    //Initialize main peripherals
    //Let's say we don't see any peripherals and try to discover them...
    IMU_Present = false;
    GPS_Present = false;
    SD_Present  = false;
    CAN_Present = false;
    //====================================================================================
    //1. Initialize Canbus
    if (WS_alive) {
      webSocket.sendTXT(":: Initialize Canbus");
    }
    delay(1000);
    if (  CanReady) {
      CAN_Present = true;
      if (WS_alive) {
        webSocket.sendTXT("::  CAN bus ok!");
      }

    } else {
      if ( ESP32Can.CANInit() != 0) {
        CAN_Present = false;
        if (WS_alive) {
          webSocket.sendTXT("::  CAN bus NOT ok!");
        }
        Serial.println("CAN shield init Failed. Can connection present?");
      }
    }
    if (WS_alive) {
      webSocket.sendTXT("::  Initializing IMU!");
    }
    //====================================================================================
    //2. Initialize IMU & Barometer
    mpu.setup();
    boolean baro_init_code = bmp.begin();
    if (mpu.isConnectedMPU9250() && mpu.isConnectedAK8963() && baro_init_code) {
      IMU_Present = true;
      Serial.println("MPU 9250 ok!");
      if (WS_alive) {
        webSocket.sendTXT("::MPU 9250 ok!");
      }
    } else {
      IMU_Present = false;
      Serial.println("MPU 9250 not ok!");
      if (WS_alive) {
        webSocket.sendTXT("::MPU 9250 not ok!");
      }
    }
    //====================================================================================
    //3. Initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("SD card init Failed. SD card present?");
      SD_Present  = false;

      if (WS_alive) {
        webSocket.sendTXT("::SD card init Failed. SD card present?");
      }
    } else
    {
      SD_Present = true;
      Serial.println("SD Card initialized.");
      if (WS_alive) {
        webSocket.sendTXT("::SD Card initialized.");
      }

      uint8_t cardType = SD.cardType();

      if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        if (WS_alive) {
          webSocket.sendTXT("::No SD card attached.");
        }
        // return;
      }

      Serial.print("SD Card Type: ");
      if (WS_alive) {
        webSocket.sendTXT("::SD Card Type: ");
      }
      if (cardType == CARD_MMC) {
        Serial.println("MMC");
        if (WS_alive) {
          webSocket.sendTXT("::MMC");
        }
      } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
        if (WS_alive) {
          webSocket.sendTXT("::SDSC");
        }
      } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
        if (WS_alive) {
          webSocket.sendTXT("::SDHC");
        }
      } else {
        Serial.println("UNKNOWN");
        if (WS_alive) {
          webSocket.sendTXT("::SD Unknown");
        }
      }
    }
    //====================================================================================
    //4. Initialize GPS
    Serial.println("GPS is not initialized with a constructor. Its presence is monitored by monitoring the serial.");
    //On each MDW cysle the GPS is considered dead, unless it returns valid data in the meantime.
    //Each MDW cycle lasts 4 sec.
    //As there is no way to reboot the gps, the only thing to do is to log the error and wait for it to provide data.
    Serial.println("All initializations completed correctly. Let's measure!");
    if (WS_alive) {
      webSocket.sendTXT("::All initializations completed correctly. Let's measure!");
    }
    delay(2000);
    xTaskCreatePinnedToCore(
      &MEAS_routine, /* Function to implement the task */
      "MEAS", /* Name of the task */
      16386,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &MEAS,  /* Task handle. */
      0); /* Core where the task should run */
    Serial.println("Measurement thread created");
    if (WS_alive) {
      webSocket.sendTXT("::Measurement thread created");
    }
    delay(100);
    init_finished = true;
    Serial.println("Peripherals ok! Threads ok! Check to see if all remains ok from time to time...");
    if (WS_alive) {
      webSocket.sendTXT("::Peripherals ok! Threads ok! Check to see if all remains ok from time to time...");
    }
    espState = ss_Measure; //Peripherals ok! Threads ok! Check to see if all remains ok from time to time...
  }
  //=============================================================================================================System Checks for Peripheral Fallout During Measurements=========================================================================================
  if (espState == ss_Measure) {
    //Serial.println("Checking on peripherals...");
    //Check if all peripherals are still around. Restart where appropriate.
    //Check on IMU and restart
    if (!IMU_Present) {
      Serial.println("++++++++ IMU dissapeared !!! ++++++++");
      if (WS_alive) {
        webSocket.sendTXT("::++++++++ IMU dissapeared !!! ++++++++");
      }
    }

    //Check on SD and restart
    if (!SD_Present) {
      Serial.println("++++++++ SD dissapeared !!! ++++++++");
      if (WS_alive) {
        webSocket.sendTXT("::++++++++ SD dissapeared !!! ++++++++");
      }

      SD.end();
      delay(1000);
      if ( SD.begin(SD_CS_PIN)) {
        Serial.print("Trying to reconnect to SD on the go...");
        if (WS_alive) {
          webSocket.sendTXT("::  Trying to reconnect to SD on the go...");
        }
        Serial.println("...ok!");
        if (WS_alive) {
          webSocket.sendTXT("::  ...ok!");
        }
        SD_Present = true;
      } else {
        Serial.print("Trying to reconnect to SD on the go...");
        if (WS_alive) {
          webSocket.sendTXT("::  Trying to reconnect to SD on the go...");
        }
        Serial.println("...failed!");
        if (WS_alive) {
          webSocket.sendTXT("::  ...failed!");
        }
      }
    }
    //Check on CAN and restart
    if (!CAN_Present) {
      Serial.println("++++++++++ CAN dissapeared !!! +++++++");
      if (WS_alive) {
        webSocket.sendTXT("::  ++++++++++ CAN dissapeared !!! +++++++");
      }
      if ( ESP32Can.CANInit() == 0) {
        CAN_Present = true;
      } else {
        ESP32Can.CANStop();
        CAN_Present = false;
        Serial.println("CAN shield init Failed. Can connection present?");
        if (WS_alive) {
          webSocket.sendTXT("::  CAN shield Init failed. Can connection ok?");
        }
      }
    }
    //Check on GPS and restart
    if (!GPS_Present) {
      Serial.println("++++++++ GPS dissapeared !!! +++++++");
      if (WS_alive) {
        webSocket.sendTXT("::  GPS signal outage or cold start or frame not ready yet. C ya in a while");
      }
      //Signal outage or cold start. Be patient.
    }
    dataFile.flush();
    Serial.print("Current filename: ");    Serial.println(Filename);
    if (WS_alive) {
      webSocket.sendTXT("::  Current filename: " + String(Filename));
    }
    Serial.print("Current filesize: "); Serial.println(dataFile.size());
    if (WS_alive) {
      webSocket.sendTXT("::  Current filesize: " + String(dataFile.size()));
    }
    if (espState != ss_DataFlush) { //Refresh the file handle every now and then
      while (SDcritical && fileIsOpen) {
        delay(5);
      }
      if (SDcritical == false) {
        SDcritical = true;
        //Log Serial.println("Locking and closind SD file for maintenance. BRB!");
        dataFile.flush();
        Serial.printf("Filesize : %08d\r\n", dataFile.size());

        if (SDTotalCharWritten > 1024000) {
          SDTotalCharWritten = 0;
          dataFile.flush();
          dataFile.close();
          Serial.println("Changing File Counter.");
          if (WS_alive) {
            webSocket.sendTXT("::  Changing filecounter.");
          }

          char sdf[16];
          sprintf(sdf, "%02d%02d%02d%02d%02d", year, month, day, hour, minute);
          //Filename = "/" + String(sdf).substring(2, 10) + ".txt";
          fmc++;
          preferences.putUInt(fileCounter, (int)fmc);
          sprintf(Filename, "/%06d.txt", fmc); // set new filename
          //const char* string0 = Filename;
          dataFile = SD.open(Filename, FILE_APPEND);
          fileIsOpen = true;
          //Log Serial.println("File opened.");
          if (!dataFile) {
            fileIsOpen = false;
            //  Serial.println("Failed to open file for writing");
          }
        }
        SDcritical = false;

      }


      //Serial.println("Checking WiFi");
      espState = ss_Idle;
    } else
    {
      espState = ss_DataFlush;
    }
  }
  //=================================================================================================================================System OTA Update=============================================================================================================
  if (espState == ss_OTAUpdate) {
    MeasActive = false;
    measHolder[0] = '\0';
    if (WS_alive) {
      webSocket.sendTXT("::  OTA update requested.");
    }
    Serial.println("OTA update requested");
    PerformOTAupdate();  ///uncomment this to allow the OTA Update script to run when possible.
    Serial.println("This is gonna be a looooooooooooong integer winter....");//journey ends here. We are back on base, meas' were transfered and OTA ok.
    delay(10000);
    MeasActive = true;
    espState = ss_Idle;
  }
  //====================================================================================================================================System Meas Buffer Flush=====================================================================================================
  if (espState == ss_DataFlush) {
    MeasActive = false;
    measHolder[0] = '\0';
    if (WS_alive) {
      webSocket.sendTXT(":: Starting data buffer flushing.");
    }
    Serial.println("Starting data buffer flushing....");
    PerformDataFlush();
    // at this point we will open the sd in readonly mode, dump the data to the db and delete the file.
    delay(10000);
    MeasActive = true;
    espState = ss_Idle;
    //when finished go to idle

  }
  //=========================================================================================================================================System Idling=========================================================================================================
  if ((espState == ss_Idle) ) {
    //  1. ss_DataFlush,       //Transfer orphan data
    //  2. ss_OTAUpdate,       //OTA Update check
    //  3. ss_InitMeasure,     //Init meas system
    //  4. ss_Measure,         //Meas
    //  5. ss_Idle             //Check Wifi and select route

    Inbound = false;




    if (Inbound) { // close to the base
      MeasActive = false; //no need for meas
      //espState = ss_DataFlush; //OTA and Flush only on demand @ 18/10/2020
      Serial.println("  At base. Going for flush and OTA.");
      if (WS_alive) {
        webSocket.sendTXT("::  At base. Turning off the measuring stage. Letting Coms ON for commands.");
      }
    } else // if outbound init and perform measurements
    {

      if (!init_finished) {
        espState = ss_InitMeasure; // the system is in init stage. Lets finish this and we'll catch up again here in a while.
      } else {
        MeasActive = true;
//        if (WS_alive) {
//          long ss = WiFi.RSSI();
//          webSocket.sendTXT(":: WIFI Station: " + String(WiFi.SSID()) + " Signal:  " + String(ss));
//        }
        espState = ss_Measure;// if the system has performed the init, then its ok to start or continue the peasurement sequence

        //Set watchdog flag for GPS
        Serial.print("WS_timeout: "); Serial.println(WS_timeout);
        //  Serial.print("WS_throttle: "); Serial.println(WS_throttle);
        saTS = millis() - saTS;
        float speeds = 1000.0001 * (sent + sdwr) / saTS;
        float baoudrate = (1.0001 * saChars) / saTS;
        saTS = millis(); sent = 0; sdwr = 0;
        Serial.print("Posting / Writing @ "); Serial.print(speeds, 3); Serial.println("Hz");

        Serial.print("Baud Rate: "); Serial.print(baoudrate, 3); Serial.println("KBps");

        Serial.print("Free memory: "); Serial.println(ESP.getFreeHeap());

        Serial.print("Sending to ");

        if (WS_alive) {
          Serial.println("Websocket.");
        } else {
          Serial.println("SD Card.");
        }
        if (speeds < 1) {
          WS_timeout++;
        }
        if (WS_timeout > 3) {
          //    webSocket.disconnect();
          Serial.println("Reboot requested");
          //ESP.restart();
          // WS_alive = false;
          WS_timeout = 0;
        }

        saChars = 0;
        Serial.println("::Room service passed from here::");
        if (WiFiMulti.run() != WL_CONNECTED) {
          delay(500);
          Serial.println("Lost WiFi. Retrying to connect on the go...");
        }

        /*
          if (WS_timeout > 1000) { // the first time it will pass through by default. The next time, data must be present or ws will reboot.
          Serial.println("Huston we have a problem. WS is dead.");
          client.flush();
          webSocket.disconnect();
          webSocket.begin(server, 8089, "/");

          // event handler
          webSocket.onEvent(webSocketEvent);

          // use HTTP Basic Authorization this is optional remove if not needed
          //webSocket.setAuthorization("user", "Password");

          // try ever 5000 again if connection has failed
          webSocket.setReconnectInterval(1000);

          WS_timeout = 0;
          return;
          //  WiFi.disconnect();
          Serial.println("WS disconnected. Going for restart...");

          //  ESP.restart();
          }*/

      }

    }

  }
  SyncDateTime();
  delay(2000);
  mpuReady = true;
  /*auto fallback
      WS_Use_IMU = true;
      WS_Use_CAN = true;
      WS_Use_GPS = true;
      WS_Use_TIM = true;*/

}///mdw routine




void PerformOTAupdate() {
  Serial.println("Starting OTA update....");
  if (WS_alive) {
    webSocket.sendTXT("::  ---------------------------OTA Started----------------------------");
    webSocket.sendTXT(String(WiFi.SSID()).c_str());
    delay(10);
    long ss = WiFi.RSSI();
    webSocket.sendTXT(String(ss).c_str());
  }


  MeasActive = false;
  delay(1000);
  //========Starting Critical OTA Update Sequence===============
  unsigned int fmwv = (int) preferences.getUInt(softwareVersion, 0); //read the latest version number of currently installed firmware

  char CurrentVersion[18];
  char NextVersion[18];
  sprintf(CurrentVersion, "fmw_%08X.bin", (int)fmwv);
  String filebin = String(server) + String("/fmw/") + String(CurrentVersion);
  fmwv++;
  Serial.print("Current firmware version: ");
  Serial.println(CurrentVersion);
  Serial.print("New firmware version to be downloaded: ");
  sprintf(NextVersion, "fmw_%08X.bin", (int)fmwv);
  Serial.println(NextVersion);
  //filebin = String("http://") + String(server) + String(":8118/fmw/") + String(NextVersion);
  filebin = String("http://") + String(server) + String(":8118/fmw/ota.bin");
  Serial.println(filebin);
  preferences.putUInt(softwareVersion, (int)fmwv);
  delay(1000);
  Serial.println(" Http client ready.");
  digitalWrite(13, HIGH);
  delay(4444);
  int outcome = 0;
  t_httpUpdate_return ret = HTTP_UPDATE_FAILED;
  while (ret == HTTP_UPDATE_FAILED) {
    ret = httpUpdate.update(client, filebin);
    delay(1000);
    outcome++;
    if (outcome > 5) {
      Serial.println("Tried repeatedly to establish a connection to no avail. Aborting");
     webSocket.sendTXT("Aborting OTA due to bad connection conditions.");
      break;
    }
  }
  Serial.println("Finished OTA attempt.");
  if (WS_alive) {
    webSocket.sendTXT("::  Finished OTA attempt.");
  }
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
      digitalWrite(13, LOW);
      sprintf(CurrentVersion, "fmw_%08X.bin", (int)fmwv);
      filebin = String(server) + String("/fmw/") + String(CurrentVersion);
      fmwv--;
      Serial.println(filebin);
      delay(100);
      preferences.putUInt(softwareVersion, (int)fmwv);//<<< Update failed. Restore the previous version number in preferences, so that next time the correct version number will be requested   .
      if (WS_alive) {
        webSocket.sendTXT("::  OTA update failed. See log for further details.");
      }
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("HTTP_UPDATE_NO_UPDATES");
      digitalWrite(13, LOW);
      sprintf(CurrentVersion, "fmw_%08X.bin", (int)fmwv);
      filebin = String(server) + String("/fmw/") + String(CurrentVersion);
      fmwv--; //if update failed, return to the previous version
      Serial.println(filebin);
      delay(100);
      preferences.putUInt(softwareVersion, (int)fmwv);//<<< Update failed. Restore the previous version number in preferences.
      if (WS_alive) {
        webSocket.sendTXT("::  NO OTA");
        webSocket.sendTXT("::  OTA=" + String(fmwv) + "--");
      }
      break;

    case HTTP_UPDATE_OK:
      Serial.println("HTTP_UPDATE_OK");
      if (WS_alive) {
        webSocket.sendTXT("::  OTA update OK!");
      }
      digitalWrite(13, LOW);
      delay(1000);
      break;
  }
  digitalWrite(13, LOW);
  client.stop();
}


void PerformDataFlush() {
  MeasActive = false;
  if (!dataflushStarted) {
    if (WS_alive) {
      webSocket.sendTXT("::  ----------------------Flush----------------------");
      webSocket.sendTXT(String(WiFi.SSID()).c_str());

      delay(10);
      long ss = WiFi.RSSI();
      webSocket.sendTXT(String(ss).c_str());
      webSocket.sendTXT(String(fmc).c_str());
    }

    WiFiClient FSclient;
    dataflushStarted = true;
    dataflush_active = true;
    Serial.println("Starting SD flush to database....");

    int trsfd = 0;

    MeasActive = false;
    delay(1000);

    trsfd = 0;
    while (SDcritical) {
      delay(200);
      trsfd++;
      if (trsfd > 10000) {
        SDcritical = false;
      }
    }
    int waitingTime = 30000;
    long op = millis();
    trsfd = 0;
    Serial.println("Started reading SD files");
    SDcritical = true;
    //Log Serial.println("Locking and closind SD file for maintenance. BRB!");
    if (fileIsOpen) {
      dataFile.flush();
      dataFile.close();
      //Serial.println("File closed.");
      //    fileIsOpen = false;
    }
    Serial.println("Connecting to " + String(server));
    // webSocket.disconnect();

    delay(2222);
    {
      UDHttp udh;
      while (fmc > 0) {/// fmc: file manager counter>> current file being processed
        char Filename[12];
        sprintf(Filename, "/%06d.txt", fmc);

        Serial.printf("Processing file: %s\n\r", Filename);
        if (WS_alive) {
          webSocket.sendTXT("::  Processing file: " + String(Filename));
        }
        //open file on sdcard to read
        if (SD.exists(Filename)) {
          root = SD.open(Filename);
          if (!root) {
            Serial.println("can not open file!");
            return;
          }
          Serial.println("So far so good");
          //upload downloaded file to local server
          int outcome = -1;
          while (outcome != 0) {
            outcome = udh.upload("http://fortion.hopto.org:8118/fmw/flushSD.php", Filename, root.size(), rdataf, progressf, responsef);
            delay(200);
            client.flush();
            client.stop();
          }
        } else {
          root.close();
          if (SD.exists(Filename)) {
            Serial.printf("Deleting file...%s\n\r", Filename);
            if (WS_alive) {
              webSocket.sendTXT("::  Deleting file: " + String(Filename));
            }
            SD.remove(Filename);

          }

        } fmc--; preferences.putUInt(fileCounter, (int)fmc); // decrement current file number and save it to memory just in case of brownout
      }

      Serial.println("done! Uploading finished.");
      if (WS_alive) {
        webSocket.sendTXT("::  Uploading finished!");
      }
    }
  }
}

int responsef(uint8_t *buffer, int len) {
  Serial.printf("%s\n", buffer);
  return 0;
}
void progressf(int percent) {
  prog++;
  if (prog == 200) {
    Serial.println("Flush " + String(percent) + "%");
    if (WS_alive) {
      webSocket.sendTXT("::  Flush " + String(percent) + "%");
    }

    prog = 0;
  }
}

int rdataf(uint8_t *buffer, int len) {
  //read file to upload
  if (root.available()) {
    return root.read(buffer, len);
  }
  return 0;
}
String urlencode(String str)
{
  String encodedString = "";
  char c;
  char code0;
  char code1;
  char code2;
  for (int i = 0; i < str.length(); i++) {
    c = str.charAt(i);
    if (c == ' ') {
      encodedString += '+';
    } else if (isalnum(c)) {
      encodedString += c;
    } else {
      code1 = (c & 0xf) + '0';
      if ((c & 0xf) > 9) {
        code1 = (c & 0xf) - 10 + 'A';
      }
      c = (c >> 4) & 0xf;
      code0 = c + '0';
      if (c > 9) {
        code0 = c - 10 + 'A';
      }
      code2 = '\0';
      encodedString += '%';
      encodedString += code0;
      encodedString += code1;
      //encodedString+=code2;
    }
    yield();
  }
  return encodedString;
}



/*
//prev version with consant file name
          Serial.print("Waiting for websocket response.");
          while (!WS_alive) {
            delay(10000);
            //webSocket.disconnect();
            Serial.print(".");
          }
          Serial.print("Websocket ready!");
          root = SD.open("/orphan.txt");
          if (!root) {
            Serial.println("Cann't open file! Aborting flush!");
            return;
          }
          while (root.available()) {
            String abuffer = root.readStringUntil('\n');
            if (WS_alive) {
              webSocket.sendTXT(abuffer + '\n');
              Serial.print(">");
              delay(20);
            } else {

              Serial.print("Waiting for websocket response.");
              while (!WS_alive) {
                delay(10000);
                webSocket.disconnect();
                Serial.print(".");
              }


*/
