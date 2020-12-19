void WBS_routine( void * parameter) {
  for (;;) {
    if (WiFi.status() == WL_CONNECTED) {
      webSocket.loop();
    } else {
      delay(10);
    }
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {


  switch (type) {
    case WStype_DISCONNECTED:
      WS_alive = false;
      Serial.printf("[WSc] Disconnected event!\n");
      break;
    case WStype_CONNECTED:
      { Serial.printf("[WSc] Connected to url: %s\n", payload);
        //  webSocket.sendTXT("A Can Answer");
        // send message to server when Connected
        WS_alive = true;

      }
      break;
    case WStype_TEXT:
      {
        sent--;
        //   Serial.printf("[WSc] get text: %s\n", payload);
        char sg[length + 1];
        sprintf(sg, "%s", payload);
        String sgg = String(sg);
        if (sgg.charAt(0) == 'D') {
          WS_timeout = 0;
          /*
                    WS_TSR = sgg.substring(1).toInt();
                 //   Serial.println(WS_TSR);
                    for (int z = 0; z < 500; z++) {
                      Serial.print(z);Serial.print(" -  ");Serial.print(SD_pool[z].TS);Serial.print(" -  ");Serial.println(WS_TSR);
                      if (SD_pool[z].TS == WS_TSR) {
                        SD_pool[z].DT = 0; //the dirty field marks the line as transfered, so it can be reused / overwritten by the next data line
                        break; //guilty found! Lets break to make it a bit faster.
                      }
                    }
          */
          /*
            if (WS_TS - WS_TSR > 200) { //Ws feedback is out of sync with current line by at least 200ms. Lets save it in the SD...
             if (SD_Present && fileIsOpen) {
               //CAUTION!!!!! At this point the SD is occupied. Each peripheral, on its own thread, may try to acces the SD while in use by other thread.
               //To avoid race conditions and bottle necks, each peripheral uses queues to temporarily store meas's that cannot be relaied directly to the SD.
               //When the SD becomes available, all the buffered meas's are written to SD and the queues are emptied. Timestamps of queued lines may be seconds earlier then the current one.
               //This will be sorted in the db transfer.
               digitalWrite(13, HIGH);
               const char* stringline = WS_msg.c_str();
               SDcritical = true;
               Serial.print("SD write. WS_TSR= "); Serial.print(WS_TSR); Serial.print(" , while WS_TS= "); Serial.print(WS_TS);

               dataFile.println(stringline);
               SDcritical = false;
               digitalWrite(13, LOW);
             }
            }*/
          //  Serial.print("Received TS from WS: "); Serial.println(WS_TSR);
        }



        //================================================
        //Special Control Codes
        //================================================
        /*
           M: toggles the IMU (only in printing mode, it does not shut it down completely)
           G: toggles the GPS (only in printing mode, it does not shut it down completely)
           C: toggles the CAN (only in printing mode, it does not shut it down completely)
           T: toggles the Time output  (only in printing mode, it does not shut it down completely)
           R: (followed by RTR frame code) Shuts all but the can and requests the RTR frame. Allows 1000 CAN frames to pass through the WS to the webpage and then resumes previous state.
                                           If in this time window no frame with this RTR id is received, that means that the CAN bus ignored the request.
           S: Esp.restart. Use with caution.
           F: SD flush. Turns the esp in data unload mode. All meas systems stop and the connection is used to upload the SD orphan measurements to database.
           O: OTA update. Enforce immediate update.
        */
        if (sgg.charAt(0) == 'R') {//R
          if (sgg.charAt(1) == 'M') {

            WS_Use_IMU = ! WS_Use_IMU;
            WS_Use_IMU ? Serial.println("[WSc] IMU ON ") : Serial.println("[WSc] IMU OFF ");
          }
          if (sgg.charAt(1) == 'G') {
            WS_Use_GPS = ! WS_Use_GPS;
            WS_Use_GPS ? Serial.println("[WSc] GPS ON ") : Serial.println("[WSc] GPS OFF ");
          }
          if (sgg.charAt(1) == 'N') {
            WS_Use_CAN = ! WS_Use_CAN;
            WS_Use_CAN ? Serial.println("[WSc] CAN ON ") : Serial.println("[WSc] CAN OFF ");
          }
          if (sgg.charAt(1) == 'L') {
            WS_Use_TIM = ! WS_Use_TIM;
            WS_Use_TIM ? Serial.println("[WSc] TIM ON ") : Serial.println("[WSc] TIM OFF ");
          }
          if (sgg.charAt(1) == 'S') {
            Serial.println("[WSc] ESP restart ");
            ESP.restart();
          }
          if (sgg.charAt(1) == 'U') {
            Serial.println("[WSc] Data flush enqueued.");
            espState = ss_DataFlush;
            // PerformDataFlush();
          }
          if (sgg.charAt(1) == 'O') {
            Serial.println("[WSc] OTA");
            espState = ss_OTAUpdate;
          }

        }



        //Serial.print("Size o: ");Serial.println(WS_size);
        // Serial.print("Size b: ");Serial.println(sz);

        //        Serial.println(sg);
        //        Serial.println(sgg.charAt(0));
        if (sgg.charAt(0) == 'Z') {
          Serial.print("[WSc] can request: ");
          String WS_Responce = sgg;
          awaitingWS = false;
        }
        if (sgg.charAt(1) == 'R') {
          Serial.print("[WSc] can request: ");
          WS_Use_IMU = false;
          WS_Use_CAN = true;
          WS_Use_GPS = false;
          WS_Use_TIM = false;


          awaitingWS = false;
          // webSocket.sendTXT("Answer: Here follows 1000 concecutive messages of CAN only");

          Serial.print("Emmiting CANBUS RTR: ");
          uint16_t addr = (uint16_t)hstol(sgg.substring(2)); // remove the first two chars or else the parse will fail. The first char is the request prefix, the second is the request type (RTR).
          Serial.printf("0x%06X\n", addr);
          CAN_frame_t tx_frame;
          tx_frame.FIR.B.FF = CAN_frame_std;
          tx_frame.MsgID = addr;//<<This is where the picked up from the websocket message request goes in.
          tx_frame.FIR.B.DLC = 0;
          //vvvvvv data is not needed actually.
          tx_frame.data.u8[0] = 0x00;
          tx_frame.data.u8[1] = 0x00;
          tx_frame.data.u8[2] = 0x00;
          tx_frame.data.u8[3] = 0x00;
          tx_frame.data.u8[4] = 0x00;
          tx_frame.data.u8[5] = 0x00;
          tx_frame.data.u8[6] = 0x00;
          tx_frame.data.u8[7] = 0x00;
          ESP32Can.CANWriteFrame(&tx_frame);//uncomment this to allow custom can msgs to be delivered
          awaitingWS = false;//<<Dont block the ws while receiveing custom can rtr's.
          //set flag that a special can request was emmited
          broadacstCANtowebpage = true;
          Serial.print("Special CAN request received from web page.\n");
        }

        // send message to server
        // webSocket.sendTXT("message here");
      }
      break;
    case WStype_BIN:
      { Serial.printf("[WSc] get binary length: %u\n", length);
        //   hexdump(payload, length);

        // send data to server        webSocket.sendBIN(payload, length);
      }
      break;
    case WStype_ERROR: {}  break;
    case WStype_PING: {} break;
    case WStype_PONG: {} break;
    case WStype_FRAGMENT_TEXT_START: {} break;
    case WStype_FRAGMENT_BIN_START: {} break;
    case WStype_FRAGMENT: {} break;
    case WStype_FRAGMENT_FIN: {} break;
      break;
  }

}
long hstol(String recv) {
  char c[recv.length() + 1];
  recv.toCharArray(c, recv.length() + 1);
  return strtol(c, NULL, 16);
}
String SYSuuid() {
  uint64_t chipid = ESP.getEfuseMac();
  char uid[7];
  sprintf(uid, "%X", (uint32_t)chipid);
  return String(uid);
}
