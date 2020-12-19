//void BLE_routine( void * parameter) {
//  //this thread runs parallel function for non blocking purposes
//  //1. it runs the gps serial char ripper
//  //2.
//  for (;;) {
//    while (!BTPLready) {
//      delay(50);
//    }
//
//    if (strlen(measHolder) > 1000) {
//
//      SerialBT.write(BTPayload, (size_t)(strlen(measHolder)));
//      delay(10);
//      SerialBT.flush();
//      BTPLready = false;
//    }
//    delay(50);
//  }
//}

void HLP_routine( void * parameter) {
  //this thread runs parallel function for non blocking purposes
  //1. it runs the gps serial char ripper
  //2.
  for (;;) {
    if (mpuReady && init_finished) {

      //if (!freshIMU) {
      mpu.update();// cost 7~11 ms

      //DO NOT cross down from 10millis. It will become unstable!!! Usuable imu freq=20. Scan higher than this frq will not produce better results.
      //So a 40 millis delay is ok
      delay(25);//WARNING!!!  DO NOT cross down from 50 millis. It will become unstable!!! Usuable imu freq=20. So a 50 down to 40 millis delay is ok
      //88 millis>> ~10Hz
      for (int i = 0; i < 4; i++) {
        //     quart[i] = mpu.getQuaternion(i);
        fquart[i] = (fbquart[i] * mpu.getQuaternion(i)  + (1 - fbquart[i]) * fquart[i]);
      }
      for (int i = 0; i < 3; i++) {
        //      accel[i] = mpu.getAcc(i);
        //      gyros[i] = mpu.getGyro(i);
        //      magne[i] = mpu.getMag(i);
        faccel[i] = (fbaccel[i] * mpu.getAcc(i)         + (1 - fbaccel[i]) * faccel[i]);
        fgyros[i] = (fbgyros[i] * mpu.getGyro(i)        + (1 - fbgyros[i]) * fgyros[i]);
        fmagne[i] = (fbmagne[i] * mpu.getMag(i)         + (1 - fbmagne[i]) * fmagne[i]);
      }
      pressure      = 0.88 * pressure    + 0.12 * bmp.readPressure();
      temperature   = 0.88 * temperature + 0.12 * bmp.readTemperature();
      altimeter     = 0.88 * altimeter   + 0.12 * bmp.readAltitude (1050.35);

      pressure      = 0.88 * pressure     + 0.12 * pressure;
      freshIMU = true;
      //      } else {
      //        delay(5);
      //      }
    }

    while (ss.available())// cost roughly 1 ms
    {

      char c = ss.read();
      //Serial.print(c);
      //      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
      GPS_Present = true;
    }

  }
}



void MEAS_routine( void * parameter) {
  for (;;) {
    //=============================================================================can measurements================================================================================
    if (WS_Use_CAN) {
      if (CAN_Present) {
        int tp = 0; //1=ext, 0 =std
        int rtr = 0; //1=rtr, 0 =nrm
        CAN_frame_t rx_frame;
        // Receive next CAN frame from queue
        if (xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3 * portTICK_PERIOD_MS) == pdTRUE) {
          unsigned long mc = micros();
          double d = 0.0009999 * ((mc - lastEntryTime) / 999.9999);
          lastNTPtime = lastNTPtime + d;
          lastEntryTime = mc;
          if (rx_frame.FIR.B.FF == CAN_frame_std) {
            tp = 0; //type = Standard
          } else
          {
            tp = 1; //type = Extended
          }
          if (rx_frame.FIR.B.RTR == CAN_RTR)       {
            rtr = 1; //Request to Receive
          } else
          {
            rtr = 0; //Normal
          }
          if (broadacstCANtowebpage) {
            char Scan[512];
            sprintf(Scan, "A#\t%03d\t%03d\t%.3f\tCAN\t%d\t%d\t0x%08X\t%d\t%02X%02X%02X%02X%02X%02X%02X%02X", VID, DID, lastNTPtime, tp, rtr, rx_frame.MsgID, rx_frame.FIR.B.DLC, rx_frame.data.u8[0], rx_frame.data.u8[1], rx_frame.data.u8[2], rx_frame.data.u8[3], rx_frame.data.u8[4], rx_frame.data.u8[5], rx_frame.data.u8[6], rx_frame.data.u8[7]);
            //
            broadacstCANtowebpageCount++;
            if (broadacstCANtowebpageCount > 200) {
              broadacstCANtowebpageCount = 0;
              broadacstCANtowebpage = false;
              WS_Use_IMU = true;
              WS_Use_CAN = true;
              WS_Use_GPS = true;

            }
          }
          char Scan[512];
          sprintf(Scan, "D#\t%03d\t%03d\t%.3f\tCAN\t%d\t%d\t0x%08X\t%d\t%02X%02X%02X%02X%02X%02X%02X%02X", VID, DID, lastNTPtime, tp, rtr, rx_frame.MsgID, rx_frame.FIR.B.DLC, rx_frame.data.u8[0], rx_frame.data.u8[1], rx_frame.data.u8[2], rx_frame.data.u8[3], rx_frame.data.u8[4], rx_frame.data.u8[5], rx_frame.data.u8[6], rx_frame.data.u8[7]);
          //vid,did,date,Type,RTR,SRC,MSGID,TRG,DLC,Q1Q2Q3Q4Q5Q6Q7Q8
          //Serial.println(Scan);
          //    sprintf(Scan, "D#\t%03d\t%03d\t%.3f\tCAN\t0C\tF004\t00\t8\tFED7D112233AAFF", VID, DID, lastNTPtime);

          saChars += strlen(Scan);
          if (measurement_cnt > 0) {
            strcat(measHolder, "\r\n");
          }
          measurement_cnt++;
          if (MeasActive) {
            if (strlen(measHolder) + strlen(Scan) < sizeof(measHolder)) {
              strcat(measHolder, Scan);
            }
          }
          Scan[0] = '\0';
        }
        else
        {
          //          char Scan[512]; //DemoData
          //          unsigned long mc = micros();
          //          double d = 0.0009999 * ((mc - lastEntryTime) / 999.9999);
          //          lastNTPtime = lastNTPtime + d;
          //          lastEntryTime = mc;
          //          sprintf(Scan, "D#\t%03d\t%03d\t%.3f\tCON\t0C\tF004\t00\t8\tFED7D112233AAFF", VID, DID, lastNTPtime);
          //          saChars += strlen(Scan);
          //          if (measurement_cnt > 0) {
          //            strcat(measHolder, "\r\n");
          //          }
          //          measurement_cnt++;
          //          strcat(measHolder, Scan);
          //          Scan[0] = '\0';
          delay(1);

        }

      }
      else
      {
        delay(1);
      }//if can present
    }
    //=============================================================================imu measurements================================================================================
    //   WS_Use_IMU=false;
    if (WS_Use_IMU) {
      //  mpu.update();
      if (freshIMU) {
        unsigned long mc = micros();
        double d = 0.0009999 * ((mc - lastEntryTime) / 999.9999);
        lastNTPtime = lastNTPtime + d;
        lastEntryTime = mc;

        char Scan[512];
        sprintf(Scan, "D#\t%03d\t%03d\t%.3f\tIMU\t%.2f\t%.2f\t%.2f\t%.2f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.1f\t%.2f", VID, DID, lastNTPtime, fquart[0], fquart[1], fquart[2], fquart[3], faccel[0], faccel[1], faccel[2], fgyros[0], fgyros[1], fgyros[2], fmagne[0], fmagne[1], fmagne[2], pressure, temperature, altimeter);
        if (measurement_cnt > 0) {
          strcat(measHolder, "\r\n");
        }
        measurement_cnt++;
        saChars += strlen(Scan);
        if (MeasActive) {
          if (strlen(measHolder) + strlen(Scan) < sizeof(measHolder)) {
            strcat(measHolder, Scan);
          }
        }
        Scan[0] = '\0';
        freshIMU = false;
      }
      else {
        delay(1);//yield a little
      }
    }
    //=========================================================================gps meas======================================================
    if (newData)  {

      GPS_Present = true;





      unsigned long age, ml;
      ml = millis() % 1000;
      gps.f_get_position(&flat, &flon, &age);

      falt = 0.0099999 * gps.altitude();
      fcrs = 0.00099999 * gps.course();
      fspd =  gps.f_speed_kmph();
      unsigned long mc = micros();
      double d = 0.0009999 * ((mc - lastEntryTime) / 999.9999);
      lastNTPtime = lastNTPtime + d;
      lastEntryTime = mc;
      char Scan[512];
      sprintf(Scan, "D#\t%03d\t%03d\t%.3f\tGPS\t%.8f\t%.8f\t%.3f\t%.3f\t%.3f", VID, DID, lastNTPtime, flat, flon, falt, fcrs, fspd);
      float dd = sqrt(sq(flat - 38.2098621) + sq(flon - 21.7327818)); //fortion
      // float dd=sqrt(sq(flat-37.910039)+sq(flon-23.737302));     //home

      if (espState == ss_Measure) {

        Inbound = dd < 0.1 ? true : false; // ca 750m around the base
      } else {
        Inbound = false;
      }
      if (!MeasActive) {
        Inbound = false; //fake news. But its ok!
      }
      //  Serial.print("Distance from base: ");Serial.println(dd);
      //We are near the base wifi, stop measuring. No meas needed. Just OTA and flush here.
      newData = false;

      if (WS_Use_GPS)  {
        if (measurement_cnt > 0) {
          strcat(measHolder, "\r\n");
        }
        measurement_cnt++;
        saChars += strlen(Scan);
        if (MeasActive) {
          if (strlen(measHolder) + strlen(Scan) < sizeof(measHolder)) {
            strcat(measHolder, Scan);
          }
        }
      }
      Scan[0] = '\0';
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths , &fix_age);
      hour+=3;
      if (hour>24) {
        hour-=24;
        day+=1;
        
      }
      if (!HasBeenSynced) {
          lastNTPtime= 
          10000000000*year+
            100000000*month+
              1000000*day+
                10000*hour+
                  100*minute+
                    1*second;

        lastEntryTime = micros();
        HasBeenSynced=true;
      }


    }
    if (measurement_cnt > 19) { // store up to 20 lines of meas, then try to send them. In general, every each sec 4 packets of 20 lines go out. That is about 80Hz nominal update rate.
      sent += measurement_cnt;
      measurement_cnt = 0;
      //   uint8_t   BTmeasHolder;

      //    memcpy ( &BTmeasHolder, &measHolder, sizeof(measHolder) );

      //        for (int y = 0; y < 2160; y++) {
      //          BTPayload[y] = (uint8_t)measHolder[y];
      //          if (measHolder[y] == '\0') {
      //            break;
      //          }
      //
      //        }
      //        BTPLready = true;
      //    Serial.printf("SSS>>>%lu \n\r",millis());
      //    SerialBT.write(BTPayload,(size_t)(strlen(measHolder)));
      //    delay(1);
      //SerialBT.flush();
      //Serial.printf("SSS>>>%lu \n\r",millis());
      //    SerialBT.write((uint8_t)"falksjdfhalksjdhfla jsd fhlakjsd hflakjsd flhakjsdfhla ksdjfhlakjd fhlakjdhf lajksd hflakjsd hflajksd fhakjsdflhajksd fhlajk",125);
      //    Serial.println(BTmeasHolder);
      //    Serial.println(String(strlen(measHolder)));
      if (strlen(measHolder) >= sizeof(measHolder)) {
        Serial.println("Sring overflow!!!");
        measHolder[0] = '\0';
      }

      if (MeasActive) {
        if (WS_alive) {
          digitalWrite(13, HIGH);
          sdwr++;

          dataFile.println(measHolder);

          SDcritical = false;
          if (!Inbound) {
            webSocket.sendTXT(measHolder);
          }
          else {
            webSocket.sendTXT("######");
            delay(15000);
          }
          digitalWrite(13, LOW);
          measHolder[0] = '\0';
        } else {
          //the websocket is not available, let's dump the data to the SD
          int t0 = millis();
          SDcritical = true;
          //Serial.println(measHolder);
          if (SD_Present) {
            // Serial.println("Writing to file ...");
            digitalWrite(13, HIGH);
            sdwr++;
            dataFile.println(measHolder);
            SDcritical = false;
            digitalWrite(13, LOW);
            t0 = millis() - t0;
            SDTotalCharWritten += strlen(measHolder);
            float cps;
            if (t0 > 0) {
              cps = 1.0000001 * strlen(measHolder) / t0;
            } else {
              cps = 0;
            }
            // Serial.printf("Writing to SD @ %.2f KBps\n",cps);
          } else {
            SDTotalCharWritten = 0;
          }
          measHolder[0] = '\0';
        }
        // Serial.println(measHolder);//uncomment to see payload
        //Serial.println(strlen(measHolder));//uncomment to see payload length
        measHolder[0] = '\0';
      }
      else {
        measHolder[0] = '\0';
        delay(1000);
      }  //Idling when close to base. No need to measure.
    }
    delay(1);
  }//for
}



void SyncDateTime() {

  if (!HasBeenSynced) {
    configTime(0, 0, NTP_SERVER);
    setenv("TZ", TZ_INFO, 1);
    if (getNTPtime(10)) {  // wait up to 10sec to sync
      delay(10);
    } else {
      Serial.println("Time not set");
      //  ESP.restart();
    }
    //showTime(timeinfo);
    lastNTPtime = (double)(

          10000000000*(timeinfo.tm_year - 100)+
            100000000*(timeinfo.tm_mon + 1)+
              1000000*timeinfo.tm_mday+
                10000*timeinfo.tm_hour+
                  100*timeinfo.tm_min+
                    1*timeinfo.tm_sec );


  


    
    //lastNTPtime = (double)time(&now); // this brings the raw unix time
    lastEntryTime = micros();
    //  getNTPtime(10);
    //  showTime(timeinfo);
    delay(10);
    HasBeenSynced = true;
  } else {
    // showTime(timeinfo);
  }
}



bool getNTPtime(int sec) {

  {
    uint32_t start = millis();
    do {
      time(&now);
      localtime_r(&now, &timeinfo);
      //Serial.print(".");
      delay(10);
    } while (((millis() - start) <= (1000 * sec)) && (timeinfo.tm_year < (2010 - 1900)));
    if (timeinfo.tm_year <= (2010 - 1900)) return false;  // the NTP call was not successful
    Serial.print("Now :");  Serial.println(now);
    char time_output[30];
    strftime(time_output, 30, "%a  %d-%m-%y %T", localtime(&now));
    // Serial.println(time_output);
    // Serial.println("..............................................................................................");
  }
  return true;
}

void showTime(tm localTime) {
  Serial.print(localTime.tm_mday);
  Serial.print('/');
  Serial.print(localTime.tm_mon + 1);
  Serial.print('/');
  Serial.print(localTime.tm_year - 100);
  Serial.print('-');
  Serial.print(localTime.tm_hour);
  Serial.print(':');
  Serial.print(localTime.tm_min);
  Serial.print(':');
  Serial.println(localTime.tm_sec);
  Serial.println(millis() - lastEntryTime);
  Serial.println(" Day of Week ");
  if (localTime.tm_wday == 0) {
    Serial.println(7);
  }
  else
  { Serial.println(localTime.tm_wday);
  }
}
