/**
  ******************************************************************************
  * @file    PEGremsy_FireBase.cpp
  * @author  Gremsy Team
  * @version v100
  * @date    2021
  * @brief   This file contains all the functions prototypes for the  
  *          firmware library.
  *
  ************************************************************
  ******************
  * @par
  * COPYRIGHT NOTICE: (c) 2011 Gremsy.
  * All rights reserved.Firmware coding style V1.0.beta
  *
  * The information contained herein is confidential
  * property of Company. The use, copying, transfer or
  * disclosure of such information is prohibited except
  * by express written agreement with Company.
  *
  ******************************************************************************
*/
/* Includes------------------------------------------------------------------------------*/
#include "PEGremsy_FireBase.h"
#include "Arduino.h"
#if defined(ESP32)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif
#include <Firebase_ESP_Client.h>

//Provide the token generation process info.
#include <addons/TokenHelper.h>
/* Private typedef------------------------------------------------------------------------------*/
/* Private define------------------------------------------------------------------------------*/
/* 1. Define the WiFi credentials */
#define WIFI_SSID "NST"
#define WIFI_PASSWORD "hoilamchido"

/* 2. Define the API Key */
#define API_KEY "AIzaSyBGvx_V92-TLHZ5wIzUWs02mO8zoOc0nHI"

/* 3. Define the project ID */
#define FIREBASE_PROJECT_ID "fir-storedemo-c2217"

/* 4. Define the user Email and password that alreadey registerd or added in your project */
#define USER_EMAIL "nguyenvankinh12@gmail.com"
#define USER_PASSWORD "nsttgd12"
/* Private macro------------------------------------------------------------------------------*/
/* Private variables------------------------------------------------------------------------------*/
//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long dataMillis = 0;
int count = 0;
/* Private function prototypes------------------------------------------------------------------------------*/
/* Private functions------------------------------------------------------------------------------*/
PEGremsy_FireBase_t::PEGremsy_FireBase_t(/* args */)
{
}

PEGremsy_FireBase_t::~PEGremsy_FireBase_t()
{
}

/** @group FIREBASE_INITIALIZE
    @{
*/#ifndef FIREBASE_INITIALIZE
#define FIREBASE_INITIALIZE
/** @brief 
    @return 
*/
void PEGremsy_FireBase_t::initialize(void)
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
    }
    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the api key (required) */
    config.api_key = API_KEY;

    /* Assign the user sign in credentials */
    auth.user.email = USER_EMAIL;
    auth.user.password = USER_PASSWORD;

    /* Assign the callback function for the long running token generation task */
    config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h

    Firebase.begin(&config, &auth);
    
    Firebase.reconnectWiFi(true);
}

#endif
/**
    @}
*/

/** @group FIREBASE_PROCESS
    @{
*/#ifndef FIREBASE_PROCESS
#define FIREBASE_PROCESS
/** @brief 
    @return 
*/
void PEGremsy_FireBase_t::process(void)
{
    static uint8_t numberOfMap = 0;

    if (Firebase.ready())// && (millis() - dataMillis > 20000))
    {
        dataMillis = millis();

        FirebaseJson content;

        //We will create the nested document in the parent path "a0/b0/c0
        //a0 is the collection id, b0 is the document id in collection a0 and c0 is the collection id in the document b0.
        //and d? is the document id in the document collection id c0 which we will create.
        String documentPath = "a0/b0/c0/d" + String(count);

        count++;

        content.set("fields/" + String(numberOfMap) +"/mapValue/fields/name/stringValue", "wrench");

        Serial.print("Create a document... ");

        if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw()))
        {
            Serial.printf("ok\n%s\n len : %d\n", fbdo.payload().c_str(), fbdo.payload().length());
        } 
        else
        {
            Serial.println(fbdo.errorReason());
        }   
        delay(500);
        
        while(numberOfMap < 10)
        {
            numberOfMap ++;
            Serial.print("Update a document... " + String(numberOfMap));
            
            content.set("fields/" + String(numberOfMap) +"/mapValue/fields/name/stringValue", "wrench" + String(numberOfMap));

            if (Firebase.Firestore.patchDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw(), "" /* updateMask */))
                Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
            else
                Serial.println(fbdo.errorReason());

            delay(500);
        }     

        numberOfMap = 0;   
    }
}

#endif
/**
    @}
*/
/************************ (C) COPYRIGHT GREMSY *****END OF FILE****************/


