#ifndef Cooler_h
#define Cooler_h

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <BlynkSimpleSerialBLE.h>
#include "./TinyGPS.h"                 // Use local version of this library
#include "./CoolerDefinitions.h"


GeoLoc checkGPS();
GeoLoc gpsdump(TinyGPS &gps) ;
bool feedgps() ;
void displayCompassDetails(void) ;
float geoBearing(struct GeoLoc &a, struct GeoLoc &b) ;
float geoDistance(struct GeoLoc &a, struct GeoLoc &b) ;
float geoHeading() ;
void setServo(int pos) ;
void setupCompass() ;
void drive(int,float,int);

#endif
