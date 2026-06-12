#include "GPS.h"
#include "pinouts.h"
#include <math.h>
#include "libraries.h"

// Converts degrees to radians
double degToRad(double degs) {
    return degs * (M_PI / 180.0);
}

/*
 * Using the onboard GPS & latest rocket GPS coordinates,
 * returns a distance (meters) to the rocket. Expensive
 * calculation, so use sparingly.
 * 
 * Returns -1 if could not eval distance.
 */
int32_t getDistanceToRocket() {
  // Ensure we have enough data
  if (gps.location.lat() == 0 || gps.location.lng() == 0 || gps.location.age() > 5000) {
    return -1;
  }
  if (rocketGPSLat == 0 or rocketGPSLon == 0) {
    return -1;
  }

  // Convert latitude and longitude from degrees to radians
  double lat1Rad = degToRad(gps.location.lat());
  double lon1Rad = degToRad(gps.location.lng());
  double lat2Rad = degToRad(rocketGPSLat/1000000.0);
  double lon2Rad = degToRad(rocketGPSLon/1000000.0);
  
  // Calculate the differences between the points
  double dLat = lat2Rad - lat1Rad;
  double dLon = lon2Rad - lon1Rad;
  
  // Apply the Haversine formula
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1Rad) * cos(lat2Rad) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  // Calculate the distance in feet
  double distance = EARTH_RADIUS_FEET * c;
  
  return distance/3.281;
}

// Configure GPS UART1 Bus
void gpsInit() {
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
}

/*
 * Displays detailed GPS information to Serial
 */
void gpsDisplayInfoGrand() {
  if (gps.location.isUpdated())
  {
    Serial.print("LOCATION   Fix Age=");
    Serial.print(gps.location.age());
    Serial.print("ms Raw Lat=");
    Serial.print(gps.location.rawLat().negative ? "-" : "+");
    Serial.print(gps.location.rawLat().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLat().billionths);
    Serial.print(" billionths],  Raw Long=");
    Serial.print(gps.location.rawLng().negative ? "-" : "+");
    Serial.print(gps.location.rawLng().deg);
    Serial.print("[+");
    Serial.print(gps.location.rawLng().billionths);
    Serial.print(" billionths],  Lat=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Long=");
    Serial.println(gps.location.lng(), 6);
  }

  else if (gps.date.isUpdated())
  {
    Serial.print("DATE       Fix Age=");
    Serial.print(gps.date.age());
    Serial.print("ms Raw=");
    Serial.print(gps.date.value());
    Serial.print(" Year=");
    Serial.print(gps.date.year());
    Serial.print(" Month=");
    Serial.print(gps.date.month());
    Serial.print(" Day=");
    Serial.println(gps.date.day());
  }

  else if (gps.time.isUpdated())
  {
    Serial.print("TIME       Fix Age=");
    Serial.print(gps.time.age());
    Serial.print("ms Raw=");
    Serial.print(gps.time.value());
    Serial.print(" Hour=");
    Serial.print(gps.time.hour());
    Serial.print(" Minute=");
    Serial.print(gps.time.minute());
    Serial.print(" Second=");
    Serial.print(gps.time.second());
    Serial.print(" Hundredths=");
    Serial.println(gps.time.centisecond());
  }

  else if (gps.speed.isUpdated())
  {
    Serial.print("SPEED      Fix Age=");
    Serial.print(gps.speed.age());
    Serial.print("ms Raw=");
    Serial.print(gps.speed.value());
    Serial.print(" Knots=");
    Serial.print(gps.speed.knots());
    Serial.print(" MPH=");
    Serial.print(gps.speed.mph());
    Serial.print(" m/s=");
    Serial.print(gps.speed.mps());
    Serial.print(" km/h=");
    Serial.println(gps.speed.kmph());
  }

  else if (gps.course.isUpdated())
  {
    Serial.print("COURSE     Fix Age=");
    Serial.print(gps.course.age());
    Serial.print("ms Raw=");
    Serial.print(gps.course.value());
    Serial.print(" Deg=");
    Serial.println(gps.course.deg());
  }

  else if (gps.altitude.isUpdated())
  {
    Serial.print("ALTITUDE   Fix Age=");
    Serial.print(gps.altitude.age());
    Serial.print("ms Raw=");
    Serial.print(gps.altitude.value());
    Serial.print(" Meters=");
    Serial.print(gps.altitude.meters());
    Serial.print(" Miles=");
    Serial.print(gps.altitude.miles());
    Serial.print(" KM=");
    Serial.print(gps.altitude.kilometers());
    Serial.print(" Feet=");
    Serial.println(gps.altitude.feet());
  }

  else if (gps.satellites.isUpdated())
  {
    Serial.print("SATELLITES Fix Age=");
    Serial.print(gps.satellites.age());
    Serial.print("ms Value=");
    Serial.println(gps.satellites.value());
  }

  else if (gps.hdop.isUpdated())
  {
    Serial.print("HDOP       Fix Age=");
    Serial.print(gps.hdop.age());
    Serial.print("ms raw=");
    Serial.print(gps.hdop.value());
    Serial.print(" hdop=");
    Serial.println(gps.hdop.hdop());
  }
}

/*
 * Handles receiving and processing GPS information.
 * To be run in loop()
 * 
 * TODO: Log local GPS data to SD too
 */
void handleGPS() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      //gpsDisplayInfoGrand();
    }
  }

  if (millis() - lastLocalGPSLog > LOCAL_GPS_LOG_RATE) {
    lastLocalGPSLog = millis();
    String resp = String("LocalGPS: ");
    resp += String(gps.satellites.age());
    resp += "," + String(gps.satellites.value());
    resp += "," + String(gps.location.lat(), 6);
    resp += "," + String(gps.location.lng(), 6);
    writeToSDLog(resp);
  }

  if (millis() > 50000 && gps.charsProcessed() < 10)
  {
    Serial.println("[ERROR] No response from onboard GPS");
  }
}


void calculateRocketVelocity() {
    static int32_t previousAltitude = 0;
    static uint32_t lastTime = 0;
    
    if (rfmLastPacketValid && millis() - rfmLastRFReceived < RFM_CONNECTED_TIMEOUT) {
        uint32_t currentTime = millis();
        int32_t currentAltitude = rocketAltitude;
        
        if (lastTime > 0 && previousAltitude > 0) {
            // Calculate velocity (ft/s)
			// TODO: ENSURE BOTH ALTIMETER MODULE & KUHGLOCKE ARE CONSISTENTLY USING ALL METERS.
			// NOTE: Using meters in code & then adding a quick frontend calculation to change it to feet would be the best setup.
            float timeDiff = (currentTime - lastTime) / 1000.0; // Convert to seconds
            float altDiff = currentAltitude - previousAltitude;
            rocketVelocity = altDiff / timeDiff;
        }
        
        previousAltitude = currentAltitude;
        lastTime = currentTime;
    }
}//calculateRocketVelocity()




