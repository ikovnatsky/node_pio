#include "../lib/ArduinoJson/ArduinoJson.h"
#include "SPIFFS.h"
#include <list>
#include "SMNode.h"
extern SMNode theNode;

void saveConfig(const char * filename)
{
  File file = SPIFFS.open(filename);

    // Allocate the document on the stack.
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<868> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    {
        Serial.println(F("Failed to read file, using default configuration"));
      //  return;
    }

  // Get the root object in the document
  JsonObject rt = doc.as<JsonObject>();
  file.close();
  file = SPIFFS.open(filename,"w");
  rt["assetName1"]=theNode.line1;
  rt["assetName2"]=theNode.line2;
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
    }

  // Close the file (File's destructor doesn't close the file)
  file.close();
}

void loadConfiguration(const char *filename)
{
  // Open file for reading
  File file = SPIFFS.open(filename);

    // Allocate the document on the stack.
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<868> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    {
        Serial.println(F("Failed to read file, using default configuration"));
      //  return;
    }

  // Get the root object in the document
  JsonObject rt = doc.as<JsonObject>();

  // load the beacons
  JsonArray bs = rt["lorahf_beacons"];
  for (auto beacons : bs) 
        {
        HFBeacon tmp;
        tmp.lon= beacons["lon"];
        tmp.lat=beacons["lat"];
        tmp.id=beacons["id"];
        tmp.z=beacons["z"];
        printf("hfs %f %f %d\n",tmp.lon,tmp.lat,tmp.id);
        theNode.HFBeaconList.push_back(tmp);
        //const char* value = request["value"];
        }

  theNode.hfDelayCal=rt["hfDelayCal"];
  theNode.rangingAddress=rt["rangingAddress"];
  theNode.configMinHdop = rt["configMinHdop"];
  printf("delay %d \n",theNode.hfDelayCal);
  
  // load the bands
  JsonArray bands = rt["bands"];
  for (auto bi : bands)
    theNode.bands.push_back(bi);
  if (theNode.bands.size()==0)
    theNode.bands.push_back(915000000);
     
  if (rt.containsKey("role"))
    if (strcmp(rt["role"],"BEACON")==0)
        theNode.role=ROLE_BEACON;
    else
        theNode.role=ROLE_NORMAL;

  if (rt.containsKey("assetName1"))
    strcpy(theNode.line1,rt["assetName1"]);
  else
    strcpy(theNode.line1,"Line1");

  if (rt.containsKey("assetName2"))
    strcpy(theNode.line2,rt["assetName2"]);
   else

   
    strcpy(theNode.line1,"Line2");
  theNode.zoneOffset=rt["zoneOffset"];

  if (rt.containsKey("configSSID"))
    {
    strcpy(theNode.configSSID,rt["configSSID"]);
    printf ("[E] no configssd\n");
    }
   else
    strcpy(theNode.configSSID,"PTG");

  if (rt.containsKey("configPWD"))
    strcpy(theNode.configPWD,rt["configPWD"]);
   else
    strcpy(theNode.configPWD,"55aa55aa55");
  
  if (rt.containsKey("rangeTrys"))
     theNode.rangeTrys = rt["rangeTrys"];
  else
     theNode.rangeTrys =2;

  if (rt.containsKey("rangeReps"))
     theNode.rangeReps = rt["rangeReps"];
  else
     theNode.rangeReps =5;

  if (rt.containsKey("rangeNumBeacons"))
     theNode.rangeNumBeacons = rt["rangeNumBeacons"];
  else
     theNode.rangeNumBeacons =2;

  if (rt.containsKey("configRefresh"))
     theNode.configRefresh = rt["configRefresh"];
  else
     theNode.configRefresh =10;

 if (rt.containsKey("ledPWM"))
     theNode.ledPWM = rt["ledPWM"];
  else
     theNode.ledPWM =50;

 if (rt.containsKey("pressureTime"))
    theNode.pressTimer.SetPeriod(rt["pressureTime"]);

 if (rt.containsKey("socTime"))
    theNode.pressTimer.SetPeriod(rt["socTime"]);

 if (rt.containsKey("fwServer"))
    theNode.fwServer=((const char *)rt["fwServer"]);
else
    theNode.fwServer="s3.us-east-2.amazonaws.com";

 if (rt.containsKey("fwImage"))
    theNode.fwImage=((const char *)rt["fwImage"]);
else
    theNode.fwImage="/smc-tag-firmware-updater/tag_v2.bin";


  file.close();
}



  