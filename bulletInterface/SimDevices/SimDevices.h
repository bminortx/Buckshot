// Copyright (c) bminortx

#ifndef URDFPARSER_SIMDEVICES_SIMDEVICES_H_
#define URDFPARSER_SIMDEVICES_SIMDEVICES_H_

// Superclass for devices
#include <SimDevices/SimDeviceInfo.h>
#include <SimDevices/SimCamera.h>
#include <SimDevices/SimGPS.h>
#include <SimDevices/SimVicon.h>

#include <map>
#include <string>
#include <vector>

class SimDevices {
 public:
  //  Constructor
  SimDevices();

  //  Initializers
  void AddDevice(SimDeviceInfo* devInfo);

  // Update devices
  void UpdateSensors();

  //  GETTERS
  std::vector<SimDeviceInfo*> GetAllRelatedDevices(std::string sDeviceBodyName);
  std::vector<SimDeviceInfo*> GetOnDevices();
  std::vector<SimDeviceInfo*> GetOnRelatedDevices(std::string sDeviceBodyName);

  //  MEMBER VARIABLES
  std::map<std::string, SimDeviceInfo*>  sim_device_map_;
};

#endif  // URDFPARSER_SIMDEVICES_SIMDEVICES_H_
