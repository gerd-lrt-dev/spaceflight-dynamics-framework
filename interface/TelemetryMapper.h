#ifndef TELEMETRYMAPPER_H
#define TELEMETRYMAPPER_H

#include "TelemetryDTO.h"
#include "include/simDataStruct.h"
#include <Eigen/Dense>


class telemetryMapper{
public:
    void setBackendData(simData simdata_);

    Telemetry getQTTelemetryData();

private:
    simData backendData;
};

#endif // TELEMETRYMAPPER_H
