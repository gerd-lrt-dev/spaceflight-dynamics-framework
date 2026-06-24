#ifndef TELEMETRYMAPPER_H
#define TELEMETRYMAPPER_H

#include "TelemetryDTO.h"
#include "include/simDataStruct.h"


class telemetryMapper{
public:
    void setBackendData(simData simdata_);

    Telemetry getQTTelemetryData();

private:
    simData backendData;
};

#endif // TELEMETRYMAPPER_H
