#pragma once

#include <stdint.h>

#include "iso22133.h"
#include "header.h"
#include "footer.h"

#pragma pack(push, 1)
/*! OSEM ID struct */
typedef struct {
	uint32_t deviceID;
	uint32_t subDeviceID;
	uint32_t systemControlCentreID;
} OSEMIDType;

/*! OSEM origin struct */
typedef struct {
	int64_t latitude;
	int64_t longitude;
	int32_t altitude;
	uint16_t rotation;
	uint8_t coordinateSystem;
} OSEMOriginType;

/*! OSEM date time struct */
typedef struct {
	uint32_t dateISO8601;
	uint16_t gpsWeek;
	uint32_t gpsQmsOfWeek;
	uint8_t leapSeconds;
} OSEMDateTimeType;

/*! OSEM accuracy requirements struct */
typedef struct {
	uint16_t maxWayDeviation;
	uint16_t maxLateralDeviation;
	uint16_t maxYawDeviation;
	uint16_t maxPositionError;
	uint16_t heabTimeout;
	uint8_t testMode;
	uint8_t monrRate;
	uint8_t monr2Rate;
	uint8_t heabRate;
	uint32_t maxMessageLength;
} OSEMAccuracyRequirementsType;

/*! OSEM time server struct */
typedef struct {
	uint32_t ip;
	uint16_t port;
} OSEMTimeServerType;

/*! OSEM ID association struct */
typedef struct {
	// TODO
} OSEMIDAssociationType;

/*! OSEM message */
typedef struct {
	HeaderType header;
	uint16_t desiredTransmitterIDValueID;
	uint16_t desiredTransmitterIDContentLength;
	uint32_t desiredTransmitterID;
	uint16_t latitudeValueID;
	uint16_t latitudeContentLength;
	int64_t latitude;
	uint16_t longitudeValueID;
	uint16_t longitudeContentLength;
	int64_t longitude;
	uint16_t altitudeValueID;
	uint16_t altitudeContentLength;
	int32_t altitude;
	uint16_t dateValueID;
	uint16_t dateContentLength;
	uint32_t date;
	uint16_t GPSWeekValueID;
	uint16_t GPSWeekContentLength;
	uint16_t GPSWeek;
	uint16_t GPSQmsOfWeekValueID;
	uint16_t GPSQmsOfWeekContentLength;
	uint32_t GPSQmsOfWeek;
	uint16_t maxWayDeviationValueID;
	uint16_t maxWayDeviationContentLength;
	uint16_t maxWayDeviation;
	uint16_t maxLateralDeviationValueID;
	uint16_t maxLateralDeviationContentLength;
	uint16_t maxLateralDeviation;
	uint16_t minPosAccuracyValueID;
	uint16_t minPosAccuracyContentLength;
	uint16_t minPosAccuracy;
	FooterType footer;
} OSEMType;						//85 bytes
#pragma pack(pop)


//! OSEM value IDs
#define VALUE_ID_OSEM_TRANSMITTER_ID 0x0010
#define VALUE_ID_OSEM_LATITUDE 0x0020
#define VALUE_ID_OSEM_LONGITUDE 0x0021
#define VALUE_ID_OSEM_ALTITUDE 0x0022
#define VALUE_ID_OSEM_DATE 0x0004
#define VALUE_ID_OSEM_GPS_WEEK 0x0003
#define VALUE_ID_OSEM_GPS_QUARTER_MILLISECOND_OF_WEEK 0x0002
#define VALUE_ID_OSEM_MAX_WAY_DEVIATION 0x0070
#define VALUE_ID_OSEM_MAX_LATERAL_DEVIATION 0x0072
#define VALUE_ID_OSEM_MIN_POSITIONING_ACCURACY 0x0074


void convertOSEMToHostRepresentation(
		const OSEMType * OSEMData,
		ObjectSettingsType * ObjectSettingsData);
