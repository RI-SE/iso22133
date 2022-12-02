#include "osem.h"
#include "defines.h"

#include <errno.h>
#include <stdio.h>
#include <string.h>

/*!
 * \brief encodeOSEMMessage Creates an OSEM message and writes it into a buffer based on supplied values. All values are passed as pointers and
 *  passing them as NULL causes the OSEM message to contain a default value for that field (a value representing "unavailable" or similar).
 * \param controlCenterTime Time of control center
 * \param latitude_deg Latitude in degrees of the test origin
 * \param longitude_deg Longitude in degrees of the test origin
 * \param altitude_m Altitude in meters above sea level of the test origin
 * \param maxWayDeviation_m Maximum allowed deviation from target trajectory point, in meters
 * \param maxLateralDeviation_m Maximum lateral deviation from trajectory allowed, in meters
 * \param minimumPositioningAccuracy_m Minimum positioning accuracy required of the object
 * \param osemDataBuffer Buffer to which OSEM message is to be written
 * \param bufferLength Size of the buffer to which OSEM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to the buffer, or -1 in case of an error
 */
ssize_t encodeOSEMMessage(
		const ObjectSettingsType* objectSettings,
		char *osemDataBuffer,
		const size_t bufferLength,
		const char debug) {

	const char SizeDifference64bitTo48bit = 2;
	OSEMType OSEMData;
	struct tm *printableTime;
	char *p = osemDataBuffer;

	if (objectSettings == NULL) {
		fprintf(stderr, "Invalid object settings input pointer\n");
		return -1;
	}
	bool timeServerUsed = objectSettings->timeServer.ip && objectSettings->timeServer.port;
	bool idAssociationUsed = false;

	// Get local time from real time system clock
	time_t tval = objectSettings->currentTime.tv_sec;
	printableTime = localtime(&tval);

	memset(osemDataBuffer, 0, bufferLength);

	// If buffer too small to hold OSEM data, generate an error
	if (bufferLength < sizeof (OSEMData) - 2 * SizeDifference64bitTo48bit) {
		fprintf(stderr, "Buffer too small to hold necessary OSEM data\n");
		return -1;
	}

	// Build header, and account for the two values which are 48 bit in the message
	uint32_t msgLen = sizeof (HeaderType) + sizeof(OSEMIDType) + sizeof(OSEMOriginType)
		- 2 * SizeDifference64bitTo48bit + sizeof (OSEMDateTimeType)
		+ sizeof(OSEMAccuracyRequirementsType) + 4*2*sizeof(uint16_t) + sizeof (FooterType) 
		+ timeServerUsed ? sizeof (OSEMTimeServerType) + 2*sizeof(uint16_t) : 0
		+ idAssociationUsed ? sizeof(OSEMIDAssociationType) + 2*sizeof(uint16_t) : 0; // TODO handle id association
	OSEMData.header = buildISOHeader(MESSAGE_ID_OSEM, msgLen, debug);

	// Fill the OSEM struct with relevant values
	OSEMData.idStructValueID = VALUE_ID_OSEM_ID_STRUCT;
	OSEMData.idStructContentLength = sizeof (OSEMData.ids);
	OSEMData.ids.deviceID = objectSettings->desiredID.transmitter;
	OSEMData.ids.subDeviceID = objectSettings->desiredID.subTransmitter;
	OSEMData.ids.systemControlCentreID = getTransmitterID();

	OSEMData.originStructValueID = VALUE_ID_OSEM_ORIGIN_STRUCT;
	OSEMData.originStructContentLength = sizeof (OSEMData.origin) - 2 * SizeDifference64bitTo48bit;
	OSEMData.origin.latitude = objectSettings->coordinateSystemOrigin.isLatitudeValid ?
		(int64_t)(objectSettings->coordinateSystemOrigin.latitude_deg * LATITUDE_ONE_DEGREE_VALUE)
			  : LATITUDE_UNAVAILABLE_VALUE;
	OSEMData.origin.longitude = objectSettings->coordinateSystemOrigin.isLongitudeValid ?
		(int64_t)(objectSettings->coordinateSystemOrigin.longitude_deg * LONGITUDE_ONE_DEGREE_VALUE)
			  : LONGITUDE_UNAVAILABLE_VALUE;
	OSEMData.origin.altitude = objectSettings->coordinateSystemOrigin.isAltitudeValid ?
		(int32_t)(objectSettings->coordinateSystemOrigin.altitude_m * ALTITUDE_ONE_METER_VALUE)
			  : ALTITUDE_UNAVAILABLE_VALUE;
	OSEMData.origin.rotation = (uint16_t)(objectSettings->coordinateSystemRotation_rad * 180.0 / M_PI * ROTATION_ONE_DEGREE_VALUE);
	OSEMData.origin.coordinateSystem = (uint8_t)(objectSettings->coordinateSystemType);

	OSEMData.dateTimeStructValueID = VALUE_ID_OSEM_DATE_TIME_STRUCT;
	OSEMData.dateTimeStructContentLength = sizeof (OSEMData.timestamp);
	
	OSEMData.timestamp.dateISO8601 =
		(uint32_t) ((printableTime->tm_year + 1900) * 10000 + (printableTime->tm_mon + 1) * 100 +
						   (printableTime->tm_mday));
	int32_t GPSWeek = getAsGPSWeek(&objectSettings->currentTime);
	OSEMData.timestamp.gpsWeek = GPSWeek < 0 ? GPS_WEEK_UNAVAILABLE_VALUE : (uint16_t) GPSWeek;
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(&objectSettings->currentTime);
	OSEMData.timestamp.gpsQmsOfWeek = GPSQmsOfWeek < 0 ? GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : (uint32_t) GPSQmsOfWeek;
	OSEMData.timestamp.leapSeconds = MS_LEAP_SEC_DIFF_UTC_GPS / 1000;

	OSEMData.accReqStructValueID = VALUE_ID_OSEM_ACC_REQ_STRUCT;
	OSEMData.accReqStructContentLength = sizeof (OSEMData.requirements);
	OSEMData.requirements.maxWayDeviation = (uint16_t)(objectSettings->maxDeviation.position_m
		* MAX_WAY_DEVIATION_ONE_METER_VALUE);
	OSEMData.requirements.maxLateralDeviation = (uint16_t)(objectSettings->maxDeviation.lateral_m
		* MAX_LATERAL_DEVIATION_ONE_METER_VALUE);
	OSEMData.requirements.maxYawDeviation = (uint16_t)(objectSettings->maxDeviation.yaw_rad * 180.0 / M_PI
		* MAX_YAW_DEVIATION_ONE_DEGREE_VALUE);
	OSEMData.requirements.maxPositionError = (uint16_t)(objectSettings->minRequiredPositioningAccuracy_m
		* MIN_POSITIONING_ACCURACY_ONE_METER_VALUE);
	OSEMData.requirements.heabTimeout = (uint16_t)((objectSettings->heabTimeout.tv_sec
		 + objectSettings->heabTimeout.tv_usec / 1000000.0) * COMMUNICATION_TIMEOUT_ONE_SECOND_VALUE);
	OSEMData.requirements.testMode = (uint8_t)(objectSettings->testMode);
	OSEMData.requirements.monrRate = (uint8_t)(objectSettings->rate.monr * MONR_RATE_ONE_HZ_VALUE);
	OSEMData.requirements.monr2Rate = (uint8_t)(objectSettings->rate.monr2 * MONR2_RATE_ONE_HZ_VALUE);
	OSEMData.requirements.heabRate = (uint8_t)(objectSettings->rate.heab * HEAB_RATE_ONE_HZ_VALUE);
	OSEMData.requirements.maxMessageLength = UINT32_MAX; // TODO set from system settings

	if (timeServerUsed) {
		OSEMData.timeServerStructValueID = VALUE_ID_OSEM_TIME_SERVER_STRUCT;
		OSEMData.timeServerStructContentLength = sizeof (OSEMData.timeserver);
		OSEMData.timeserver.ip = objectSettings->timeServer.ip;
		OSEMData.timeserver.port = objectSettings->timeServer.port;
	}
	else {
		OSEMData.timeServerStructValueID = 0;
		OSEMData.timeServerStructContentLength = 0;
		OSEMData.timeserver.ip = 0;
		OSEMData.timeserver.port = 0;
	}

	if (debug) {
		printf
			("OSEM message:\n\tID struct value ID: 0x%x\n\tID struct content length: %u"
			 "\n\tDevice ID: %u \n\tSub device ID: %u \n\tSystem control centre ID: %u"
			 "\n\tOrigin struct value ID: 0x%x\n\tOrigin struct content length: %u"
			 "\n\tLatitude: %ld [100 nanodegrees]\n\tLongitude: %ld [100 nanodegrees]"
			 "\n\tAltitude: %d [cm]\n\tRotation: %u [10 millidegrees]\n\tCoordinate system: %u"
			 "\n\tDate time struct value ID: 0x%x\n\tDate time struct content length: %u"
			 "\n\tDate: %u [YYYYMMDD]\n\tGPS week: %u\n\tGPS second of week: %u [¼ ms]"
			 "\n\tLeap seconds: %u [s]"
			 "\n\tAcc req struct value ID: 0x%x\n\tAcc req struct content length: %u"
			 "\n\tMax way deviation: %u [mm]\n\tMax lateral deviation: %u [mm]"
			 "\n\tMax yaw deviation: %u [10 millidegrees]\n\tMax position error: %u [cm]"
			 "\n\tHEAB timeout: %u [10 ms]\n\tTest mode: %u\n\tMONR rate: %u [1 Hz]"
			 "\n\tMONR2 rate: %u [1 Hz]\n\tHEAB rate: %u [1 Hz]\n\tMax message length: %u [B]",
			 OSEMData.idStructValueID, OSEMData.idStructContentLength, OSEMData.ids.deviceID,
			 OSEMData.ids.subDeviceID, OSEMData.ids.systemControlCentreID, OSEMData.originStructValueID,
			 OSEMData.originStructContentLength, OSEMData.origin.latitude,
			 OSEMData.origin.longitude, OSEMData.origin.altitude, OSEMData.origin.rotation,
			 OSEMData.origin.coordinateSystem, OSEMData.dateTimeStructValueID,
			 OSEMData.dateTimeStructContentLength, OSEMData.timestamp.dateISO8601,
			 OSEMData.timestamp.gpsWeek, OSEMData.timestamp.gpsQmsOfWeek,
			 OSEMData.timestamp.leapSeconds, OSEMData.accReqStructValueID,
			 OSEMData.accReqStructContentLength, OSEMData.requirements.maxWayDeviation,
			 OSEMData.requirements.maxLateralDeviation, OSEMData.requirements.maxYawDeviation,
			 OSEMData.requirements.maxPositionError, OSEMData.requirements.heabTimeout,
			 OSEMData.requirements.testMode, OSEMData.requirements.monrRate,
			 OSEMData.requirements.monr2Rate, OSEMData.requirements.heabRate,
			 OSEMData.requirements.maxMessageLength);
		if (timeServerUsed) {
			printf
				("\n\tTime server struct value ID: 0x%x\n\tTime server struct content length: %u"
				 "\n\tTime server IP: %s\n\tTime server port: %u",
				 OSEMData.timeServerStructValueID, OSEMData.timeServerStructContentLength,
				 OSEMData.timeserver.ip, OSEMData.timeserver.port);
		}
	}

	// Switch endianness to little endian for all fields
	OSEMData.idStructValueID = htole16(OSEMData.idStructValueID);
	OSEMData.idStructContentLength = htole16(OSEMData.idStructContentLength);
	OSEMData.ids.deviceID = htole32(OSEMData.ids.deviceID);
	OSEMData.ids.subDeviceID = htole32(OSEMData.ids.subDeviceID);
	OSEMData.ids.systemControlCentreID = htole32(OSEMData.ids.systemControlCentreID);
	OSEMData.originStructValueID = htole16(OSEMData.originStructValueID);
	OSEMData.originStructContentLength = htole16(OSEMData.originStructContentLength);
	OSEMData.origin.latitude = (int64_t) htole48(OSEMData.origin.latitude);
	OSEMData.origin.longitude = (int64_t) htole48(OSEMData.origin.longitude);
	OSEMData.origin.altitude = htole32(OSEMData.origin.altitude);
	OSEMData.origin.rotation = htole16(OSEMData.origin.rotation);
	OSEMData.dateTimeStructValueID = htole16(OSEMData.dateTimeStructValueID);
	OSEMData.dateTimeStructContentLength = htole16(OSEMData.dateTimeStructContentLength);
	OSEMData.timestamp.dateISO8601 = htole32(OSEMData.timestamp.dateISO8601);
	OSEMData.timestamp.gpsWeek = htole16(OSEMData.timestamp.gpsWeek);
	OSEMData.timestamp.gpsQmsOfWeek = htole32(OSEMData.timestamp.gpsQmsOfWeek);
	OSEMData.accReqStructValueID = htole16(OSEMData.accReqStructValueID);
	OSEMData.accReqStructContentLength = htole16(OSEMData.accReqStructContentLength);
	OSEMData.requirements.maxWayDeviation = htole16(OSEMData.requirements.maxWayDeviation);
	OSEMData.requirements.maxLateralDeviation = htole16(OSEMData.requirements.maxLateralDeviation);
	OSEMData.requirements.maxYawDeviation = htole16(OSEMData.requirements.maxYawDeviation);
	OSEMData.requirements.maxPositionError = htole16(OSEMData.requirements.maxPositionError);
	OSEMData.requirements.heabTimeout = htole16(OSEMData.requirements.heabTimeout);
	OSEMData.requirements.maxMessageLength = htole32(OSEMData.requirements.maxMessageLength);

	// Copy data from OSEM struct into the buffer
	// Must be done before constructing the footer due to the two 48bit size anomalies
	memcpy(p, &OSEMData.header, sizeof (OSEMData.header));
	p += sizeof (OSEMData.header);

	memcpy(p, &OSEMData.idStructValueID, sizeof (OSEMData.idStructValueID)
		+ sizeof (OSEMData.idStructContentLength) + sizeof (OSEMData.ids));
	p += sizeof (OSEMData.idStructValueID) + sizeof (OSEMData.idStructContentLength)
		+ sizeof (OSEMData.ids);

	// Special handling of 48 bit values
	memcpy(p, &OSEMData.originStructValueID, sizeof (OSEMData.originStructValueID)
		+ sizeof (OSEMData.originStructContentLength));
	p += sizeof (OSEMData.originStructValueID) + sizeof (OSEMData.originStructContentLength);
	memcpy(p, &OSEMData.origin.latitude, sizeof (OSEMData.origin.latitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.origin.latitude) - SizeDifference64bitTo48bit;
	memcpy(p, &OSEMData.origin.longitude, sizeof (OSEMData.origin.longitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.origin.longitude) - SizeDifference64bitTo48bit;
	memcpy(p, &OSEMData.origin.altitude, sizeof (OSEMData.origin.altitude)
		+ sizeof (OSEMData.origin.rotation) + sizeof (OSEMData.origin.coordinateSystem));
	p += sizeof (OSEMData.origin.altitude) + sizeof (OSEMData.origin.rotation)
		+ sizeof (OSEMData.origin.coordinateSystem);
	
	// Copy rest of struct (excluding footer) directly into buffer since no more byte anomalies remain
	memcpy(p, &OSEMData.dateTimeStructValueID, sizeof (OSEMData.dateTimeStructValueID)
		+ sizeof (OSEMData.dateTimeStructContentLength) + sizeof (OSEMData.timestamp)
		+ sizeof (OSEMData.accReqStructValueID) + sizeof (OSEMData.accReqStructContentLength)
		+ sizeof (OSEMData.requirements));
	p += sizeof (OSEMData.dateTimeStructValueID) + sizeof (OSEMData.dateTimeStructContentLength)
		+ sizeof (OSEMData.timestamp) + sizeof (OSEMData.accReqStructValueID)
		+ sizeof (OSEMData.accReqStructContentLength) + sizeof (OSEMData.requirements);
	
	if (timeServerUsed) {
		memcpy(p, &OSEMData.timeServerStructValueID, sizeof (OSEMData.timeServerStructValueID)
			+ sizeof (OSEMData.timeServerStructContentLength) + sizeof (OSEMData.timeserver));
		p += sizeof (OSEMData.timeServerStructValueID) + sizeof (OSEMData.timeServerStructContentLength)
			+ sizeof (OSEMData.timeserver);
	}
	// TODO id association list
	// Build footer
	OSEMData.footer =
		buildISOFooter(osemDataBuffer, p - osemDataBuffer + sizeof(OSEMData.footer), debug);
	memcpy(p, &OSEMData.footer, sizeof (OSEMData.footer));
	p += sizeof (OSEMData.footer);

	return p - osemDataBuffer;
}


/*!
 * \brief Decode OSEM messages, fills the OSEMType data struct with information and outputs a custom struct.
 * \param ObjectSettingsData Struct holding origin settings, time etc.
 * \param osemDataBuffer Data buffer with values that are to be decoded
 * \param senderID Variable for holding sender of parsed OSEM message
 * \param bufferLength Length of the osemDatabuffer
 * \param debug Flag for enabling debugging
 * \return Value according to ::ISOMessageReturnValue
 */
ssize_t decodeOSEMMessage(ObjectSettingsType * objectSettingsData,
										const char *osemDataBuffer, const size_t bufferLength,
										uint32_t * senderID, const char debug) {

	OSEMType OSEMData;
	const char SizeDifference64bitTo48bit = 2;
	const char *p = osemDataBuffer;
	const uint16_t ExpectedOSEMStructSize = (uint16_t) (sizeof (OSEMData) - sizeof (OSEMData.header)
														- sizeof (OSEMData.footer.Crc));	//TO do check!!
	uint16_t valueID;
	uint16_t contentLength;

	if (objectSettingsData == NULL || osemDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}



	ssize_t retval = MESSAGE_OK;

	memset(objectSettingsData, 0, sizeof (*objectSettingsData));
	memset(&OSEMData, 0, sizeof (OSEMData));

	if ((retval = decodeISOHeader(p, bufferLength, &OSEMData.header, debug)) != MESSAGE_OK) {
		memset(objectSettingsData, 0, sizeof (*objectSettingsData));
		return retval;
	}
	p += sizeof (OSEMData.header);
	if (senderID != NULL) {
		*senderID = OSEMData.header.transmitterID;
	}

	// If message is not a OSEM message, generate an error
	if (OSEMData.header.messageID != MESSAGE_ID_OSEM) {
		fprintf(stderr, "Attempted to pass non-OSEM message into OSEM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	// Decode contents
	while ((size_t) (p - osemDataBuffer) < OSEMData.header.messageLength + sizeof (OSEMData.header)) {
		// Decode value ID and length
		memset(&valueID, 0, sizeof (valueID));
		memcpy(&valueID, p, sizeof (valueID));
		valueID = le16toh(valueID);
		p += sizeof (valueID);
		memset(&contentLength, 0, sizeof (contentLength));
		memcpy(&contentLength, p, sizeof (contentLength));
		contentLength = le16toh(contentLength);
		p += sizeof (contentLength);
		// Handle contents
		switch (valueID) {
		case VALUE_ID_OSEM_LATITUDE:
			memcpy(&OSEMData.latitudeValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.latitudeContentLength, &contentLength, sizeof (OSEMData.latitudeContentLength));
			// Don't like solution anyone has a better one?
			memcpy(&OSEMData.latitude, p, sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit);
			OSEMData.latitude = le48toh(OSEMData.latitude);
			break;
		case VALUE_ID_OSEM_LONGITUDE:
			memcpy(&OSEMData.longitudeValueID, &contentLength, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.longitudeContentLength, &contentLength,
				   sizeof (OSEMData.longitudeContentLength));
			// Don't like solution anyone has a better one?
			memcpy(&OSEMData.longitude, p, sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit);
			OSEMData.longitude = le48toh(OSEMData.longitude);
			break;
		case VALUE_ID_OSEM_ALTITUDE:
			memcpy(&OSEMData.altitudeValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.altitudeContentLength, &contentLength, sizeof (OSEMData.altitudeContentLength));
			memcpy(&OSEMData.altitude, p, sizeof (OSEMData.altitude));
			OSEMData.altitude = le32toh(OSEMData.altitude);
			break;
		case VALUE_ID_OSEM_DATE:
			memcpy(&OSEMData.dateValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.dateContentLength, &contentLength, sizeof (OSEMData.dateContentLength));
			memcpy(&OSEMData.date, p, sizeof (OSEMData.date));
			OSEMData.date = le32toh(OSEMData.date);
			break;
		case VALUE_ID_OSEM_GPS_WEEK:
			memcpy(&OSEMData.GPSWeekValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.GPSWeekContentLength, &contentLength, sizeof (OSEMData.GPSWeekContentLength));
			memcpy(&OSEMData.GPSWeek, p, sizeof (OSEMData.GPSWeek));
			OSEMData.GPSWeek = le16toh(OSEMData.GPSWeek);
			break;
		case VALUE_ID_OSEM_GPS_QUARTER_MILLISECOND_OF_WEEK:
			memcpy(&OSEMData.GPSQmsOfWeekValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.GPSQmsOfWeekContentLength, &contentLength,
				   sizeof (OSEMData.GPSQmsOfWeekContentLength));
			memcpy(&OSEMData.GPSQmsOfWeek, p, sizeof (OSEMData.GPSQmsOfWeek));
			OSEMData.GPSQmsOfWeek = le32toh(OSEMData.GPSQmsOfWeek);
			break;
		case VALUE_ID_OSEM_MAX_WAY_DEVIATION:
			memcpy(&OSEMData.maxWayDeviationValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.maxWayDeviationContentLength, &contentLength,
				   sizeof (OSEMData.maxWayDeviationContentLength));
			memcpy(&OSEMData.maxWayDeviation, p, sizeof (OSEMData.maxWayDeviation));
			OSEMData.maxWayDeviation = le16toh(OSEMData.maxWayDeviation);
			break;
		case VALUE_ID_OSEM_MAX_LATERAL_DEVIATION:
			memcpy(&OSEMData.maxLateralDeviationValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.maxLateralDeviationContentLength, &contentLength,
				   sizeof (OSEMData.maxLateralDeviationContentLength));
			memcpy(&OSEMData.maxLateralDeviation, p, sizeof (OSEMData.maxLateralDeviation));
			OSEMData.maxLateralDeviation = le16toh(OSEMData.maxLateralDeviation);
			break;
		case VALUE_ID_OSEM_MIN_POSITIONING_ACCURACY:
			memcpy(&OSEMData.minPosAccuracyValueID, &valueID, sizeof (OSEMData.latitudeValueID));
			memcpy(&OSEMData.minPosAccuracyContentLength, &contentLength,
				   sizeof (OSEMData.minPosAccuracyContentLength));
			memcpy(&OSEMData.minPosAccuracy, p, sizeof (OSEMData.minPosAccuracy));
			OSEMData.minPosAccuracy = le16toh(OSEMData.minPosAccuracy);
			break;
		case VALUE_ID_OSEM_TRANSMITTER_ID:
			memcpy(&OSEMData.desiredTransmitterIDValueID, &valueID, sizeof (OSEMData.desiredTransmitterIDValueID));
			memcpy(&OSEMData.desiredTransmitterIDContentLength, &contentLength,
				   sizeof (OSEMData.desiredTransmitterIDContentLength));
			memcpy(&OSEMData.desiredTransmitterID, p, sizeof (OSEMData.desiredTransmitterID));
			OSEMData.desiredTransmitterID = le32toh(OSEMData.desiredTransmitterID);
			break;
		default:
			printf("Unable to handle OSEM value ID 0x%x\n", valueID);
			break;
		}
		p += contentLength;
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - osemDataBuffer), &OSEMData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding OSEM footer\n");
		return retval;
	}
	p += sizeof (OSEMData.footer);

	if ((retval = verifyChecksum(osemDataBuffer, OSEMData.header.messageLength + sizeof (OSEMData.header),
								 OSEMData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "OSEM checksum error\n");
		return retval;
	}

	if (debug) {
		printf("OSEM message:\n");
		printf("\tDesired transmitter ID value ID: 0x%x\n", OSEMData.desiredTransmitterIDValueID);
		printf("\tDesired transmitter ID content length: %u\n", OSEMData.desiredTransmitterIDContentLength);
		printf("\tDesired transmitter ID: %u\n", OSEMData.desiredTransmitterID);
		printf("\tLatitude value ID: 0x%x\n", OSEMData.latitudeValueID);
		printf("\tLatitude content length: %u\n", OSEMData.latitudeContentLength);
		printf("\tLatitude: %ld\n", OSEMData.latitude);
		printf("\tLongitude value ID: 0x%x\n", OSEMData.longitudeValueID);
		printf("\tLongitude content length: %u\n", OSEMData.longitudeContentLength);
		printf("\tLongitude: %ld\n", OSEMData.longitude);
		printf("\tAltitude value ID: 0x%x\n", OSEMData.altitudeValueID);
		printf("\tAltitude content length: %u\n", OSEMData.altitudeContentLength);
		printf("\tAltitude: %d\n", OSEMData.altitude);
		printf("\tDate value ID: 0x%x\n", OSEMData.dateValueID);
		printf("\tDate content length ID: %u\n", OSEMData.dateContentLength);
		printf("\tDate: %u\n", OSEMData.date);
		printf("\tGPS week value ID: 0x%x\n", OSEMData.GPSWeekValueID);
		printf("\tGPS week content length: %u\n", OSEMData.GPSWeekContentLength);
		printf("\tGPS week: %u\n", OSEMData.GPSWeek);
		printf("\tGPS time of week value ID: 0x%x\n", OSEMData.GPSQmsOfWeekValueID);
		printf("\tGPS time of week content length: %u\n", OSEMData.GPSQmsOfWeekContentLength);
		printf("\tGPS time of week: %u [¼ ms]\n", OSEMData.GPSQmsOfWeek);
		printf("\tMax position deviation value ID: 0x%x\n", OSEMData.maxWayDeviationValueID);
		printf("\tMax position deviation content length: %u\n", OSEMData.maxWayDeviationContentLength);
		printf("\tMax position deviation: %u\n", OSEMData.maxWayDeviation);
		printf("\tMax lateral deviation value ID: 0x%x\n", OSEMData.maxLateralDeviationValueID);
		printf("\tMax lateral deviation content length: %u\n", OSEMData.maxLateralDeviationContentLength);
		printf("\tMax lateral deviation: %u\n", OSEMData.maxLateralDeviation);
		printf("\tMin position accuracy value ID: 0x%x\n", OSEMData.minPosAccuracyValueID);
		printf("\tMin position accuracy content length: %u\n", OSEMData.minPosAccuracyContentLength);
		printf("\tMin position accuracy: %u\n", OSEMData.minPosAccuracy);
	}

	// Fill output struct with parsed data
	convertOSEMToHostRepresentation(&OSEMData, objectSettingsData);
	return retval < 0 ? retval : p - osemDataBuffer;
}


/*!
 * \brief Maps values from OSEMType struct to custom data struct ObjectSettingsType where values can later be used
 * \param OSEMData OSEMType struct with decoded OSEM values from server
 * \param ObjectSettingsData Custom struct ObjectSettingsType output values needed later in other functions
 */
void convertOSEMToHostRepresentation(const OSEMType * OSEMData, ObjectSettingsType * ObjectSettingsData) {
	// Check for validity of contents
	ObjectSettingsData->isTransmitterIDValid =
			OSEMData->desiredTransmitterIDValueID
			&& OSEMData->desiredTransmitterID != TRANSMITTER_ID_UNAVAILABLE_VALUE;
	ObjectSettingsData->isTimestampValid =
			OSEMData->GPSWeekValueID
			&& OSEMData->GPSQmsOfWeekValueID
			&& OSEMData->GPSWeek != GPS_WEEK_UNAVAILABLE_VALUE
			&& OSEMData->GPSQmsOfWeek != GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	ObjectSettingsData->isLateralDeviationLimited =
			OSEMData->maxLateralDeviationValueID
			&& OSEMData->maxLateralDeviation != MAX_LATERAL_DEVIATION_UNAVAILABLE_VALUE;
	ObjectSettingsData->isPositionDeviationLimited =
			OSEMData->maxWayDeviationValueID
			&& OSEMData->maxWayDeviation != MAX_WAY_DEVIATION_UNAVAILABLE_VALUE;
	ObjectSettingsData->isPositioningAccuracyRequired =
			OSEMData->minPosAccuracyValueID
			&& OSEMData->minPosAccuracy != MIN_POSITIONING_ACCURACY_NOT_REQUIRED_VALUE;
	ObjectSettingsData->coordinateSystemOrigin.isLatitudeValid =
			OSEMData->latitudeValueID
			&& OSEMData->latitude != LATITUDE_UNAVAILABLE_VALUE;
	ObjectSettingsData->coordinateSystemOrigin.isLongitudeValid =
			OSEMData->longitudeValueID
			&& OSEMData->longitude != LONGITUDE_UNAVAILABLE_VALUE;
	ObjectSettingsData->coordinateSystemOrigin.isAltitudeValid =
			OSEMData->altitudeValueID
			&& OSEMData->altitude != ALTITUDE_UNAVAILABLE_VALUE;

	// Convert timestamp
	if (ObjectSettingsData->isTimestampValid) {
		ObjectSettingsData->isTimestampValid = setToGPStime(&ObjectSettingsData->currentTime,
															OSEMData->GPSWeek, OSEMData->GPSQmsOfWeek) >= 0;
	}

	// Transmitter ID
	ObjectSettingsData->desiredTransmitterID =
			ObjectSettingsData->isTransmitterIDValid ?
				OSEMData->desiredTransmitterID : 0;

	// Convert origin
	ObjectSettingsData->coordinateSystemOrigin.latitude_deg =
		ObjectSettingsData->coordinateSystemOrigin.isLatitudeValid ? OSEMData->latitude /
		LATITUDE_ONE_DEGREE_VALUE : 0;
	ObjectSettingsData->coordinateSystemOrigin.longitude_deg =
		ObjectSettingsData->coordinateSystemOrigin.isLongitudeValid ? OSEMData->longitude /
		LONGITUDE_ONE_DEGREE_VALUE : 0;
	ObjectSettingsData->coordinateSystemOrigin.altitude_m =
		ObjectSettingsData->coordinateSystemOrigin.isAltitudeValid ? OSEMData->altitude /
		ALTITUDE_ONE_METER_VALUE : 0;

	// Accuracies / deviations
	ObjectSettingsData->minRequiredPositioningAccuracy_m = ObjectSettingsData->isPositioningAccuracyRequired ?
		OSEMData->minPosAccuracy / MIN_POSITIONING_ACCURACY_ONE_METER_VALUE : 0;
	ObjectSettingsData->maxPositionDeviation_m = ObjectSettingsData->isPositionDeviationLimited ?
		OSEMData->maxWayDeviation / MAX_WAY_DEVIATION_ONE_METER_VALUE : 0;
	ObjectSettingsData->maxLateralDeviation_m = ObjectSettingsData->isLateralDeviationLimited ?
		OSEMData->maxLateralDeviation / MAX_LATERAL_DEVIATION_ONE_METER_VALUE : 0;

}
