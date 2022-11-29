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

	// Get local time from real time system clock
	if (objectSettings->isTimestampValid) {
		time_t tval = objectSettings->currentTime.tv_sec;
		printableTime = localtime(&tval);
	}

	memset(osemDataBuffer, 0, bufferLength);

	// If buffer too small to hold OSEM data, generate an error
	if (bufferLength < sizeof (OSEMData) - 2 * SizeDifference64bitTo48bit) {
		fprintf(stderr, "Buffer too small to hold necessary OSEM data\n");
		return -1;
	}

	// Build header, and account for the two values which are 48 bit in the message
	OSEMData.header = buildISOHeader(MESSAGE_ID_OSEM, sizeof (OSEMData)
									 - 2 * SizeDifference64bitTo48bit, debug);

	// Fill the OSEM struct with relevant values
	OSEMData.desiredTransmitterIDValueID = VALUE_ID_OSEM_TRANSMITTER_ID;
	OSEMData.desiredTransmitterIDContentLength = sizeof (OSEMData.desiredTransmitterID);
	OSEMData.desiredTransmitterID = objectSettings->isTransmitterIDValid ?
				objectSettings->desiredTransmitterID : TRANSMITTER_ID_UNAVAILABLE_VALUE;

	OSEMData.latitudeValueID = VALUE_ID_OSEM_LATITUDE;
	OSEMData.latitudeContentLength = sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit;
	OSEMData.latitude = objectSettings->coordinateSystemOrigin.isLatitudeValid ?
				(int64_t)(objectSettings->coordinateSystemOrigin.latitude_deg * LATITUDE_ONE_DEGREE_VALUE)
			  : LATITUDE_UNAVAILABLE_VALUE;

	OSEMData.longitudeValueID = VALUE_ID_OSEM_LONGITUDE;
	OSEMData.longitudeContentLength = sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit;
	OSEMData.longitude = objectSettings->coordinateSystemOrigin.isLongitudeValid ?
				(int64_t)(objectSettings->coordinateSystemOrigin.longitude_deg * LONGITUDE_ONE_DEGREE_VALUE)
			  : LONGITUDE_UNAVAILABLE_VALUE;

	OSEMData.altitudeValueID = VALUE_ID_OSEM_ALTITUDE;
	OSEMData.altitudeContentLength = sizeof (OSEMData.altitude);
	OSEMData.altitude = objectSettings->coordinateSystemOrigin.isAltitudeValid ?
				(int32_t)(objectSettings->coordinateSystemOrigin.altitude_m * ALTITUDE_ONE_METER_VALUE)
			  : ALTITUDE_UNAVAILABLE_VALUE;

	OSEMData.dateValueID = VALUE_ID_OSEM_DATE;
	OSEMData.dateContentLength = sizeof (OSEMData.date);
	OSEMData.date = objectSettings->isTimestampValid ?
				(uint32_t) ((printableTime->tm_year + 1900) * 10000 + (printableTime->tm_mon + 1) * 100 +
						   (printableTime->tm_mday))
			  : DATE_UNAVAILABLE_VALUE; //Should OSEM be able to be sent without Date?

	OSEMData.GPSWeekValueID = VALUE_ID_OSEM_GPS_WEEK;
	OSEMData.GPSWeekContentLength = sizeof (OSEMData.GPSWeek);
	int32_t GPSWeek = getAsGPSWeek(&objectSettings->currentTime);

	OSEMData.GPSWeek = GPSWeek < 0 ? GPS_WEEK_UNAVAILABLE_VALUE : (uint16_t) GPSWeek;

	OSEMData.GPSQmsOfWeekValueID = VALUE_ID_OSEM_GPS_QUARTER_MILLISECOND_OF_WEEK;
	OSEMData.GPSQmsOfWeekContentLength = sizeof (OSEMData.GPSQmsOfWeek);
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(&objectSettings->currentTime);

	OSEMData.GPSQmsOfWeek = GPSQmsOfWeek < 0 ? GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : (uint32_t) GPSQmsOfWeek;

	OSEMData.maxWayDeviationValueID = VALUE_ID_OSEM_MAX_WAY_DEVIATION;
	OSEMData.maxWayDeviationContentLength = sizeof (OSEMData.maxWayDeviation);
	OSEMData.maxWayDeviation = objectSettings->isPositionDeviationLimited ?
				(uint16_t)(objectSettings->maxPositionDeviation_m * MAX_WAY_DEVIATION_ONE_METER_VALUE)
			  : MAX_WAY_DEVIATION_UNAVAILABLE_VALUE;

	OSEMData.maxLateralDeviationValueID = VALUE_ID_OSEM_MAX_LATERAL_DEVIATION;
	OSEMData.maxLateralDeviationContentLength = sizeof (OSEMData.maxLateralDeviation);
	OSEMData.maxLateralDeviation = objectSettings->isLateralDeviationLimited ?
				(uint16_t)(objectSettings->maxLateralDeviation_m * MAX_LATERAL_DEVIATION_ONE_METER_VALUE)
			  : MAX_LATERAL_DEVIATION_UNAVAILABLE_VALUE;

	OSEMData.minPosAccuracyValueID = VALUE_ID_OSEM_MIN_POSITIONING_ACCURACY;
	OSEMData.minPosAccuracyContentLength = sizeof (OSEMData.minPosAccuracy);
	OSEMData.minPosAccuracy = objectSettings->isPositioningAccuracyRequired ?
				(uint16_t)(objectSettings->minRequiredPositioningAccuracy_m * MIN_POSITIONING_ACCURACY_ONE_METER_VALUE)
			  : MIN_POSITIONING_ACCURACY_NOT_REQUIRED_VALUE;

	if (debug) {
		printf
			("OSEM message:\n\tLatitude value ID: 0x%x\n\tLatitude content length: %u\n\tLatitude: %ld [100 nanodegrees]\n\t"
			 "Longitude value ID: 0x%x\n\tLongitude content length: %u\n\tLongitude: %ld [100 nanodegrees]\n\t"
			 "Altitude value ID: 0x%x\n\tAltitude content length: %u\n\tAltitude: %d [cm]\n\t"
			 "Date value ID: 0x%x\n\tDate content length: %u\n\tDate: %u\n\t"
			 "GPS week value ID: 0x%x\n\tGPS week content length: %u\n\tGPS week: %u\n\t"
			 "GPS second of week value ID: 0x%x\n\tGPS second of week content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
			 "Max way deviation value ID: 0x%x\n\tMax way deviation content length: %u\n\tMax way deviation: %u\n\t"
			 "Max lateral deviation value ID: 0x%x\n\tMax lateral deviation content length: %u\n\t"
			 "Min positioning accuracy value ID: 0x%x\n\tMin positioning accuracy content length: %u\n\tMin positioning accuracy: %u\n",
			 OSEMData.latitudeValueID, OSEMData.latitudeContentLength, OSEMData.latitude,
			 OSEMData.longitudeValueID, OSEMData.longitudeContentLength, OSEMData.longitude,
			 OSEMData.altitudeValueID, OSEMData.altitudeContentLength, OSEMData.altitude,
			 OSEMData.dateValueID, OSEMData.dateContentLength, OSEMData.date, OSEMData.GPSWeekValueID,
			 OSEMData.GPSWeekContentLength, OSEMData.GPSWeek, OSEMData.GPSQmsOfWeekValueID,
			 OSEMData.GPSQmsOfWeekContentLength, OSEMData.GPSQmsOfWeek, OSEMData.maxWayDeviationValueID,
			 OSEMData.maxWayDeviationContentLength, OSEMData.maxWayDeviation,
			 OSEMData.maxLateralDeviationValueID, OSEMData.maxLateralDeviationContentLength,
			 OSEMData.maxLateralDeviation, OSEMData.minPosAccuracyValueID,
			 OSEMData.minPosAccuracyContentLength);
	}

	// Switch endianness to little endian for all fields
	OSEMData.desiredTransmitterIDValueID = htole16(OSEMData.desiredTransmitterIDValueID);
	OSEMData.desiredTransmitterIDContentLength = htole16(OSEMData.desiredTransmitterIDContentLength);
	OSEMData.desiredTransmitterID = htole32(OSEMData.desiredTransmitterID);
	OSEMData.latitudeValueID = htole16(OSEMData.latitudeValueID);
	OSEMData.latitudeContentLength = htole16(OSEMData.latitudeContentLength);
	OSEMData.latitude = (int64_t) htole48(OSEMData.latitude);
	OSEMData.longitudeValueID = htole16(OSEMData.longitudeValueID);
	OSEMData.longitudeContentLength = htole16(OSEMData.longitudeContentLength);
	OSEMData.longitude = (int64_t) htole48(OSEMData.longitude);
	OSEMData.altitudeValueID = htole16(OSEMData.altitudeValueID);
	OSEMData.altitudeContentLength = htole16(OSEMData.altitudeContentLength);
	OSEMData.altitude = (int32_t) htole32(OSEMData.altitude);
	OSEMData.dateValueID = htole16(OSEMData.dateValueID);
	OSEMData.dateContentLength = htole16(OSEMData.dateContentLength);
	OSEMData.date = htole32(OSEMData.date);
	OSEMData.GPSWeekValueID = htole16(OSEMData.GPSWeekValueID);
	OSEMData.GPSWeekContentLength = htole16(OSEMData.GPSWeekContentLength);
	OSEMData.GPSWeek = htole16(OSEMData.GPSWeek);
	OSEMData.GPSQmsOfWeekValueID = htole16(OSEMData.GPSQmsOfWeekValueID);
	OSEMData.GPSQmsOfWeekContentLength = htole16(OSEMData.GPSQmsOfWeekContentLength);
	OSEMData.GPSQmsOfWeek = htole32(OSEMData.GPSQmsOfWeek);
	OSEMData.maxWayDeviationValueID = htole16(OSEMData.maxWayDeviationValueID);
	OSEMData.maxWayDeviationContentLength = htole16(OSEMData.maxWayDeviationContentLength);
	OSEMData.maxWayDeviation = htole16(OSEMData.maxWayDeviation);
	OSEMData.maxLateralDeviationValueID = htole16(OSEMData.maxLateralDeviationValueID);
	OSEMData.maxLateralDeviationContentLength = htole16(OSEMData.maxLateralDeviationContentLength);
	OSEMData.maxLateralDeviation = htole16(OSEMData.maxLateralDeviation);
	OSEMData.minPosAccuracyValueID = htole16(OSEMData.minPosAccuracyValueID);
	OSEMData.minPosAccuracyContentLength = htole16(OSEMData.minPosAccuracyContentLength);
	OSEMData.minPosAccuracy = htole16(OSEMData.minPosAccuracy);


	// Copy data from OSEM struct into the buffer
	// Must be done before constructing the footer due to the two 48bit size anomalies
	memcpy(p, &OSEMData.header, sizeof (OSEMData.header));
	p += sizeof (OSEMData.header);

	memcpy(p, &OSEMData.desiredTransmitterIDValueID,
		   sizeof (OSEMData.desiredTransmitterIDValueID) + sizeof (OSEMData.desiredTransmitterIDContentLength)
		   + sizeof (OSEMData.desiredTransmitterID));
	p += sizeof (OSEMData.desiredTransmitterIDValueID) + sizeof (OSEMData.desiredTransmitterIDContentLength)
			+ sizeof (OSEMData.desiredTransmitterID);

	// Special handling of 48 bit value
	memcpy(p, &OSEMData.latitudeValueID,
		   sizeof (OSEMData.latitudeValueID) + sizeof (OSEMData.latitudeContentLength));
	p += sizeof (OSEMData.latitudeValueID) + sizeof (OSEMData.latitudeContentLength);
	memcpy(p, &OSEMData.latitude, sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.latitude) - SizeDifference64bitTo48bit;

	// Special handling of 48 bit value
	memcpy(p, &OSEMData.longitudeValueID,
		   sizeof (OSEMData.longitudeValueID) + sizeof (OSEMData.longitudeContentLength));
	p += sizeof (OSEMData.longitudeValueID) + sizeof (OSEMData.longitudeContentLength);
	memcpy(p, &OSEMData.longitude, sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit);
	p += sizeof (OSEMData.longitude) - SizeDifference64bitTo48bit;

	// Copy rest of struct (excluding footer) directly into buffer since no more byte anomalies remain
	memcpy(p, &OSEMData.altitudeValueID, sizeof (OSEMData) - sizeof (OSEMData.footer)
		   - (size_t) (p - osemDataBuffer + 2 * SizeDifference64bitTo48bit));
	p += sizeof (OSEMData) - sizeof (OSEMData.footer) - (size_t) (p - osemDataBuffer +
																  2 * SizeDifference64bitTo48bit);

	// Build footer
	OSEMData.footer =
		buildISOFooter(osemDataBuffer, sizeof (OSEMType) - 2 * SizeDifference64bitTo48bit, debug);
	memcpy(p, &OSEMData.footer, sizeof (OSEMData.footer));

	return sizeof (OSEMType) - 2 * SizeDifference64bitTo48bit;
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