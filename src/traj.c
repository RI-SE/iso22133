#include "traj.h"
#include "iohelpers.h"
#include <errno.h>
#include <string.h>

static uint16_t trajectoryMessageCrc = 0;

//! TRAJ header field descriptions
static DebugStrings_t TRAJIdentifierDescription = 	{"Trajectory ID",	"",	&printU32};
static DebugStrings_t TRAJNameDescription = 		{"Trajectory name",	"",	&printString};
static DebugStrings_t TRAJInfoDescription = 		{"Trajectory info",	"",	&printU8};

/*!
 * \brief encodeTRAJMessageHeader Creates a TRAJ message header based on supplied values and resets
 *	an internal CRC to be used in the corresponding footer. The header is printed to a buffer.
 * \param trajectoryID ID of the trajectory
 * \param trajectoryVersion Version of the trajectory
 * \param trajectoryName A string of maximum length 63 excluding the null terminator
 * \param nameLength Length of the name string excluding the null terminator
 * \param numberOfPointsInTraj Number of points in the subsequent trajectory
 * \param trajDataBuffer Buffer to which TRAJ header is to be printed
 * \param bufferLength Length of buffer to which TRAJ header is to be printed
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold header
 *		EMSGSIZE	if trajectory name is too long
 */
ssize_t encodeTRAJMessageHeader(
	const uint16_t trajectoryID,
	const TrajectoryInfoType trajectoryInfo,
	const char* trajectoryName,
	const size_t nameLength,
	const uint32_t numberOfPointsInTraj,
	char *trajDataBuffer,
	const size_t bufferLength,
	const char debug)
{

	TRAJHeaderType TRAJData;
	char* p = trajDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	memset(trajDataBuffer, 0, bufferLength);

	// Error guarding
	if (trajectoryName == NULL && nameLength > 0) {
		errno = EINVAL;
		printf("Trajectory name length and pointer mismatch\n");
		return -1;
	}
	else if (trajDataBuffer == NULL) {
		errno = EINVAL;
		printf("Trajectory data buffer invalid\n");
		return -1;
	}
	else if (bufferLength < sizeof (TRAJHeaderType)) {
		errno = ENOBUFS;
		printf("Buffer too small to hold necessary TRAJ header data\n");
		return -1;
	}
	else if (nameLength >= sizeof (TRAJData.trajectoryName)) {
		errno = EMSGSIZE;
		printf("Trajectory name <%s> too long for TRAJ message\n", trajectoryName);
		return -1;
	}

	// Construct ISO header
	TRAJData.header = buildISOHeader(
		MESSAGE_ID_TRAJ,
		sizeof (TRAJHeaderType)
			+ numberOfPointsInTraj * sizeof (TRAJPointType)
			+ sizeof (TRAJFooterType),
		debug);
	p += sizeof (HeaderType);

	if (debug) {
			printf("TRAJ message header:\n");
	}

	// Fill contents
	retval |= encodeContent(VALUE_ID_TRAJ_TRAJECTORY_IDENTIFIER, &trajectoryID, &p,
		sizeof (TRAJData.trajectoryID), &remainingBytes, &TRAJIdentifierDescription, debug);
	TRAJData.trajectoryInfo = trajectoryInfo;
	retval |= encodeContent(VALUE_ID_TRAJ_TRAJECTORY_INFO, &TRAJData.trajectoryInfo, &p,
		sizeof (TRAJData.trajectoryInfo), &remainingBytes, &TRAJInfoDescription, debug);
	
	// Encode string separately since encodeContent does not handle strings
	TRAJData.trajectoryNameValueID = VALUE_ID_TRAJ_TRAJECTORY_NAME;
	TRAJData.trajectoryNameContentLength = sizeof (TRAJData.trajectoryName);
	memset(TRAJData.trajectoryName, 0, sizeof (TRAJData.trajectoryName));
	if (trajectoryName != NULL) {
		memcpy(&TRAJData.trajectoryName, trajectoryName, nameLength);
		// Ensure null termination
		TRAJData.trajectoryName[TRAJ_NAME_STRING_MAX_LENGTH] = '\0';
	}
	memcpy(p, &TRAJData.trajectoryNameValueID, sizeof (TRAJData.trajectoryNameValueID));
	p += sizeof (TRAJData.trajectoryNameValueID);
	memcpy(p, &TRAJData.trajectoryNameContentLength, sizeof (TRAJData.trajectoryNameContentLength));
	p += sizeof (TRAJData.trajectoryNameContentLength);
	memcpy(p, TRAJData.trajectoryName, sizeof (TRAJData.trajectoryName));
	p += sizeof (TRAJData.trajectoryName);
	
	if (debug) {
		printContent(TRAJData.trajectoryNameValueID,
			TRAJData.trajectoryNameContentLength,
			TRAJData.trajectoryName,
			&TRAJNameDescription);
	}
	// Switch endianness to little endian for name
	TRAJData.trajectoryNameValueID = htole16(TRAJData.trajectoryNameValueID);
	TRAJData.trajectoryNameContentLength = htole16(TRAJData.trajectoryNameContentLength);

	// Reset CRC
	trajectoryMessageCrc = DEFAULT_CRC_INIT_VALUE;

	// Update CRC
	size_t dataLen = p - trajDataBuffer;
	char* crcPtr = trajDataBuffer;
	while (dataLen-- > 0) {
		trajectoryMessageCrc = crcByte(trajectoryMessageCrc, (uint8_t) (*crcPtr++));
	}
	return retval ? retval : p - trajDataBuffer;
}

/*!
 * \brief decodeTRAJMessageHeader
 * \param trajHeader Output data struct, to be used by host
 * \param trajDataBuffer Received trajectory data buffer
 * \param bufferLength Length of trajDataBuffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold header
 *		EMSGSIZE	if trajectory name is too long
 */
ssize_t decodeTRAJMessageHeader(
		TrajectoryHeaderType* trajHeader,
		const char* trajDataBuffer,
		const size_t bufferLength,
		const char debug) {

	TRAJHeaderType TRAJHeaderData;
	const char *p = trajDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (trajDataBuffer == NULL || trajHeader == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to TRAJ header parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&TRAJHeaderData, 0, sizeof (TRAJHeaderData));
	memset(trajHeader, 0, sizeof (*trajHeader));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &TRAJHeaderData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (TRAJHeaderData.header);

	// If message is not a  message, generate an error
	if (TRAJHeaderData.header.messageID != MESSAGE_ID_TRAJ) {
		fprintf(stderr, "Attempted to pass non-TRAJ message into TRAJ header parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}


	while (p - trajDataBuffer < sizeof (TRAJHeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_TRAJ_TRAJECTORY_IDENTIFIER:
			TRAJHeaderData.trajectoryIDValueID = valueID;
			TRAJHeaderData.trajectoryIDContentLength = contentLength;
			memcpy(&TRAJHeaderData.trajectoryID, p, sizeof (TRAJHeaderData.trajectoryID));
			expectedContentLength = sizeof (TRAJHeaderData.trajectoryID);
			break;
		case VALUE_ID_TRAJ_TRAJECTORY_NAME:
			TRAJHeaderData.trajectoryNameValueID = valueID;
			TRAJHeaderData.trajectoryNameContentLength = contentLength;
			memcpy(&TRAJHeaderData.trajectoryName, p, contentLength);
			expectedContentLength = TRAJ_NAME_STRING_MAX_LENGTH;
			break;
		case VALUE_ID_TRAJ_TRAJECTORY_INFO:
			TRAJHeaderData.trajectoryInfoValueID = valueID;
			TRAJHeaderData.trajectoryInfoContentLength = contentLength;
			memcpy(&TRAJHeaderData.trajectoryInfo, p, sizeof (TRAJHeaderData.trajectoryInfo));
			expectedContentLength = sizeof (TRAJHeaderData.trajectoryInfo);
			break;

		default:
			fprintf(stderr, "Value ID 0x%x does not match any known TRAJ header value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	if (debug) {
		printf("TRAJ header data:\n");
		printf("\tTrajectory ID: 0x%x\n", TRAJHeaderData.trajectoryID);
		printf("\tTrajectory name: %s\n", TRAJHeaderData.trajectoryName);
		printf("\tTrajectory info: %u\n", TRAJHeaderData.trajectoryInfo);
		printf("\tTRAJ length: %ld bytes\n", TRAJHeaderData.header.messageLength
			- sizeof(TRAJHeaderType) - sizeof(TRAJFooterType) + sizeof(FooterType));
	}

	// Fill output struct with parsed data
	convertTRAJHeaderToHostRepresentation(&TRAJHeaderData, TRAJHeaderData.header.messageLength, trajHeader);
	return retval < 0 ? retval : p - trajDataBuffer;
}

/*!
 * \brief convertTRAJHeaderToHostRepresentation Converts a TRAJ header message to be used by host
 * \param TRAJHeaderData Data struct containing ISO formatted data
 * \param trajectoryHeaderData Output data struct, to be used by host
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertTRAJHeaderToHostRepresentation(TRAJHeaderType* TRAJHeaderData,
				uint32_t trajectoryLength,	TrajectoryHeaderType* trajectoryHeaderData) {
	if (TRAJHeaderData == NULL || trajectoryHeaderData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "TRAJ header input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	trajectoryHeaderData->trajectoryID = TRAJHeaderData->trajectoryID;
	memcpy(trajectoryHeaderData->trajectoryName, TRAJHeaderData->trajectoryName, sizeof(TRAJHeaderData->trajectoryName));
	trajectoryHeaderData->trajectoryInfo = TRAJHeaderData->trajectoryInfo;
	trajectoryHeaderData->trajectoryLength = trajectoryLength - sizeof(TRAJHeaderType) - sizeof(TRAJFooterType) + sizeof(FooterType);
	trajectoryHeaderData->nWaypoints = trajectoryHeaderData->trajectoryLength/sizeof(TRAJPointType);

	return MESSAGE_OK;
}

/*!
 * \brief encodeTRAJMessagePoint Creates a TRAJ message point based on supplied values and updates an internal
 * CRC to be used in the footer. Also prints the TRAJ point to a buffer.
 * \param pointTimeFromStart Time from start of the trajectory point
 * \param position Position of the point
 * \param speed Speed at the point
 * \param acceleration Acceleration at the point
 * \param curvature Curvature of the trajectory at the point
 * \param trajDataBufferPointer Buffer to which the message is to be printed
 * \param remainingBufferLength Remaining bytes in the buffer to which the message is to be printed
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold point
 */
ssize_t encodeTRAJMessagePoint(const struct timeval *pointTimeFromStart, const CartesianPosition position,
							   const SpeedType speed, const AccelerationType acceleration,
							   const float curvature, char *trajDataBufferPointer,
							   const size_t remainingBufferLength, const char debug) {

	TRAJPointType TRAJData;
	size_t dataLen;

	if (remainingBufferLength < sizeof (TRAJPointType)) {
		errno = ENOBUFS;
		printf("Buffer too small to hold necessary TRAJ point data\n");
		return -1;
	}
	else if (trajDataBufferPointer == NULL) {
		errno = EINVAL;
		printf("Trajectory data buffer invalid\n");
		return -1;
	}

	// Fill contents
	TRAJData.relativeTimeValueID = VALUE_ID_TRAJ_RELATIVE_TIME;
	TRAJData.relativeTimeContentLength = sizeof (TRAJData.relativeTime);
	TRAJData.relativeTime =
		(uint32_t) (((double)(pointTimeFromStart->tv_sec) + pointTimeFromStart->tv_usec / 1000000.0)
					* RELATIVE_TIME_ONE_SECOND_VALUE);

	TRAJData.xPositionValueID = VALUE_ID_TRAJ_X_POSITION;
	TRAJData.xPositionContentLength = sizeof (TRAJData.xPosition);
	TRAJData.yPositionValueID = VALUE_ID_TRAJ_Y_POSITION;
	TRAJData.yPositionContentLength = sizeof (TRAJData.yPosition);
	TRAJData.zPositionValueID = VALUE_ID_TRAJ_Z_POSITION;
	TRAJData.zPositionContentLength = sizeof (TRAJData.zPosition);
	if (position.isPositionValid) {
		TRAJData.xPosition = (int32_t) (position.xCoord_m * POSITION_ONE_METER_VALUE);
		TRAJData.yPosition = (int32_t) (position.yCoord_m * POSITION_ONE_METER_VALUE);
		TRAJData.zPosition = (int32_t) (position.zCoord_m * POSITION_ONE_METER_VALUE);
	}
	else {
		errno = EINVAL;
		printf("Position is a required field in TRAJ messages\n");
		return -1;
	}

	TRAJData.headingValueID = VALUE_ID_TRAJ_HEADING;
	TRAJData.headingContentLength = sizeof (TRAJData.heading);
	if (position.isHeadingValid) {
		TRAJData.heading = (uint16_t) (mapHostHeadingToISOHeading(position.heading_rad)
									   * 180.0 / M_PI * HEADING_ONE_DEGREE_VALUE);
	}
	else {
		TRAJData.heading = HEADING_UNAVAILABLE_VALUE;
	}

	TRAJData.longitudinalSpeedValueID = VALUE_ID_TRAJ_LONGITUDINAL_SPEED;
	TRAJData.longitudinalSpeedContentLength = sizeof (TRAJData.longitudinalSpeed);
	TRAJData.lateralSpeedValueID = VALUE_ID_TRAJ_LATERAL_SPEED;
	TRAJData.lateralSpeedContentLength = sizeof (TRAJData.lateralSpeed);
	if (speed.isLongitudinalValid) {
		TRAJData.longitudinalSpeed = (int16_t) (speed.longitudinal_m_s * SPEED_ONE_METER_PER_SECOND_VALUE);
	}
	else {
		errno = EINVAL;
		printf("Longitudinal speed is a required field in TRAJ messages\n");
		return -1;
	}
	TRAJData.lateralSpeed =
		speed.isLateralValid ? (int16_t) (speed.lateral_m_s *
										  SPEED_ONE_METER_PER_SECOND_VALUE) : SPEED_UNAVAILABLE_VALUE;

	TRAJData.longitudinalAccelerationValueID = VALUE_ID_TRAJ_LONGITUDINAL_ACCELERATION;
	TRAJData.longitudinalAccelerationContentLength = sizeof (TRAJData.longitudinalAcceleration);
	TRAJData.lateralAccelerationValueID = VALUE_ID_TRAJ_LATERAL_ACCELERATION;
	TRAJData.lateralAccelerationContentLength = sizeof (TRAJData.lateralAcceleration);
	TRAJData.longitudinalAcceleration = acceleration.isLongitudinalValid ?
		(int16_t) (acceleration.longitudinal_m_s2 *
				   ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) : ACCELERATION_UNAVAILABLE_VALUE;
	TRAJData.lateralAcceleration =
		acceleration.isLateralValid ? (int16_t) (acceleration.lateral_m_s2 *
												 ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) :
		ACCELERATION_UNAVAILABLE_VALUE;

	TRAJData.curvatureValueID = VALUE_ID_TRAJ_CURVATURE;
	TRAJData.curvatureContentLength = sizeof (TRAJData.curvature);
	TRAJData.curvature = curvature;

	if (debug) {
		printf("TRAJ message point:\n\t"
			   "Relative time value ID: 0x%x\n\t"
			   "Relative time content length: %u\n\t"
			   "Relative time: %u\n\t"
			   "x position value ID: 0x%x\n\t"
			   "x position content length: %u\n\t"
			   "x position: %d\n\t"
			   "y position value ID: 0x%x\n\t"
			   "y position content length: %u\n\t"
			   "y position: %d\n\t"
			   "z position value ID: 0x%x\n\t"
			   "z position content length: %u\n\t"
			   "z position: %d\n\t"
			   "Heading value ID: 0x%x\n\t"
			   "Heading content length: %u\n\t"
			   "Heading: %u\n\t"
			   "Longitudinal speed value ID: 0x%x\n\t"
			   "Longitudinal speed content length: %u\n\t"
			   "Longitudinal speed: %d\n\t"
			   "Lateral speed value ID: 0x%x\n\t"
			   "Lateral speed content length: %u\n\t"
			   "Lateral speed: %d\n\t"
			   "Longitudinal acceleration value ID: 0x%x\n\t"
			   "Longitudinal acceleration content length: %u\n\t"
			   "Longitudinal acceleration: %d\n\t"
			   "Lateral acceleration value ID: 0x%x\n\t"
			   "Lateral acceleration content length: %u\n\t"
			   "Lateral acceleration: %d\n\t"
			   "Curvature value ID: 0x%x\n\t"
			   "Curvature content length: %u\n\t"
			   "Curvature: %.6f\n",
			   TRAJData.relativeTimeValueID, TRAJData.relativeTimeContentLength,
			   TRAJData.relativeTime, TRAJData.xPositionValueID, TRAJData.xPositionContentLength,
			   TRAJData.xPosition, TRAJData.yPositionValueID, TRAJData.yPositionContentLength,
			   TRAJData.yPosition, TRAJData.zPositionValueID, TRAJData.zPositionContentLength,
			   TRAJData.zPosition, TRAJData.headingValueID, TRAJData.headingContentLength,
			   TRAJData.heading, TRAJData.longitudinalSpeedValueID, TRAJData.longitudinalSpeedContentLength,
			   TRAJData.longitudinalSpeed, TRAJData.lateralSpeedValueID, TRAJData.lateralSpeedContentLength,
			   TRAJData.lateralSpeed, TRAJData.longitudinalAccelerationValueID,
			   TRAJData.longitudinalAccelerationContentLength, TRAJData.longitudinalAcceleration,
			   TRAJData.lateralAccelerationValueID, TRAJData.lateralAccelerationContentLength,
			   TRAJData.lateralAcceleration, TRAJData.curvatureValueID, TRAJData.curvatureContentLength,
			   (double_t) TRAJData.curvature);
	}

	// Convert from host endianness to little endian
	TRAJData.relativeTimeValueID = htole16(TRAJData.relativeTimeValueID);
	TRAJData.relativeTimeContentLength = htole16(TRAJData.relativeTimeContentLength);
	TRAJData.relativeTime = htole32(TRAJData.relativeTime);
	TRAJData.xPositionValueID = htole16(TRAJData.xPositionValueID);
	TRAJData.xPositionContentLength = htole16(TRAJData.xPositionContentLength);
	TRAJData.xPosition = (int32_t) htole32(TRAJData.xPosition);
	TRAJData.yPositionValueID = htole16(TRAJData.yPositionValueID);
	TRAJData.yPositionContentLength = htole16(TRAJData.yPositionContentLength);
	TRAJData.yPosition = (int32_t) htole32(TRAJData.yPosition);
	TRAJData.zPositionValueID = htole16(TRAJData.zPositionValueID);
	TRAJData.zPositionContentLength = htole16(TRAJData.zPositionContentLength);
	TRAJData.zPosition = (int32_t) htole32(TRAJData.zPosition);
	TRAJData.headingValueID = htole16(TRAJData.headingValueID);
	TRAJData.headingContentLength = htole16(TRAJData.headingContentLength);
	TRAJData.heading = htole16(TRAJData.heading);
	TRAJData.longitudinalSpeedValueID = htole16(TRAJData.longitudinalSpeedValueID);
	TRAJData.longitudinalSpeedContentLength = htole16(TRAJData.longitudinalSpeedContentLength);
	TRAJData.longitudinalSpeed = (int16_t) htole16(TRAJData.longitudinalSpeed);
	TRAJData.lateralSpeedValueID = htole16(TRAJData.lateralSpeedValueID);
	TRAJData.lateralSpeedContentLength = htole16(TRAJData.lateralSpeedContentLength);
	TRAJData.lateralSpeed = (int16_t) htole16(TRAJData.lateralSpeed);
	TRAJData.longitudinalAccelerationValueID = htole16(TRAJData.longitudinalAccelerationValueID);
	TRAJData.longitudinalAccelerationContentLength = htole16(TRAJData.longitudinalAccelerationContentLength);
	TRAJData.longitudinalAcceleration = (int16_t) htole16(TRAJData.longitudinalAcceleration);
	TRAJData.lateralAccelerationValueID = htole16(TRAJData.lateralAccelerationValueID);
	TRAJData.lateralAccelerationContentLength = htole16(TRAJData.lateralAccelerationContentLength);
	TRAJData.lateralAcceleration = (int16_t) htole16(TRAJData.lateralAcceleration);
	TRAJData.curvatureValueID = htole16(TRAJData.curvatureValueID);
	TRAJData.curvatureContentLength = htole16(TRAJData.curvatureContentLength);
	TRAJData.curvature = htolef(TRAJData.curvature);

	memcpy(trajDataBufferPointer, &TRAJData, sizeof (TRAJData));

	// Update CRC
	dataLen = sizeof (TRAJData);
	while (dataLen-- > 0) {
		trajectoryMessageCrc = crcByte(trajectoryMessageCrc, (uint8_t) (*trajDataBufferPointer++));
	}

	return sizeof (TRAJPointType);
}


/*!
 * \brief encodeTRAJMessageFooter Creates a TRAJ message footer based on an internal CRC from previous header
 * and points, and prints it to a buffer.
 * \param trajDataBuffer Buffer to which TRAJ message is to be printed
 * \param remainingBufferLength Remaining bytes in the buffer to which the message is to be printed
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold footer
 */
ssize_t encodeTRAJMessageFooter(char *trajDataBuffer, const size_t remainingBufferLength, const char debug) {

	TRAJFooterType TRAJData;

	if (remainingBufferLength < sizeof (TRAJFooterType)) {
		errno = ENOBUFS;
		printf("Buffer too small to hold TRAJ footer data\n");
		return -1;
	}
	else if (trajDataBuffer == NULL) {
		errno = EINVAL;
		printf("Invalid trajectory data buffer supplied\n");
		return -1;
	}

	TRAJData.footer.Crc = trajectoryMessageCrc;

	memcpy(trajDataBuffer, &TRAJData, sizeof (TRAJData));

	if (debug) {
		printf("Encoded ISO footer:\n\tCRC: 0x%x\n", TRAJData.footer.Crc);
	}

	return sizeof (TRAJFooterType);
}


/*!
 * \brief decodeTRAJMessagePoint
 * \param trajHeader Output data struct, to be used by host
 * \param trajDataBuffer Received trajectory data buffer
 * \param bufferLength Length of trajDataBuffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes printed, or -1 in case of error with the following errnos:
 *		EINVAL		if one of the input parameters are invalid
 *		ENOBUFS		if supplied buffer is too small to hold header
 *		EMSGSIZE	if trajectory name is too long
 */
ssize_t decodeTRAJMessagePoint(
		TrajectoryWaypointType* wayPoint,
		const char* trajDataBuffer,
		const char debug) {

	TRAJPointType TRAJPointData;
	const char *p = trajDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (trajDataBuffer == NULL || wayPoint == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to TRAJ points parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&TRAJPointData, 0, sizeof (TRAJPointData));
	memset(wayPoint, 0, sizeof (*wayPoint));

	while (p - trajDataBuffer < sizeof (TRAJPointData)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_TRAJ_RELATIVE_TIME:
			TRAJPointData.relativeTimeValueID = valueID;
			TRAJPointData.relativeTimeContentLength = contentLength;
			memcpy(&TRAJPointData.relativeTime, p, sizeof (TRAJPointData.relativeTime));
			expectedContentLength = sizeof (TRAJPointData.relativeTime);
			break;
		case VALUE_ID_TRAJ_X_POSITION:
			TRAJPointData.xPositionValueID = valueID;
			TRAJPointData.xPositionContentLength = contentLength;
			memcpy(&TRAJPointData.xPosition, p, sizeof (TRAJPointData.xPosition));
			expectedContentLength = sizeof(TRAJPointData.xPosition);
			break;
		case VALUE_ID_TRAJ_Y_POSITION:
			TRAJPointData.yPositionValueID = valueID;
			TRAJPointData.yPositionContentLength = contentLength;
			memcpy(&TRAJPointData.yPosition, p, sizeof (TRAJPointData.yPosition));
			expectedContentLength = sizeof(TRAJPointData.yPosition);
			break;
		case VALUE_ID_TRAJ_Z_POSITION:
			TRAJPointData.zPositionValueID = valueID;
			TRAJPointData.zPositionContentLength = contentLength;
			memcpy(&TRAJPointData.zPosition, p, sizeof (TRAJPointData.zPosition));
			expectedContentLength = sizeof(TRAJPointData.zPosition);
			break;
		case VALUE_ID_TRAJ_HEADING:
			TRAJPointData.headingValueID = valueID;
			TRAJPointData.headingContentLength = contentLength;
			memcpy(&TRAJPointData.heading, p, sizeof (TRAJPointData.heading));
			expectedContentLength = sizeof(TRAJPointData.heading);
			break;
		case VALUE_ID_TRAJ_LONGITUDINAL_SPEED:
			TRAJPointData.longitudinalSpeedValueID = valueID;
			TRAJPointData.longitudinalSpeedContentLength = contentLength;
			memcpy(&TRAJPointData.longitudinalSpeed, p, sizeof (TRAJPointData.longitudinalSpeed));
			expectedContentLength = sizeof(TRAJPointData.longitudinalSpeed);
			break;
		case VALUE_ID_TRAJ_LATERAL_SPEED:
			TRAJPointData.lateralSpeedValueID = valueID;
			TRAJPointData.lateralSpeedContentLength = contentLength;
			memcpy(&TRAJPointData.lateralSpeed, p, sizeof (TRAJPointData.lateralSpeed));
			expectedContentLength = sizeof(TRAJPointData.lateralSpeed);
			break;
		case VALUE_ID_TRAJ_LONGITUDINAL_ACCELERATION:
			TRAJPointData.longitudinalAccelerationValueID = valueID;
			TRAJPointData.longitudinalAccelerationContentLength = contentLength;
			memcpy(&TRAJPointData.longitudinalAcceleration, p, sizeof (TRAJPointData.longitudinalAcceleration));
			expectedContentLength = sizeof(TRAJPointData.longitudinalAcceleration);
			break;
		case VALUE_ID_TRAJ_LATERAL_ACCELERATION:
			TRAJPointData.lateralAccelerationValueID = valueID;
			TRAJPointData.lateralAccelerationContentLength = contentLength;
			memcpy(&TRAJPointData.lateralAcceleration, p, sizeof (TRAJPointData.lateralAcceleration));
			expectedContentLength = sizeof(TRAJPointData.lateralAcceleration);
			break;
		case VALUE_ID_TRAJ_CURVATURE:
			TRAJPointData.curvatureValueID = valueID;
			TRAJPointData.curvatureContentLength = contentLength;
			memcpy(&TRAJPointData.curvature, p, sizeof (TRAJPointData.curvature));
			expectedContentLength = sizeof(TRAJPointData.curvature);
			break;

		default:
			fprintf(stderr, "Value ID 0x%x does not match any known TRAJ point value IDs\n", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld\n",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	TRAJPointData.relativeTime = le32toh(TRAJPointData.relativeTime);
	TRAJPointData.xPosition = (int32_t)le32toh(TRAJPointData.xPosition);
	TRAJPointData.yPosition = (int32_t)le32toh(TRAJPointData.yPosition);
	TRAJPointData.zPosition = (int32_t)le32toh(TRAJPointData.zPosition);
	TRAJPointData.heading = le16toh(TRAJPointData.heading);
	TRAJPointData.longitudinalSpeed = (int16_t)le16toh(TRAJPointData.longitudinalSpeed);
	TRAJPointData.lateralSpeed = (int16_t)le16toh(TRAJPointData.lateralSpeed);
	TRAJPointData.longitudinalAcceleration = (int16_t)le16toh(TRAJPointData.longitudinalAcceleration);
	TRAJPointData.lateralAcceleration = (int16_t)le16toh(TRAJPointData.lateralAcceleration);
	// curvature is floating point, let's not mess with endianness?

	if (debug) {
		printf("TRAJ point data:\n");
		printf("\tTRAJPointData.relativeTimeValueID: %x\n", TRAJPointData.relativeTimeValueID);
		printf("\tTRAJPointData.relativeTimeContentLength : %d\n", TRAJPointData.relativeTimeContentLength);
		printf("\tTRAJPointData Time: %d\n", TRAJPointData.relativeTime);
		printf("\tTRAJPointData X: %d\n", TRAJPointData.xPosition);
		printf("\tTRAJPointData Y: %d\n", TRAJPointData.yPosition);
		printf("\tTRAJPointData Z: %d\n", TRAJPointData.zPosition);
		printf("\tTRAJPointData Heading: %d\n", TRAJPointData.heading);
		printf("\tTRAJPointData Longitudinal speed: %d\n", TRAJPointData.longitudinalSpeed);
		printf("\tTRAJPointData Lateral speed: %d\n", TRAJPointData.lateralSpeed);
		printf("\tTRAJPointData Longitudinal acceleration: %d\n", TRAJPointData.longitudinalAcceleration);
		printf("\tTRAJPointData Lateral acceleration: %d\n", TRAJPointData.lateralAcceleration);
		printf("\tTRAJPointData Curvature: %3.3f\n", TRAJPointData.curvature);
		printf("\tRaw data: ");
		for(int i = 0; i<sizeof(TRAJPointType); i ++) printf("%x-", (unsigned char*)trajDataBuffer[i]);
		printf("\n");
	}

	// Fill output struct with parsed data
	convertTRAJPointToHostRepresentation(&TRAJPointData, wayPoint);
	return retval < 0 ? retval : p - trajDataBuffer;
}



/*!
 * \brief convertTRAJPointToHostRepresentation Converts a TRAJ header message to be used by host
 * \param TRAJPointData Data struct containing ISO formatted data
 * \param wayPoint Output data struct, to be used by host
 * \return Value according to ::ISOMessageReturnValue
 */
enum ISOMessageReturnValue convertTRAJPointToHostRepresentation(
		TRAJPointType* TRAJPointData,
		TrajectoryWaypointType* wayPoint) {
	
	if (TRAJPointData == NULL || wayPoint == NULL) {
		errno = EINVAL;
		fprintf(stderr, "TRAJ point input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	//Need to check data available and endians here ....
	wayPoint->relativeTime.tv_sec = (long) (TRAJPointData->relativeTime / RELATIVE_TIME_ONE_SECOND_VALUE);
	wayPoint->relativeTime.tv_usec = (long) ((TRAJPointData->relativeTime
			- wayPoint->relativeTime.tv_sec * RELATIVE_TIME_ONE_SECOND_VALUE) / RELATIVE_TIME_ONE_SECOND_VALUE
			* 1000000.0);
	wayPoint->pos.xCoord_m = TRAJPointData->xPosition / POSITION_ONE_METER_VALUE;
	wayPoint->pos.yCoord_m = TRAJPointData->yPosition / POSITION_ONE_METER_VALUE;
	wayPoint->pos.zCoord_m = TRAJPointData->zPosition / POSITION_ONE_METER_VALUE;
	wayPoint->pos.isPositionValid = TRAJPointData->xPositionValueID != 0
			&& TRAJPointData->yPositionValueID  != 0 && TRAJPointData->zPositionValueID != 0;
	wayPoint->pos.heading_rad = TRAJPointData->heading / HEADING_ONE_DEGREE_VALUE * M_PI / 180.0;
	wayPoint->pos.heading_rad = mapISOHeadingToHostHeading(wayPoint->pos.heading_rad);
	wayPoint->pos.isHeadingValid = TRAJPointData->headingValueID != 0;
	wayPoint->spd.longitudinal_m_s = TRAJPointData->longitudinalSpeed / SPEED_ONE_METER_PER_SECOND_VALUE;
	wayPoint->spd.isLongitudinalValid = TRAJPointData->longitudinalSpeedValueID != 0;
	wayPoint->spd.lateral_m_s = TRAJPointData->lateralSpeed / SPEED_ONE_METER_PER_SECOND_VALUE;
	wayPoint->spd.isLateralValid = TRAJPointData->lateralSpeedValueID != 0;
	wayPoint->acc.longitudinal_m_s2 = TRAJPointData->longitudinalAcceleration
			/ ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE;
	wayPoint->acc.isLongitudinalValid = TRAJPointData->longitudinalAccelerationValueID != 0;
	wayPoint->acc.lateral_m_s2 = TRAJPointData->lateralAcceleration
			/ ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE;
	wayPoint->acc.isLateralValid = TRAJPointData->lateralAccelerationValueID != 0;
	wayPoint->curvature = TRAJPointData->curvature;
	
	return MESSAGE_OK;
}