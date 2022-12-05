#pragma once
#include "header.h"
#include "footer.h"
#include "defines.h"
#include <stdint.h>

#pragma pack(push, 1)
/*! TRAJ message */
#define TRAJ_NAME_STRING_MAX_LENGTH 64
typedef struct {
	HeaderType header;
	uint16_t trajectoryIDValueID;
	uint16_t trajectoryIDContentLength;
	uint16_t trajectoryID;
	uint16_t trajectoryNameValueID;
	uint16_t trajectoryNameContentLength;
	char trajectoryName[TRAJ_NAME_STRING_MAX_LENGTH];
	uint16_t trajectoryVersionValueID;
	uint16_t trajectoryVersionContentLength;
	uint16_t trajectoryVersion;
} TRAJHeaderType;

//! TRAJ header field descriptions
static DebugStrings_t TrajectoryIDDescription = {"Trajectory ID",	"",			&printU32};
static DebugStrings_t TrajectoryNameDescription = {"Trajectory name",	"",		&printU32};
static DebugStrings_t TrajectoryVersionDescription = {"Trajectory version",	"",	&printU32};

typedef struct {
	uint16_t relativeTimeValueID;
	uint16_t relativeTimeContentLength;
	uint32_t relativeTime;
	uint16_t xPositionValueID;
	uint16_t xPositionContentLength;
	int32_t xPosition;
	uint16_t yPositionValueID;
	uint16_t yPositionContentLength;
	int32_t yPosition;
	uint16_t zPositionValueID;
	uint16_t zPositionContentLength;
	int32_t zPosition;
	uint16_t headingValueID;
	uint16_t headingContentLength;
	uint16_t heading;
	uint16_t longitudinalSpeedValueID;
	uint16_t longitudinalSpeedContentLength;
	int16_t longitudinalSpeed;
	uint16_t lateralSpeedValueID;
	uint16_t lateralSpeedContentLength;
	int16_t lateralSpeed;
	uint16_t longitudinalAccelerationValueID;
	uint16_t longitudinalAccelerationContentLength;
	int16_t longitudinalAcceleration;
	uint16_t lateralAccelerationValueID;
	uint16_t lateralAccelerationContentLength;
	int16_t lateralAcceleration;
	uint16_t curvatureValueID;
	uint16_t curvatureContentLength;
	float_t curvature;
} TRAJPointType;

typedef struct {
	FooterType footer;
} TRAJFooterType;
#pragma pack(pop)

//! TRAJ value IDs
#define VALUE_ID_TRAJ_TRAJECTORY_IDENTIFIER 0x0101
#define VALUE_ID_TRAJ_TRAJECTORY_NAME 0x0102
#define VALUE_ID_TRAJ_TRAJECTORY_VERSION 0x0103
#define VALUE_ID_TRAJ_RELATIVE_TIME 0x0001
#define VALUE_ID_TRAJ_X_POSITION 0x0010
#define VALUE_ID_TRAJ_Y_POSITION 0x0011
#define VALUE_ID_TRAJ_Z_POSITION 0x0012
#define VALUE_ID_TRAJ_HEADING 0x0030
#define VALUE_ID_TRAJ_LONGITUDINAL_SPEED 0x0040
#define VALUE_ID_TRAJ_LATERAL_SPEED 0x0041
#define VALUE_ID_TRAJ_LONGITUDINAL_ACCELERATION 0x0050
#define VALUE_ID_TRAJ_LATERAL_ACCELERATION 0x0051
#define VALUE_ID_TRAJ_CURVATURE 0x0052

static enum ISOMessageReturnValue convertTRAJHeaderToHostRepresentation(TRAJHeaderType* TRAJHeaderData,
				uint32_t trajectoryLength,	TrajectoryHeaderType* trajectoryHeaderData);
static enum ISOMessageReturnValue convertTRAJPointToHostRepresentation(TRAJPointType* TRAJPointData,
														TrajectoryWaypointType* wayPoint);
