#include "iso22133.h"
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// Temporary fix since HAVE_BYTESWAP_H does not appear to work for linux
// TODO: remove once several systems start using this library
// Windows mostly runs on little endian architectures
// TODO: Future-proof - make below endian conversion host independent on Windows
#if defined _WIN32 || defined __APPLE__
#	define htole16(value) (value)
#	define htole32(value) (value)
#	define htole64(value) (value)

#	define le16toh(value) (value)
#	define le32toh(value) (value)

#	if defined __GNUC__
#		define htobe16(value) __builtin_bswap16(value)

#	elif defined (_MSC_VER)
#		include <stdlib.h>
#		define htobe16(x) _byteswap_ushort(x)
#   endif

#elif __linux__
#	include <byteswap.h>
#	include <endian.h>
#endif



// ************************* Global ISO protocol settings ********************************************************
static const uint8_t SupportedProtocolVersions[] = { 2 };

#define ISO_PROTOCOL_VERSION 2	//!< ISO protocol version of messages sent
#define ACK_REQ 0


// ************************* Debug printout helper data **********************************************************
/*! Debug formatting functions*/
typedef void (*DebugPrinter_t)(const void*);
void printU8(const void* val)  { printf("%u",  *(const uint8_t*)val); }
void printU16(const void* val) { printf("%u",  *(const uint16_t*)val); }
void printU32(const void* val) { printf("%u",  *(const uint32_t*)val); }
void printU64(const void* val) { printf("%lu", *(const uint64_t*)val); }
void printI8(const void* val)  { printf("%d",  *(const int8_t*)val); }
void printI16(const void* val) { printf("%d",  *(const int16_t*)val); }
void printI32(const void* val) { printf("%d",  *(const int32_t*)val); }
void printI64(const void* val) { printf("%ld", *(const int64_t*)val); }

/*! Debug struct */
typedef struct {
	char* name;
	char* unit;
	DebugPrinter_t printer;
} DebugStrings_t;

// ************************* Type definitions according ISO protocol specification *******************************
//! Predefined integer values with special meaning
#define ISO_SYNC_WORD 0x7E7E
#define TRANSMITTER_ID_UNAVAILABLE_VALUE UINT32_MAX
#define LATITUDE_UNAVAILABLE_VALUE 900000000001
#define LATITUDE_ONE_DEGREE_VALUE 10000000000.0
#define LONGITUDE_UNAVAILABLE_VALUE 1800000000001
#define LONGITUDE_ONE_DEGREE_VALUE 10000000000.0
#define ALTITUDE_UNAVAILABLE_VALUE 800001
#define ALTITUDE_ONE_METER_VALUE 100.0
#define DATE_UNAVAILABLE_VALUE 0
#define GPS_WEEK_UNAVAILABLE_VALUE 10001
#define GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE 2419200000
#define MAX_WAY_DEVIATION_UNAVAILABLE_VALUE 65535
#define MAX_WAY_DEVIATION_ONE_METER_VALUE 1000.0
#define MAX_LATERAL_DEVIATION_UNAVAILABLE_VALUE 65535
#define MAX_LATERAL_DEVIATION_ONE_METER_VALUE 1000.0
#define MIN_POSITIONING_ACCURACY_NOT_REQUIRED_VALUE 0
#define MIN_POSITIONING_ACCURACY_ONE_METER_VALUE 1000.0	// ISO specification unclear on this value
#define TRIGGER_ID_UNAVAILABLE 65535
#define TRIGGER_TYPE_UNAVAILABLE 65535
#define TRIGGER_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define ACTION_ID_UNAVAILABLE 65535
#define ACTION_TYPE_UNAVAILABLE 65535
#define ACTION_TYPE_PARAMETER_UNAVAILABLE 4294967295
#define POSITION_ONE_METER_VALUE 1000.0
#define POSITION_UNAVAILABLE_VALUE (-32768)
#define LENGTH_ONE_METER_VALUE 1000.0
#define LENGTH_UNAVAILABLE_VALUE UINT32_MAX
#define MASS_ONE_KILOGRAM_VALUE 1000.0
#define MASS_UNAVAILABLE_VALUE UINT32_MAX
#define HEADING_UNAVAILABLE_VALUE 36001
#define HEADING_ONE_DEGREE_VALUE 100.0
#define PITCH_UNAVAILABLE_VALUE 36001
#define PITCH_ONE_DEGREE_VALUE 100.0
#define ROLL_UNAVAILABLE_VALUE 36001
#define ROLL_ONE_DEGREE_VALUE 100.0
#define SPEED_UNAVAILABLE_VALUE (-32768)
#define SPEED_ONE_METER_PER_SECOND_VALUE 100.0
#define ACCELERATION_UNAVAILABLE_VALUE 32001
#define ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE 1000.0
#define RELATIVE_TIME_ONE_SECOND_VALUE 1000.0
#define STEERING_ANGLE_ONE_DEGREE_VALUE 100.0
#define STEERING_ANGLE_MAX_VALUE_DEG 18000
#define STEERING_ANGLE_MIN_VALUE_DEG (-18000)
#define STEERING_ANGLE_MAX_VALUE_RAD M_PI
#define STEERING_ANGLE_MIN_VALUE_RAD (-M_PI)
#define STEERING_ANGLE_UNAVAILABLE_VALUE 18001
#define MAX_VALUE_PERCENTAGE 100
#define MIN_VALUE_PERCENTAGE (-100)



#define DEFAULT_CRC_INIT_VALUE 0x0000
#define DEFAULT_CRC_CHECK_ENABLED 1

typedef enum {
	ISO_DRIVE_DIRECTION_FORWARD = 0,
	ISO_DRIVE_DIRECTION_BACKWARD = 1,
	ISO_DRIVE_DIRECTION_UNAVAILABLE = 2
} DriveDirectionValues;
typedef enum {
	ISO_OBJECT_STATE_OFF = 0,
	ISO_OBJECT_STATE_INIT = 1,
	ISO_OBJECT_STATE_ARMED = 2,
	ISO_OBJECT_STATE_DISARMED = 3,
	ISO_OBJECT_STATE_RUNNING = 4,
	ISO_OBJECT_STATE_POSTRUN = 5,
	ISO_OBJECT_STATE_REMOTE_CONTROLLED = 6,
	ISO_OBJECT_STATE_ABORTING = 7
} ObjectStateValues;
typedef enum {
	ISO_NOT_READY_TO_ARM = 0,
	ISO_READY_TO_ARM = 1,
	ISO_READY_TO_ARM_UNAVAILABLE = 2
} ArmReadinessValues;

#define BITMASK_ERROR_ABORT_REQUEST				0x80
#define BITMASK_ERROR_OUTSIDE_GEOFENCE			0x40
#define BITMASK_ERROR_BAD_POSITIONING_ACCURACY	0x20
#define BITMASK_ERROR_ENGINE_FAULT				0x10
#define BITMASK_ERROR_BATTERY_FAULT				0x08
#define BITMASK_ERROR_OTHER						0x04
#define BITMASK_ERROR_SYNC_POINT_ENDED			0x02
#define BITMASK_ERROR_VENDOR_SPECIFIC			0x01



#pragma pack(push,1)			// Ensure sizeof() is useable for (most) network byte lengths
/*! ISO message header */
typedef struct {
	uint16_t SyncWordU16;
	uint8_t TransmitterIdU8;
	uint8_t MessageCounterU8;
	uint8_t AckReqProtVerU8;
	uint16_t MessageIdU16;
	uint32_t MessageLengthU32;
} HeaderType;

/*! ISO message footer */
typedef struct {
	uint16_t Crc;
} FooterType;


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


/*! OPRO message */
typedef struct {
	HeaderType header;
	uint16_t objectTypeValueID;
	uint16_t objectTypeContentLength;
	uint8_t objectType;
	uint16_t actorTypeValueID;
	uint16_t actorTypeContentLength;
	uint8_t actorType;
	uint16_t operationModeValueID;
	uint16_t operationModeContentLength;
	uint8_t operationMode;
	uint16_t massValueID;
	uint16_t massContentLength;
	uint32_t mass;
	uint16_t objectLengthXValueID;
	uint16_t objectLengthXContentLength;
	uint32_t objectLengthX;
	uint16_t objectLengthYValueID;
	uint16_t objectLengthYContentLength;
	uint32_t objectLengthY;
	uint16_t objectLengthZValueID;
	uint16_t objectLengthZContentLength;
	uint32_t objectLengthZ;
	uint16_t positionDisplacementXValueID;
	uint16_t positionDisplacementXContentLength;
	int16_t positionDisplacementX;
	uint16_t positionDisplacementYValueID;
	uint16_t positionDisplacementYContentLength;
	int16_t positionDisplacementY;
	uint16_t positionDisplacementZValueID;
	uint16_t positionDisplacementZContentLength;
	int16_t positionDisplacementZ;
	FooterType footer;
} OPROType;

#define VALUE_ID_OPRO_OBJECT_TYPE 0x0100
#define VALUE_ID_OPRO_ACTOR_TYPE 0x0101
#define VALUE_ID_OPRO_OPERATION_MODE 0x0102
#define VALUE_ID_OPRO_MASS 0x0103
#define VALUE_ID_OPRO_OBJECT_LENGTH_X 0x0104
#define VALUE_ID_OPRO_OBJECT_LENGTH_Y 0x0105
#define VALUE_ID_OPRO_OBJECT_LENGTH_Z 0x0106
#define VALUE_ID_OPRO_POSITION_DISPLACEMENT_X 0x0107
#define VALUE_ID_OPRO_POSITION_DISPLACEMENT_Y 0x0108
#define VALUE_ID_OPRO_POSITION_DISPLACEMENT_Z 0x0109

//! OPRO field descriptions
static DebugStrings_t OPROObjectTypeDescription =				{"ObjectType",				"",		&printU8};
static DebugStrings_t OPROActorTypeDescription =				{"ActorType",				"",		&printU8};
static DebugStrings_t OPROOperationModeDescription =			{"OperationMode",			"",		&printU8};
static DebugStrings_t OPROMassDescription =						{"Mass",					"[g]",	&printU32};
static DebugStrings_t OPROObjectLengthXDescription =			{"Object length X",			"[mm]", &printU32};
static DebugStrings_t OPROObjectLengthYDescription =			{"Object length Y",			"[mm]", &printU32};
static DebugStrings_t OPROObjectLengthZDescription =			{"Object length Z",			"[mm]", &printU32};
static DebugStrings_t OPROPositionDisplacementXDescription =	{"Position displacement X", "[mm]", &printI16};
static DebugStrings_t OPROPositionDisplacementYDescription =	{"Position displacement Y", "[mm]", &printI16};
static DebugStrings_t OPROPositionDisplacementZDescription =	{"Position displacement Z", "[mm]", &printI16};


/*! FOPR message */
typedef struct {
	HeaderType header;
	uint16_t foreignTransmitterIDValueID;
	uint16_t foreignTransmitterIDContentLength;
	uint32_t foreignTransmitterID;
	uint16_t objectTypeValueID;
	uint16_t objectTypeContentLength;
	uint8_t objectType;
	uint16_t actorTypeValueID;
	uint16_t actorTypeContentLength;
	uint8_t actorType;
	uint16_t operationModeValueID;
	uint16_t operationModeContentLength;
	uint8_t operationMode;
	uint16_t massValueID;
	uint16_t massContentLength;
	uint32_t mass;
	uint16_t objectLengthXValueID;
	uint16_t objectLengthXContentLength;
	uint32_t objectLengthX;
	uint16_t objectLengthYValueID;
	uint16_t objectLengthYContentLength;
	uint32_t objectLengthY;
	uint16_t objectLengthZValueID;
	uint16_t objectLengthZContentLength;
	uint32_t objectLengthZ;
	uint16_t positionDisplacementXValueID;
	uint16_t positionDisplacementXContentLength;
	int16_t positionDisplacementX;
	uint16_t positionDisplacementYValueID;
	uint16_t positionDisplacementYContentLength;
	int16_t positionDisplacementY;
	uint16_t positionDisplacementZValueID;
	uint16_t positionDisplacementZContentLength;
	int16_t positionDisplacementZ;
	FooterType footer;
} FOPRType;

#define VALUE_ID_FOPR_FOREIGN_TRANSMITTER_ID	0x00FF
#define VALUE_ID_FOPR_OBJECT_TYPE 0x0100
#define VALUE_ID_FOPR_ACTOR_TYPE 0x0101
#define VALUE_ID_FOPR_OPERATION_MODE 0x0102
#define VALUE_ID_FOPR_MASS 0x0103
#define VALUE_ID_FOPR_OBJECT_LENGTH_X 0x0104
#define VALUE_ID_FOPR_OBJECT_LENGTH_Y 0x0105
#define VALUE_ID_FOPR_OBJECT_LENGTH_Z 0x0106
#define VALUE_ID_FOPR_POSITION_DISPLACEMENT_X 0x0107
#define VALUE_ID_FOPR_POSITION_DISPLACEMENT_Y 0x0108
#define VALUE_ID_FOPR_POSITION_DISPLACEMENT_Z 0x0109

static DebugStrings_t FOPRForeignTransmitterIDDescription = 	{"Foreign transmitter ID", "", &printU32};
static DebugStrings_t FOPRObjectTypeDescription = 				{"ObjectType", "", &printU8};
static DebugStrings_t FOPRActorTypeDescription = 				{"ActorType", "", &printU8};
static DebugStrings_t FOPROperationModeDescription = 			{"OperationMode", "", &printU8};
static DebugStrings_t FOPRMassDescription = 					{"Mass", "[g]", &printU32};
static DebugStrings_t FOPRObjectLengthXDescription = 			{"Object length X", "[mm]", &printU32};
static DebugStrings_t FOPRObjectLengthYDescription = 			{"Object length Y", "[mm]", &printU32};
static DebugStrings_t FOPRObjectLengthZDescription = 			{"Object length Z", "[mm]", &printU32};
static DebugStrings_t FOPRPositionDisplacementXDescription = 	{"Position displacement X", "[mm]", &printI16};
static DebugStrings_t FOPRPositionDisplacementYDescription = 	{"Position displacement Y", "[mm]", &printI16};
static DebugStrings_t FOPRPositionDisplacementZDescription = 	{"Position displacement Z", "[mm]", &printI16};


/*! OSTM message */
typedef struct {
	HeaderType header;
	uint16_t stateValueID;
	uint16_t stateContentLength;
	uint8_t state;
	FooterType footer;
} OSTMType;						//16 bytes

//! OSTM value IDs
#define VALUE_ID_OSTM_STATE_CHANGE_REQUEST 0x0064


/*! STRT message */
typedef struct {
	HeaderType header;
	uint16_t StartTimeValueIdU16;
	uint16_t StartTimeContentLengthU16;
	uint32_t StartTimeU32;
	uint16_t GPSWeekValueID;
	uint16_t GPSWeekContentLength;
	uint16_t GPSWeek;
	FooterType footer;
} STRTType;						//27 bytes

//! STRT value IDs
#define VALUE_ID_STRT_GPS_QMS_OF_WEEK 0x0002
#define VALUE_ID_STRT_GPS_WEEK 0x0003


//! MONR message */
typedef struct {
	HeaderType header;
	uint16_t monrStructValueID;
	uint16_t monrStructContentLength;
	uint32_t gpsQmsOfWeek;
	int32_t xPosition;
	int32_t yPosition;
	int32_t zPosition;
	uint16_t heading;
	int16_t longitudinalSpeed;
	int16_t lateralSpeed;
	int16_t longitudinalAcc;
	int16_t lateralAcc;
	uint8_t driveDirection;
	uint8_t state;
	uint8_t readyToArm;
	uint8_t errorStatus;
	FooterType footer;
} MONRType;

//! MONR value IDs
#define VALUE_ID_MONR_STRUCT 0x80


/*! HEAB message */
typedef struct {
	HeaderType header;
	uint16_t HEABStructValueID;
	uint16_t HEABStructContentLength;
	uint32_t GPSQmsOfWeek;
	uint8_t controlCenterStatus;
	FooterType footer;
} HEABType;						//16 bytes

//! HEAB value IDs
#define VALUE_ID_HEAB_STRUCT 0x0090


/*! SYPM message */
typedef struct {
	HeaderType header;
	uint16_t syncPointTimeValueID;
	uint16_t syncPointTimeContentLength;
	uint32_t syncPointTime;
	uint16_t freezeTimeValueID;
	uint16_t freezeTimeContentLength;
	uint32_t freezeTime;
	FooterType footer;
} SYPMType;

//! SYPM value IDs
#define VALUE_ID_SYPM_SYNC_POINT_TIME 0x0001
#define VALUE_ID_SYPM_FREEZE_TIME 0x0002


/*! MTSP message */
typedef struct {
	HeaderType header;
	uint16_t estSyncPointTimeValueID;
	uint16_t estSyncPointTimeContentLength;
	uint32_t estSyncPointTime;
	FooterType footer;
} MTSPType;

//! MTSP value IDs
#define VALUE_ID_MTSP_EST_SYNC_POINT_TIME 0x0001


/*! TRCM message */
typedef struct {
	HeaderType header;
	uint16_t triggerIDValueID;
	uint16_t triggerIDContentLength;
	uint16_t triggerID;
	uint16_t triggerTypeValueID;
	uint16_t triggerTypeContentLength;
	uint16_t triggerType;
	uint16_t triggerTypeParameter1ValueID;
	uint16_t triggerTypeParameter1ContentLength;
	uint32_t triggerTypeParameter1;
	uint16_t triggerTypeParameter2ValueID;
	uint16_t triggerTypeParameter2ContentLength;
	uint32_t triggerTypeParameter2;
	uint16_t triggerTypeParameter3ValueID;
	uint16_t triggerTypeParameter3ContentLength;
	uint32_t triggerTypeParameter3;
	FooterType footer;
} TRCMType;

//! TRCM value IDs
#define VALUE_ID_TRCM_TRIGGER_ID 0x0001
#define VALUE_ID_TRCM_TRIGGER_TYPE 0x0002
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM1 0x0011
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM2 0x0012
#define VALUE_ID_TRCM_TRIGGER_TYPE_PARAM3 0x0013


/*! TREO message */
typedef struct {
	HeaderType header;
	uint16_t triggerIDValueID;
	uint16_t triggerIDContentLength;
	uint16_t triggerID;
	uint16_t timestamp_qmsowValueID;
	uint16_t timestamp_qmsowContentLength;
	uint32_t timestamp_qmsow;
	FooterType footer;
} TREOType;

//! TREO value IDs
#define VALUE_ID_TREO_TRIGGER_ID 0x0001
#define VALUE_ID_TREO_TRIGGER_TIMESTAMP 0x0002


/*! ACCM message */
typedef struct {
	HeaderType header;
	uint16_t actionIDValueID;
	uint16_t actionIDContentLength;
	uint16_t actionID;
	uint16_t actionTypeValueID;
	uint16_t actionTypeContentLength;
	uint16_t actionType;
	uint16_t actionTypeParameter1ValueID;
	uint16_t actionTypeParameter1ContentLength;
	uint32_t actionTypeParameter1;
	uint16_t actionTypeParameter2ValueID;
	uint16_t actionTypeParameter2ContentLength;
	uint32_t actionTypeParameter2;
	uint16_t actionTypeParameter3ValueID;
	uint16_t actionTypeParameter3ContentLength;
	uint32_t actionTypeParameter3;
	FooterType footer;
} ACCMType;

//! ACCM value IDs
#define VALUE_ID_ACCM_ACTION_ID 0x0002
#define VALUE_ID_ACCM_ACTION_TYPE 0x0003
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM1 0x00A1
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM2 0x00A2
#define VALUE_ID_ACCM_ACTION_TYPE_PARAM3 0x00A3


/*! EXAC message */
typedef struct {
	HeaderType header;
	uint16_t actionIDValueID;
	uint16_t actionIDContentLength;
	uint16_t actionID;
	uint16_t executionTime_qmsoWValueID;
	uint16_t executionTime_qmsoWContentLength;
	uint32_t executionTime_qmsoW;
	FooterType footer;
} EXACType;

//! EXAC value IDs
#define VALUE_ID_EXAC_ACTION_ID 0x0002
#define VALUE_ID_EXAC_ACTION_EXECUTE_TIME 0x0003

/*! CATA message */
// TODO
//! CATA value IDs
// TODO


/*! INSUP message */
typedef struct {
	HeaderType header;
	uint16_t modeValueID;
	uint16_t modeContentLength;
	uint8_t mode;
	FooterType footer;
} INSUPType;

//! INSUP value IDs
#define VALUE_ID_INSUP_MODE 0x0200

/*! PODI message */
typedef struct {
	HeaderType header;
	uint16_t foreignTransmitterIDValueID;
	uint16_t foreignTransmitterIDContentLength;
	uint32_t foreignTransmitterID;
	uint16_t gpsQmsOfWeekValueID;
	uint16_t gpsQmsOfWeekContentLength;
	uint32_t gpsQmsOfWeek;
	uint16_t objectStateValueID;
	uint16_t objectStateContentLength;
	uint8_t  objectState;
	uint16_t xPositionValueID;
	uint16_t xPositionContentLength;
	int32_t  xPosition;
	uint16_t yPositionValueID;
	uint16_t yPositionContentLength;
	int32_t  yPosition;
	uint16_t zPositionValueID;
	uint16_t zPositionContentLength;
	int32_t  zPosition;
	uint16_t headingValueID;
	uint16_t headingContentLength;
	uint16_t heading;
	uint16_t pitchValueID;
	uint16_t pitchContentLength;
	uint16_t pitch;
	uint16_t rollValueID;
	uint16_t rollContentLength;
	uint16_t roll;
	uint16_t longitudinalSpeedValueID;
	uint16_t longitudinalSpeedContentLength;
	int16_t  longitudinalSpeed;
	uint16_t lateralSpeedValueID;
	uint16_t lateralSpeedContentLength;
	int16_t  lateralSpeed;
	FooterType footer;
} PODIType;

//! PODI value IDs
#define VALUE_ID_PODI_FOREIGN_TRANSMITTER_ID	0x00FF
#define VALUE_ID_PODI_GPS_QMS_OF_WEEK			0x010A
#define VALUE_ID_PODI_OBJECT_STATE				0x010C
#define VALUE_ID_PODI_X_POSITION				0x010D
#define VALUE_ID_PODI_Y_POSITION				0x010E
#define VALUE_ID_PODI_Z_POSITION				0x010F
#define VALUE_ID_PODI_HEADING					0x0110
#define VALUE_ID_PODI_PITCH						0x0111
#define VALUE_ID_PODI_ROLL						0x0112
#define VALUE_ID_PODI_LONGITUDINAL_SPEED		0x0113
#define VALUE_ID_PODI_LATERAL_SPEED				0x0114

//! PODI field descriptions
static DebugStrings_t PODIForeignTransmitterIdDescription = {"ForeignTransmitterID",	"",			&printU32};
static DebugStrings_t PODIGpsQmsOfWeekDescription =			{"GpsQmsOfWeek",			"[¼ ms]",	&printU32};
static DebugStrings_t PODIObjectStateDescription =			{"ObjectState",				"",			&printU8};
static DebugStrings_t PODIxPositionDescription =			{"Object X position",		"[mm]",		&printI32};
static DebugStrings_t PODIyPositionDescription =			{"Object Y position",		"[mm]",		&printI32};
static DebugStrings_t PODIzPositionDescription =			{"Object Z position",		"[mm]",		&printI32};
static DebugStrings_t PODIHeadingDescription =				{"Object heading (yaw)",	"[cdeg]",	&printU16};
static DebugStrings_t PODIPitchDescription =				{"Object pitch",			"[cdeg]",	&printU16};
static DebugStrings_t PODIRollDescription =					{"Object roll",				"[cdeg]",	&printU16};
static DebugStrings_t PODILongitudinalSpeedDescription =	{"Longitudinal speed",		"[cm/s]",	&printI16};
static DebugStrings_t PODILateralSpeedDescription =			{"Lateral speed",			"[cm/s]",	&printI16};

typedef struct{
	HeaderType header;
	uint16_t statusValueID;
	uint16_t statusContentLength;
	uint8_t status;
	FooterType footer;
} GREMType;

//! GREM value IDs
#define VALUE_ID_GREM_STATUS	0x00FF

//! GREM field descriptions
static DebugStrings_t GREMStatusDescription = {"Status",	"", &printU32};

/*! RCMM message */
typedef struct {
	HeaderType header;
	uint16_t controlStatusValueID;
	uint16_t controlStatusContentLength;
	uint8_t controlStatus;
	uint16_t speedValueID;
	uint16_t speedContentLength;
	int16_t speed;
	uint16_t steeringValueID;
	uint16_t steeringContentLength;
	int16_t steering;
	uint16_t commandValueID;
	uint16_t commandContentLength;
	uint8_t command;
	FooterType footer;
} RCMMType;

//! RCMM value IDs
#define VALUE_ID_RCMM_CONTROL_STATUS			0x0001
#define VALUE_ID_RCMM_SPEED_METER_PER_SECOND	0x0011
#define VALUE_ID_RCMM_STEERING_ANGLE			0x0012
#define VALUE_ID_RCMM_STEERING_PERCENTAGE		0x0031
#define VALUE_ID_RCMM_SPEED_PERCENTAGE			0x0032
#define VALUE_ID_RCMM_CONTROL					0xA201

static DebugStrings_t RCMMControlStatusDescription = {"Control Status",	"",	&printU8};
static DebugStrings_t RCMMSpeedDescription_m_s = {"Speed",	"[m/s]",	&printI16};
static DebugStrings_t RCMMSpeedDescriptionPct = {"Speed",	"[%%]",	&printI16};
static DebugStrings_t RCMMSteeringDescriptionDeg = {"Steering",	"[deg]",	&printI16};
static DebugStrings_t RCMMSteeringDescriptionPct = {"Steering",	"[%%]",	&printI16};
static DebugStrings_t RCMMCommandDescription = {"Command (AstaZero)",	"",	&printU8};

/*! RDCA message - Request Direct Control Action*/
typedef struct {
	HeaderType header;
	uint16_t intendedReceiverIDValueID;
	uint16_t intendedReceiverIDContentLength;
	uint32_t intendedReceiverID;
	uint16_t gpsQmsOfWeekValueID;
	uint16_t gpsQmsOfWeekContentLength;
	uint32_t gpsQmsOfWeek;
	uint16_t steeringActionValueID;
	uint16_t steeringActionContentLength;
	int16_t steeringAction;
	uint16_t speedActionValueID;
	uint16_t speedActionContentLength;
	int16_t speedAction;
	FooterType footer;
} RDCAType;						//27 bytes

//! RDCA value IDs
#define VALUE_ID_RDCA_GPS_QMS_OF_WEEK			0x010A
#define VALUE_ID_RDCA_STEERING_ANGLE 			0x0204
#define VALUE_ID_RDCA_STEERING_PERCENTAGE		0x0205
#define VALUE_ID_RDCA_SPEED_METER_PER_SECOND	0x0206
#define VALUE_ID_RDCA_SPEED_PERCENTAGE			0x0207
#define VALUE_ID_RDCA_INTENDED_RECEIVER			0x0100

//! RDCA field descriptions
static DebugStrings_t RDCAIntendedReceiverDescription = {"Intended receiver ID", "",	&printU32};
static DebugStrings_t RDCAGpsQmsOfWeekDescription = {"GpsQmsOfWeek",	"[¼ ms]",	&printU32};
static DebugStrings_t RDCASteeringDescriptionDeg = {"Steering",		"[deg]",	&printI16};
static DebugStrings_t RDCASteeringDescriptionPct = {"Steering",		"%%",	&printI16};
static DebugStrings_t RDCASpeedDescription_m_s = {"Speed",		"[m/s]",	&printI16};
static DebugStrings_t RDCASpeedDescriptionPct = {"Speed",		"%%",	&printI16};


/*! GDRM message - General Data Request Message*/
typedef struct {
	HeaderType header;
	uint16_t DataCodeValueIdU16;
	uint16_t DataCodeContentLengthU16;
	uint16_t DataCode;
	FooterType footer;
} GDRMType;						//19 bytes

//! GDRM value IDs
#define VALUE_ID_GDRM_DATA_CODE 0x0205


//! GDRM field descriptions
static DebugStrings_t GDRMDataCodeDescription = {"Data code",	"",			&printU32};


/*! DCTI message - Direct Control Transmitter Ids*/
typedef struct {
	HeaderType header;
	uint16_t TotalCountValueIdU16;
	uint16_t TotalCountContentLengthU16;
	uint16_t TotalCount;
	uint16_t CounterValueIdU16;
	uint16_t CounterContentLengthU16;
	uint16_t Counter;
	uint16_t TransmitterIDValueIdU16;
	uint16_t TransmitterIDContentLengthU16;
	uint32_t TransmitterID;
	FooterType footer;
} DCTIType;						//33 bytes

//! DCTI value IDs
#define VALUE_ID_DCTI_TOTAL_COUNT 0x0202
#define VALUE_ID_DCTI_COUNTER 0x0203
#define VALUE_ID_DCTI_TRANSMITTER_ID 0x0010

//! DCTI field descriptions
static DebugStrings_t DCTITotalCountDescription = {"Total count",	"",			&printU32};
static DebugStrings_t DCTICounterDescription = {"Counter",	"",					&printU32};
static DebugStrings_t DCTITransmitterIdDescription = {"TransmitterID",	"",		&printU32};

#pragma pack(pop)


// ************************* Non-ISO type definitions and defines ************************************************
// Byte swapper definitions for 6 byte values and floats
#if __BYTE_ORDER == __LITTLE_ENDIAN
#define le48toh(x) (x)
#define htole48(x) (x)
#define htolef(x) (x)
#else
#define le48toh(x) (le64toh(x) >> 16)
#define htole48(x) (htole64(x) >> 16)
#define htolef_a(x) \
	htole32((union { uint32_t i; float f; }){ .f = (x) }.i)
#define htolef(x) \
  ((union { uint32_t i; float f; }){ .i = htolef_a(x) }.f)
#endif

// Time constants
// Leap seconds between UTC and GPS
#define MS_LEAP_SEC_DIFF_UTC_GPS (18000)
// Length of a week
#define WEEK_TIME_QMS 2419200000
#define WEEK_TIME_MS 604800000
// Time between 1970-01-01 and 1980-01-06
#define MS_TIME_DIFF_UTC_GPS ((uint64_t)(315964800000))

// ************************** static function declarations ********************************************************
static ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length,
											 HeaderType * HeaderData, const char debug);
static ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length,
											 FooterType * HeaderData, const char debug);
static HeaderType buildISOHeader(ISOMessageID id, uint32_t messageLength, const char debug);
static FooterType buildISOFooter(const void *message, const size_t sizeExclFooter, const char debug);
static char isValidMessageID(const uint16_t id);
static double_t mapISOHeadingToHostHeading(const double_t isoHeading_rad);
static double_t mapHostHeadingToISOHeading(const double_t hostHeading_rad);

static ISOMessageReturnValue convertHEABToHostRepresentation(
		HEABType* HEABData,
		const struct timeval *currentTime,
		const uint8_t TransmitterId,
		HeabMessageDataType* heabData);
static ISOMessageReturnValue convertGDRMToHostRepresentation(
		GDRMType* GDRMData,
		GdrmMessageDataType* gdrmData);
static ISOMessageReturnValue convertDCTIToHostRepresentation(DCTIType* DCTIData,
		DctiMessageDataType* dctiData);
static void convertMONRToHostRepresentation(
		const MONRType * MONRData,
		const struct timeval *currentTime,
		ObjectMonitorType * monitorData);
static ISOMessageReturnValue convertRDCAToHostRepresentation(
		RDCAType* RDCAData,const struct timeval* currentTime,
		RequestControlActionType* rdcaData);
static void convertOSEMToHostRepresentation(
		const OSEMType * OSEMData,
		ObjectSettingsType * ObjectSettingsData);
static ISOMessageReturnValue convertPODIToHostRepresentation(
		PODIType* PODIData,
		const struct timeval* currentTime,
		PeerObjectInjectionType* peerData);
static ISOMessageReturnValue convertOPROToHostRepresentation(
		const OPROType* OPROData,
		ObjectPropertiesType* objectProperties);
static ISOMessageReturnValue convertFOPRToHostRepresentation(
		const FOPRType* FOPRData,
		ForeignObjectPropertiesType* foreignObjectProperties);
static ISOMessageReturnValue verifyChecksum(
		const void *data,
		const size_t dataLen,
		const uint16_t crc,
		const char debug);
static ISOMessageReturnValue convertRCMMToHostRepresentation(RCMMType * RCMMData, 
		RemoteControlManoeuvreMessageType* rcmmData);
ISOMessageReturnValue convertGREMoHostRepresentation(GREMType* GREMdata,
		GeneralResponseMessageType* gremData);
static uint16_t crcByte(const uint16_t crc, const uint8_t byte);
static uint16_t crc16(const uint8_t * data, size_t dataLen);
static int encodeContent(uint16_t valueID, const void* src, char** dest, const size_t contentSize, size_t* bufferSpace, DebugStrings_t *debugStruct, const char debug);

static void printContent(const uint16_t valueID, const uint16_t contentLength, const void* value, DebugStrings_t* deb);

// Time functions
static int8_t setToGPStime(struct timeval *time, const uint16_t GPSweek, const uint32_t GPSqmsOfWeek);
static int32_t getAsGPSWeek(const struct timeval *time);
static int64_t getAsGPSQuarterMillisecondOfWeek(const struct timeval *time);
static uint64_t getAsGPSms(const struct timeval *time);

// ************************** static variables ********************************************************************
static uint16_t trajectoryMessageCrc = 0;
static int8_t isCRCVerificationEnabled = DEFAULT_CRC_CHECK_ENABLED;
static uint8_t transmitterID = 0xFF;

// ************************** function definitions ****************************************************************
void printContent(
		const uint16_t valueID,
		const uint16_t contentLength,
		const void* value,
		DebugStrings_t* deb) {
	if(deb && deb->printer) {
		printf("\t%s value ID: 0x%x\n\t%s content length: %u\n",
			   deb->name, valueID,
			   deb->name, contentLength);
		printf("\t%s: ", deb->name);
		deb->printer(value);
		printf(" %s\n", deb->unit);
	}
}


/*!
 * \brief encodeContent Copies ISO content into a buffer and manipulates the related pointers
 *			and size variables.
 * \param valueID ValueID of the data
 * \param src Pointer to data to be copied
 * \param dest Reference to pointer where data is to be stored (incremented after data copy)
 * \param dataSize Size of content to be copied
 * \param bufferSpace Remaining buffer space in data buffer pointed to by dest
 * \return 0 on success, -1 otherwise
 */
int encodeContent(uint16_t valueID,
		const void* src,
		char** dest,
		const size_t contentSize,
		size_t* bufferSpace,
		DebugStrings_t *debugStruct,
		const char debug) {
	uint16_t contentLength = (uint16_t) contentSize;

	union {
		uint8_t  u8Data;
		uint16_t u16Data;
		uint32_t u32Data;
		uint64_t u64Data;
	} data;

	if (*bufferSpace < contentSize + sizeof (valueID)
			+ sizeof (contentLength)) {
		errno = ENOBUFS;
		return -1;
	}

	if(debug && debugStruct->printer) {
		printContent(valueID, contentLength, src, debugStruct);
	}
	
	valueID = htole16(valueID);
	memcpy(*dest, &valueID, sizeof (valueID));
	*dest += sizeof (valueID);
	*bufferSpace -= sizeof (valueID);

	contentLength = htole16(contentLength);
	memcpy(*dest, &contentLength, sizeof (contentLength));
	*dest += sizeof (contentLength);
	*bufferSpace -= sizeof (contentLength);

	switch (contentSize) {
	case sizeof (uint8_t):
		memcpy(&data.u8Data, src, contentSize);
		break;
	case sizeof (uint16_t):
		memcpy(&data.u16Data, src, contentSize);
		data.u16Data = htole16(data.u16Data);
		break;
	case sizeof (uint32_t):
		memcpy(&data.u32Data, src, contentSize);
		data.u32Data = htole32(data.u32Data);
		break;
	case sizeof (uint64_t):
		memcpy(&data.u64Data, src, contentSize);
		data.u64Data = htole64(data.u64Data);
		break;
	default:
		errno = EINVAL;
		return -1;
	}

	memcpy(*dest, &data, contentSize);
	*dest += contentSize;
	*bufferSpace -= contentSize;
	return 0;
}

/*!
 * \brief setTransmitterID Sets the transmitter ID for subsequent constructed ISO messages
 * \param newTransmitterID Transmitter ID to be set
 */
void setTransmitterID(const uint8_t newTransmitterID) {
	transmitterID = newTransmitterID;
}

/*!
 * \brief getTransmitterID Get the configured transmitter ID
 * \return Transmitter ID
 */
uint8_t getTransmitterID() {
	return transmitterID;
}

/*!
 * \brief decodeISOHeader Convert data in a buffer to an ISO heade
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue decodeISOHeader(const char *MessageBuffer, const size_t length, HeaderType * HeaderData,
									  const char debug) {

	const char *p = MessageBuffer;
	ISOMessageReturnValue retval = MESSAGE_OK;
	const char ProtocolVersionBitmask = 0x7F;
	char messageProtocolVersion = 0;
	char isProtocolVersionSupported = 0;

	// If not enough data to fill header, generate error
	if (length < sizeof (HeaderData)) {
		fprintf(stderr, "Too little raw data to fill ISO header\n");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode ISO header
	memcpy(&HeaderData->SyncWordU16, p, sizeof (HeaderData->SyncWordU16));
	HeaderData->SyncWordU16 = le16toh(HeaderData->SyncWordU16);
	p += sizeof (HeaderData->SyncWordU16);

	// If sync word is not correct, generate error
	if (HeaderData->SyncWordU16 != ISO_SYNC_WORD) {
		fprintf(stderr, "Sync word error when decoding ISO header\n");
		memset(HeaderData, 0, sizeof (*HeaderData));
		return MESSAGE_SYNC_WORD_ERROR;
	}

	memcpy(&HeaderData->TransmitterIdU8, p, sizeof (HeaderData->TransmitterIdU8));
	p += sizeof (HeaderData->TransmitterIdU8);

	memcpy(&HeaderData->MessageCounterU8, p, sizeof (HeaderData->MessageCounterU8));
	p += sizeof (HeaderData->MessageCounterU8);

	memcpy(&HeaderData->AckReqProtVerU8, p, sizeof (HeaderData->AckReqProtVerU8));
	p += sizeof (HeaderData->AckReqProtVerU8);

	// Loop over permitted protocol versions to see if current version is among them
	messageProtocolVersion = HeaderData->AckReqProtVerU8 & ProtocolVersionBitmask;
	for (size_t i = 0; i < sizeof (SupportedProtocolVersions) / sizeof (SupportedProtocolVersions[0]); ++i) {
		if (SupportedProtocolVersions[i] == messageProtocolVersion) {
			isProtocolVersionSupported = 1;
			break;
		}
	}

	// Generate error if protocol version not supported
	if (!isProtocolVersionSupported) {
		fprintf(stderr, "Protocol version %u not supported\n", messageProtocolVersion);
		retval = MESSAGE_VERSION_ERROR;
		memset(HeaderData, 0, sizeof (*HeaderData));
		return retval;
	}

	memcpy(&HeaderData->MessageIdU16, p, sizeof (HeaderData->MessageIdU16));
	p += sizeof (HeaderData->MessageIdU16);
	HeaderData->MessageIdU16 = le16toh(HeaderData->MessageIdU16);

	memcpy(&HeaderData->MessageLengthU32, p, sizeof (HeaderData->MessageLengthU32));
	p += sizeof (HeaderData->MessageLengthU32);
	HeaderData->MessageLengthU32 = le32toh(HeaderData->MessageLengthU32);

	if (debug) {
		printf("SyncWordU16 = 0x%x\n", HeaderData->SyncWordU16);
		printf("TransmitterIdU8 = 0x%x\n", HeaderData->TransmitterIdU8);
		printf("MessageCounterU8 = 0x%x\n", HeaderData->MessageCounterU8);
		printf("AckReqProtVerU8 = 0x%x\n", HeaderData->AckReqProtVerU8);
		printf("MessageIdU16 = 0x%x\n", HeaderData->MessageIdU16);
		printf("MessageLengthU32 = 0x%x\n", HeaderData->MessageLengthU32);
	}

	return retval;
}

/*!
 * \brief decodeISOFooter Convert data in a buffer to an ISO footer
 * \param MessageBuffer Buffer containing raw data to be converted
 * \param length Length of buffer
 * \param HeaderData Struct in which to store resulting data
 * \param debug Flag for enabling debugging of this function
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue decodeISOFooter(const char *MessageBuffer, const size_t length, FooterType * FooterData,
									  const char debug) {

	// If too little data, generate error
	if (length < sizeof (FooterData->Crc)) {
		fprintf(stderr, "Too little raw data to fill ISO footer\n");
		memset(FooterData, 0, sizeof (*FooterData));
		return MESSAGE_LENGTH_ERROR;
	}
	memcpy(&FooterData->Crc, MessageBuffer, sizeof (FooterData->Crc));
	FooterData->Crc = le16toh(FooterData->Crc);

	if (debug) {
		printf("Decoded ISO footer:\n\tCRC: 0x%x\n", FooterData->Crc);
	}

	return MESSAGE_OK;
}

/*!
 * \brief buildISOHeader Constructs an ISO header based on the supplied message ID and content length
 * \param id Message ID of the message for which the header is to be used
 * \param messageLength Length of the message including header and footer
 * \param debug Flag for enabling debugging
 * \return A struct containing ISO header data
 */
HeaderType buildISOHeader(ISOMessageID id, uint32_t messageLength, const char debug) {
	HeaderType header;

	header.SyncWordU16 = ISO_SYNC_WORD;
	header.TransmitterIdU8 = transmitterID;
	header.MessageCounterU8 = 0;
	header.AckReqProtVerU8 = ACK_REQ | ISO_PROTOCOL_VERSION;
	if (messageLength >= sizeof (HeaderType) + sizeof (FooterType)) {
		header.MessageIdU16 = (uint16_t) id;
		header.MessageLengthU32 = messageLength - sizeof (HeaderType) - sizeof (FooterType);
	}
	else {
		fprintf(stderr, "Supplied message length too small to hold header and footer\n");
		header.MessageIdU16 = (uint16_t) MESSAGE_ID_INVALID;
		header.MessageLengthU32 = 0;
	}

	if (debug) {
		printf("Encoded ISO header:\n\tSync word: 0x%x\n\tTransmitter ID: %u\n\tMessage counter: %u\n\t"
			   "Ack request | Protocol version: 0x%x\n\tMessage ID: 0x%x\n\tMessage length: %u\n",
			   header.SyncWordU16, header.TransmitterIdU8, header.MessageCounterU8, header.AckReqProtVerU8,
			   header.MessageIdU16, header.MessageLengthU32);
	}

	// Convert from host endianness to little endian
	header.SyncWordU16 = htole16(header.SyncWordU16);
	header.MessageIdU16 = htole16(header.MessageIdU16);
	header.MessageLengthU32 = htole32(header.MessageLengthU32);

	return header;
}

/*!
 * \brief buildISOFooter Constructs a footer for an ISO message
 * \param message Pointer to start of message header
 * \param messageSize Size of the entire message including header and footer
 * \param debug Flag for enabling debugging
 * \return A struct containing ISO footer data
 */
FooterType buildISOFooter(const void *message, const size_t messageSize, const char debug) {
	FooterType footer;

	// Calculate CRC - remembering that message begins with header and messageSize will include header and footer
	footer.Crc = crc16(message, messageSize - sizeof (FooterType));

	if (debug) {
		printf("Encoded ISO footer:\n\tCRC: 0x%x\n", footer.Crc);
	}

	footer.Crc = htole16(footer.Crc);

	return footer;
}

/*!
 * \brief isValidMessageID Determines if specified id is a valid ISO message ID. The reserved range is deemed
 * invalid and vendor specific range is deemed valid.
 * \param id An ISO message id to be checked
 * \return 1 if valid, 0 if not
 */
char isValidMessageID(const uint16_t id) {
	return id == MESSAGE_ID_MONR || id == MESSAGE_ID_HEAB || id == MESSAGE_ID_TRAJ || id == MESSAGE_ID_OSEM
		|| id == MESSAGE_ID_OSTM || id == MESSAGE_ID_STRT || id == MESSAGE_ID_MONR2 || id == MESSAGE_ID_SOWM
		|| id == MESSAGE_ID_INFO || id == MESSAGE_ID_RCMM || id == MESSAGE_ID_SYPM || id == MESSAGE_ID_MTSP
		|| id == MESSAGE_ID_TRCM || id == MESSAGE_ID_ACCM || id == MESSAGE_ID_TREO || id == MESSAGE_ID_EXAC
		|| id == MESSAGE_ID_CATA || id == MESSAGE_ID_RCCM || id == MESSAGE_ID_RCRT || id == MESSAGE_ID_PIME
		|| id == MESSAGE_ID_COSE || id == MESSAGE_ID_MOMA
		|| (id >= MESSAGE_ID_VENDOR_SPECIFIC_LOWER_LIMIT && id <= MESSAGE_ID_VENDOR_SPECIFIC_UPPER_LIMIT);
}

/*!
 * \brief getISOMessageType Determines the ISO message type of a raw data buffer
 * \param messageData Buffer containing raw data to be parsed into an ISO message
 * \param length Size of buffer to be parsed
 * \param debug Flag for enabling debugging information
 * \return Value according to ::ISOMessageID
 */
ISOMessageID getISOMessageType(const char *messageData, const size_t length, const char debug) {
	HeaderType header;

	// Decode header
	if (decodeISOHeader(messageData, length, &header, debug) != MESSAGE_OK) {
		fprintf(stderr, "Unable to parse raw data into ISO message header\n");
		return MESSAGE_ID_INVALID;
	}

	// Check if header contains valid message ID, if so return it
	if (isValidMessageID(header.MessageIdU16))
		return (ISOMessageID) header.MessageIdU16;
	else {
		printf("Message ID %u does not match any known ISO message\n", header.MessageIdU16);
		return MESSAGE_ID_INVALID;
	}
}


/*!
 * \brief crcByte Updates the given CRC based on an input byte from data
 * \param crc CRC from previous byte
 * \param byte New data byte
 * \return New CRC value
 */
uint16_t crcByte(const uint16_t crc, const uint8_t byte) {
	static const uint16_t crcTable[256] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
		0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
		0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
		0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
		0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
		0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
		0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
		0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
		0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
		0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
		0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
		0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
		0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
		0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
		0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
		0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
		0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
		0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
		0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
		0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
		0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
		0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
		0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
	};

	return (uint16_t) ((crc << 8) ^ crcTable[(crc >> 8) ^ byte]);
}


/*!
 * \brief crc16 Calculates the 16 bit CCITT checksum value for the polynomial
 *				x^16 + x^12 + x^5 + 1
 * \param data Block of data for which CRC is to be calculated
 * \param dataLen Length of the block of data
 * \return CRC checksum
 */
uint16_t crc16(const uint8_t * data, size_t dataLen) {
	uint16_t crc = DEFAULT_CRC_INIT_VALUE;

	while (dataLen-- > 0) {
		crc = crcByte(crc, *data++);
	}
	return crc;
}


/*!
 * \brief verifyChecksum Generates a checksum for specified data and checks if it matches against
 *			the specified CRC. If the specified CRC is 0, the message does not contain a CRC value
 *			and the message is assumed uncorrupted.
 * \param data Data for which checksum is to be verified
 * \param dataLen Length of the data
 * \param CRC Received CRC value for the data
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue verifyChecksum(const void *data, const size_t dataLen, const uint16_t CRC,
									 const char debug) {
	if (!isCRCVerificationEnabled || CRC == 0) {
		return MESSAGE_OK;
	}

	const uint16_t dataCRC = crc16(data, dataLen);

	if (debug) {
		printf("CRC given: %u, CRC calculated: %u\n", CRC, dataCRC);
	}
	return dataCRC == CRC ? MESSAGE_OK : MESSAGE_CRC_ERROR;
}

/*!
 * \brief setISOCRCVerification Enables or disables checksum verification on received messages (default
 *			is to enable checksum verification)
 * \param enabled Boolean for enabling or disabling the checksum verification
 */
void setISOCRCVerification(const int8_t enabled) {
	isCRCVerificationEnabled = enabled ? 1 : 0;
	return;
}


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
ssize_t encodeTRAJMessageHeader(const uint16_t trajectoryID, const uint16_t trajectoryVersion,
								const char *trajectoryName, const size_t nameLength,
								const uint32_t numberOfPointsInTraj, char *trajDataBuffer,
								const size_t bufferLength, const char debug) {

	TRAJHeaderType TRAJData;
	size_t dataLen;

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
	TRAJData.header = buildISOHeader(MESSAGE_ID_TRAJ, sizeof (TRAJHeaderType)
									 + numberOfPointsInTraj * sizeof (TRAJPointType) +
									 sizeof (TRAJFooterType), debug);

	// Fill contents
	TRAJData.trajectoryIDValueID = VALUE_ID_TRAJ_TRAJECTORY_IDENTIFIER;
	TRAJData.trajectoryIDContentLength = sizeof (TRAJData.trajectoryID);
	TRAJData.trajectoryID = trajectoryID;

	TRAJData.trajectoryVersionValueID = VALUE_ID_TRAJ_TRAJECTORY_VERSION;
	TRAJData.trajectoryVersionContentLength = sizeof (TRAJData.trajectoryVersion);
	TRAJData.trajectoryVersion = trajectoryVersion;

	TRAJData.trajectoryNameValueID = VALUE_ID_TRAJ_TRAJECTORY_NAME;
	TRAJData.trajectoryNameContentLength = sizeof (TRAJData.trajectoryName);
	memset(TRAJData.trajectoryName, 0, sizeof (TRAJData.trajectoryName));
	if (trajectoryName != NULL) {
		memcpy(&TRAJData.trajectoryName, trajectoryName, nameLength);
	}

	if (debug) {
		printf("TRAJ message header:\n\t"
			   "Trajectory ID value ID: 0x%x\n\t"
			   "Trajectory ID content length: %u\n\t"
			   "Trajectory ID: %u\n\t"
			   "Trajectory name value ID: 0x%x\n\t"
			   "Trajectory name content length: %u\n\t"
			   "Trajectory name: %s\n\t"
			   "Trajectory version value ID: 0x%x\n\t"
			   "Trajectory version content length: %u\n\t"
			   "Trajectory version: %u\n", TRAJData.trajectoryIDValueID,
			   TRAJData.trajectoryIDContentLength, TRAJData.trajectoryID,
			   TRAJData.trajectoryNameValueID, TRAJData.trajectoryNameContentLength,
			   TRAJData.trajectoryName, TRAJData.trajectoryVersionValueID,
			   TRAJData.trajectoryVersionContentLength, TRAJData.trajectoryVersion);
	}

	// Switch endianness to little endian for all fields
	TRAJData.trajectoryIDValueID = htole16(TRAJData.trajectoryIDValueID);
	TRAJData.trajectoryIDContentLength = htole16(TRAJData.trajectoryIDContentLength);
	TRAJData.trajectoryID = htole16(TRAJData.trajectoryID);
	TRAJData.trajectoryVersionValueID = htole16(TRAJData.trajectoryVersionValueID);
	TRAJData.trajectoryVersionContentLength = htole16(TRAJData.trajectoryVersionContentLength);
	TRAJData.trajectoryVersion = htole16(TRAJData.trajectoryVersion);
	TRAJData.trajectoryNameValueID = htole16(TRAJData.trajectoryNameValueID);
	TRAJData.trajectoryNameContentLength = htole16(TRAJData.trajectoryNameContentLength);

	// Reset CRC
	trajectoryMessageCrc = DEFAULT_CRC_INIT_VALUE;

	memcpy(trajDataBuffer, &TRAJData, sizeof (TRAJData));

	// Update CRC
	dataLen = sizeof (TRAJData);
	while (dataLen-- > 0) {
		trajectoryMessageCrc = crcByte(trajectoryMessageCrc, (uint8_t) (*trajDataBuffer++));
	}
	return sizeof (TRAJHeaderType);
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
	OSEMData.latitude = objectSettings->coordinateSystemOrigin.isLongitudeValid ?
				(int64_t)(objectSettings->coordinateSystemOrigin.longitude_deg * LONGITUDE_ONE_DEGREE_VALUE)
			  : LONGITUDE_UNAVAILABLE_VALUE;

	OSEMData.altitudeValueID = VALUE_ID_OSEM_ALTITUDE;
	OSEMData.altitudeContentLength = sizeof (OSEMData.altitude);
	OSEMData.latitude = objectSettings->coordinateSystemOrigin.isAltitudeValid ?
				(int64_t)(objectSettings->coordinateSystemOrigin.altitude_m * ALTITUDE_ONE_METER_VALUE)
			  : ALTITUDE_UNAVAILABLE_VALUE;

	OSEMData.dateValueID = VALUE_ID_OSEM_DATE;
	OSEMData.dateContentLength = sizeof (OSEMData.date);
	OSEMData.date = objectSettings->isTimestampValid ?
				(uint32_t) ((printableTime->tm_year + 1900) * 10000 + (printableTime->tm_mon + 1) * 100 +
						   (printableTime->tm_mday))
			  : DATE_UNAVAILABLE_VALUE;

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
		*senderID = OSEMData.header.TransmitterIdU8;
	}

	// If message is not a OSEM message, generate an error
	if (OSEMData.header.MessageIdU16 != MESSAGE_ID_OSEM) {
		fprintf(stderr, "Attempted to pass non-OSEM message into OSEM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	// Decode contents
	while ((size_t) (p - osemDataBuffer) < OSEMData.header.MessageLengthU32 + sizeof (OSEMData.header)) {
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

	if ((retval = verifyChecksum(osemDataBuffer, OSEMData.header.MessageLengthU32 + sizeof (OSEMData.header),
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

/*!
 * \brief encodeOSTMMessage Constructs an ISO OSTM message based on specified command
 * \param command Command to send to object according to ::ObjectCommandType
 * \param ostmDataBuffer Data buffer to which OSTM is to be written
 * \param bufferLength Length of data buffer to which OSTM is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeOSTMMessage(const ObjectCommandType command, char *ostmDataBuffer, const size_t bufferLength,
						  const char debug) {

	OSTMType OSTMData;

	memset(ostmDataBuffer, 0, bufferLength);

	// Check so buffer can hold message
	if (bufferLength < sizeof (OSTMData)) {
		fprintf(stderr, "Buffer too small to hold necessary OSTM data\n");
		return -1;
	}

	// Check vs allowed commands
	if (!
		(command == OBJECT_COMMAND_ARM || command == OBJECT_COMMAND_DISARM
		 || command == OBJECT_COMMAND_REMOTE_CONTROL)) {
		fprintf(stderr, "OSTM does not support command %u\n", (uint8_t) command);
		return -1;
	}

	// Construct header
	OSTMData.header = buildISOHeader(MESSAGE_ID_OSTM, sizeof (OSTMData), debug);

	// Fill contents
	OSTMData.stateValueID = VALUE_ID_OSTM_STATE_CHANGE_REQUEST;
	OSTMData.stateContentLength = sizeof (OSTMData.state);
	OSTMData.state = (uint8_t) command;

	if (debug) {
		printf("OSTM message:\n\tState change request value ID: 0x%x\n\t"
			   "State change request content length: %u\n\tState change request: %u\n",
			   OSTMData.stateValueID, OSTMData.stateContentLength, OSTMData.state);
	}

	// Convert from host endianness to little endian
	OSTMData.stateValueID = htole16(OSTMData.stateValueID);
	OSTMData.stateContentLength = htole16(OSTMData.stateContentLength);

	// Construct footer
	OSTMData.footer = buildISOFooter(&OSTMData, sizeof (OSTMData), debug);

	memcpy(ostmDataBuffer, &OSTMData, sizeof (OSTMData));

	return sizeof (OSTMType);
}

/*!
 * \brief decodeOSTMMessage Decodes an ISO OSTM message.
 * \param ostmDataBuffer Buffer with data to be decoded.
 * \param bufferLength Length of OSTM data buffer.
 * \param command Decoded state change request.
 * \param debug Flag for enabling debugging.
 * \return Size of decoded message, or negative according to
 *		::ISOMessageReturnValue if an error occurred.
 */
ssize_t decodeOSTMMessage(
		const char* ostmDataBuffer,
		const size_t bufferLength,
		ObjectCommandType* command,
		const char debug) {

	OSTMType OSTMData;
	const char *p = ostmDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (ostmDataBuffer == NULL || command == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to OSTM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&OSTMData, 0, sizeof (OSTMData));
	memset(command, 0, sizeof (*command));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &OSTMData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (OSTMData.header);

	// If message is not a PODI message, generate an error
	if (OSTMData.header.MessageIdU16 != MESSAGE_ID_OSTM) {
		fprintf(stderr, "Attempted to pass non-OSTM message into OSTM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (OSTMData.header.MessageLengthU32 > sizeof (OSTMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "OSTM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - ostmDataBuffer < OSTMData.header.MessageLengthU32 + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_OSTM_STATE_CHANGE_REQUEST:
			memcpy(&OSTMData.state, p, sizeof (OSTMData.state));
			OSTMData.stateValueID = valueID;
			OSTMData.stateContentLength = contentLength;
			expectedContentLength = sizeof (OSTMData.state);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known OSTM value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - ostmDataBuffer), &OSTMData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding OSTM footer\n");
		return retval;
	}
	p += sizeof (OSTMData.footer);

	if ((retval = verifyChecksum(ostmDataBuffer, OSTMData.header.MessageLengthU32 + sizeof (HeaderType),
								 OSTMData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "OSTM checksum error\n");
		return retval;
	}

	if (debug) {
		printf("OSTM message:\n");
		printf("\tRequested state value ID: 0x%x\n", OSTMData.stateValueID);
		printf("\tRequested state content length: %u\n", OSTMData.stateContentLength);
		printf("\tRequested state: %u\n", OSTMData.state);
	}

	*command = OSTMData.state;
	return retval < 0 ? retval : p - ostmDataBuffer;
}


/*!
 * \brief encodeSTRTMessage Constructs an ISO STRT message based on start time parameters
 * \param timeOfStart Time when test shall start, a value of NULL indicates that the time is not known
 * \param strtDataBuffer Data buffer in which to place encoded STRT message
 * \param bufferLength Size of data buffer in which to place encoded STRT message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeSTRTMessage(const struct timeval *timeOfStart, char *strtDataBuffer,
						  const size_t bufferLength, const char debug) {

	STRTType STRTData;

	memset(strtDataBuffer, 0, bufferLength);

	// If buffer too small to hold STRT data, generate an error
	if (bufferLength < sizeof (STRTType)) {
		fprintf(stderr, "Buffer too small to hold necessary STRT data\n");
		return -1;
	}

	STRTData.header = buildISOHeader(MESSAGE_ID_STRT, sizeof (STRTType), debug);

	// Fill contents
	STRTData.StartTimeValueIdU16 = VALUE_ID_STRT_GPS_QMS_OF_WEEK;
	STRTData.StartTimeContentLengthU16 = sizeof (STRTData.StartTimeU32);
	int64_t startTime = getAsGPSQuarterMillisecondOfWeek(timeOfStart);

	STRTData.StartTimeU32 = timeOfStart == NULL || startTime < 0 ?
		GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : (uint32_t) startTime;
	STRTData.GPSWeekValueID = VALUE_ID_STRT_GPS_WEEK;
	STRTData.GPSWeekContentLength = sizeof (STRTData.GPSWeek);
	int32_t GPSWeek = getAsGPSWeek(timeOfStart);

	STRTData.GPSWeek = timeOfStart == NULL || GPSWeek < 0 ? GPS_WEEK_UNAVAILABLE_VALUE : (uint16_t) GPSWeek;

	if (debug) {
		printf("STRT message:\n\tGPS second of week value ID: 0x%x\n\t"
			   "GPS second of week content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
			   "GPS week value ID: 0x%x\n\tGPS week content length: %u\n\t"
			   "GPS week: %u\n", STRTData.StartTimeValueIdU16, STRTData.StartTimeContentLengthU16,
			   STRTData.StartTimeU32, STRTData.GPSWeekValueID, STRTData.GPSWeekContentLength,
			   STRTData.GPSWeek);
	}

	// Swap from host endianness to little endian
	STRTData.StartTimeValueIdU16 = htole16(STRTData.StartTimeValueIdU16);
	STRTData.StartTimeContentLengthU16 = htole16(STRTData.StartTimeContentLengthU16);
	STRTData.StartTimeU32 = htole32(STRTData.StartTimeU32);
	STRTData.GPSWeekValueID = htole16(STRTData.GPSWeekValueID);
	STRTData.GPSWeekContentLength = htole16(STRTData.GPSWeekContentLength);
	STRTData.GPSWeek = htole16(STRTData.GPSWeek);

	// Construct footer
	STRTData.footer = buildISOFooter(&STRTData, sizeof (STRTType), debug);

	memcpy(strtDataBuffer, &STRTData, sizeof (STRTType));

	return sizeof (STRTType);
}


/*!
 * \brief encodeHEABMessage Constructs an ISO HEAB message based on current control center status and system time
 * \param heabTime Timestamp to be placed in heab struct
 * \param status Current control center status according to ::ControlCenterStatusType. Entering an unaccepable value
 *	makes this parameter default to ABORT
 * \param heabDataBuffer Buffer to which HEAB message is to be written
 * \param bufferLength Size of buffer to which HEAB message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeHEABMessage(const struct timeval *heabTime, const ControlCenterStatusType status,
						  char *heabDataBuffer, const size_t bufferLength, const char debug) {

	HEABType HEABData;

	memset(heabDataBuffer, 0, bufferLength);

	// If buffer too small to hold HEAB data, generate an error
	if (bufferLength < sizeof (HEABType)) {
		fprintf(stderr, "Buffer too small to hold necessary HEAB data\n");
		return -1;
	}

	// Construct header
	HEABData.header = buildISOHeader(MESSAGE_ID_HEAB, sizeof (HEABData), debug);

	// Fill contents
	HEABData.HEABStructValueID = VALUE_ID_HEAB_STRUCT;
	HEABData.HEABStructContentLength = sizeof (HEABType) - sizeof (HeaderType) - sizeof (FooterType)
		- sizeof (HEABData.HEABStructValueID) - sizeof (HEABData.HEABStructContentLength);
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(heabTime);

	HEABData.GPSQmsOfWeek =
		GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	if (!
		(status == CONTROL_CENTER_STATUS_INIT || status == CONTROL_CENTER_STATUS_READY
		 || status == CONTROL_CENTER_STATUS_ABORT || status == CONTROL_CENTER_STATUS_RUNNING
		 || status == CONTROL_CENTER_STATUS_TEST_DONE || status == CONTROL_CENTER_STATUS_NORMAL_STOP)) {
		printf("HEAB does not support status ID %u - defaulting to ABORT\n", (uint8_t) status);
		HEABData.controlCenterStatus = (uint8_t) CONTROL_CENTER_STATUS_ABORT;
	}
	else {
		HEABData.controlCenterStatus = (uint8_t) status;
	}

	if (debug) {
		printf("HEAB message:\n\tHEAB struct value ID: 0x%x\n\t"
			   "HEAB struct content length: %u\n\tGPS second of week: %u [¼ ms]\n\t"
			   "Control center status: 0x%x\n", HEABData.HEABStructValueID, HEABData.HEABStructContentLength,
			   HEABData.GPSQmsOfWeek, HEABData.controlCenterStatus);
	}

	// Switch from host endianness to little endian
	HEABData.HEABStructValueID = htole16(HEABData.HEABStructValueID);
	HEABData.HEABStructContentLength = htole16(HEABData.HEABStructContentLength);
	HEABData.GPSQmsOfWeek = htole32(HEABData.GPSQmsOfWeek);

	HEABData.footer = buildISOFooter(&HEABData, sizeof (HEABData), debug);

	memcpy(heabDataBuffer, &HEABData, sizeof (HEABData));

	return sizeof (HEABType);

}


/*!
 * \brief decodeHEABMessage Fills HEAB data elements from a buffer of raw data
 * \param heabDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used for determining the GPS week
 * \param heabData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeHEABMessage(const char *heabDataBuffer,
										const size_t bufferLength,
										const struct timeval currentTime,
										HeabMessageDataType* heabData,
										const char debug) {

	HEABType HEABData;
	const char *p = heabDataBuffer;
	ISOMessageReturnValue retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (heabDataBuffer == NULL || heabData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to HEAB parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&HEABData, 0, sizeof (HEABData));
	memset(heabData, 0, sizeof (*heabData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &HEABData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (HEABData.header);

	// If message is not a HEAB message, generate an error
	if (HEABData.header.MessageIdU16 != MESSAGE_ID_HEAB) {
		fprintf(stderr, "Attempted to pass non-HEAB message into HEAB parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	
	if (HEABData.header.MessageLengthU32 > sizeof (HEABType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "HEAB message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}
	
	memcpy(&valueID, p, sizeof (valueID));
	p += sizeof (valueID);
	memcpy(&contentLength, p, sizeof (contentLength));
	p += sizeof (contentLength);
	valueID = le16toh(valueID);
	contentLength = le16toh(contentLength);
	HEABData.HEABStructValueID = valueID;
	HEABData.HEABStructContentLength = contentLength;

	if (contentLength != (sizeof(HEABData.GPSQmsOfWeek) + sizeof(HEABData.controlCenterStatus))) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, sizeof(HEABData.GPSQmsOfWeek) + sizeof(HEABData.controlCenterStatus));
			return MESSAGE_LENGTH_ERROR;
	}

	memcpy(&HEABData.GPSQmsOfWeek, p, sizeof (HEABData.GPSQmsOfWeek));
	HEABData.GPSQmsOfWeek = le32toh(HEABData.GPSQmsOfWeek);
	p += sizeof (HEABData.GPSQmsOfWeek);
	memcpy(&HEABData.controlCenterStatus, p, sizeof (HEABData.controlCenterStatus));
	p += sizeof (HEABData.controlCenterStatus);

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - heabDataBuffer), &HEABData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding HEAB footer\n");
		return retval;
	}
	p += sizeof (HEABData.footer);

	if (debug) {
		printf("HEAB message:\n");
		printf("\tStruct value ID: 0x%x\n", HEABData.HEABStructValueID);
		printf("\tStruct content length: %u\n", HEABData.HEABStructContentLength);
		printf("\tGPSQmsOfWeek: %u\n", HEABData.GPSQmsOfWeek);
		printf("\tControlCenterStatus: %u\n", HEABData.controlCenterStatus);
	}

	retval = convertHEABToHostRepresentation(&HEABData, &currentTime, HEABData.header.TransmitterIdU8, heabData);

	return retval < 0 ? retval : p - heabDataBuffer;
}



/*!
 * \brief convertHEABToHostRepresentation Converts a HEAB message to be used by host
 * \param HEABData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param transmitterId of the HEAB sender
 * \param heabData Output data struct, to be used by host
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue convertHEABToHostRepresentation(HEABType* HEABData,
		const struct timeval *currentTime,
		const uint8_t transmitterId,
		HeabMessageDataType* heabData) {

	
	if (HEABData == NULL || heabData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "HEAB input pointer error");
		return ISO_FUNCTION_ERROR;
	}


	heabData->transmitterID = transmitterId;
	setToGPStime(&heabData->dataTimestamp, (uint16_t) getAsGPSWeek(currentTime), HEABData->GPSQmsOfWeek);
	heabData->controlCenterStatus = HEABData->controlCenterStatus;


	return MESSAGE_OK;
}


/*!
 * \brief encodeMONRMessage Constructs an ISO MONR message based on object dynamics data from trajectory file or data generated in a simulator
 * \param objectTime Time of the object
 * \param position Position of the object in relation to test origin (includes heading/yaw)
 * \param speed Speed of the object (longitudinal and lateral)
 * \param acceleration Acceleration of the object (longitudinal and lateral)
 * \param driveDirection Drive direction with respect to the heading (backward or forward)
 * \param objectState Current State of the object (off(0),init(1),armed(2),disarmed(3),running(4),postrun(5),remoteControlled(6),aborting(7))
 * \param readyToArm Ready to arm indicator (notReady(0),readyToARM(1),unavailable(2))
 * \param objectErrorState Error status of the object (bit field encoded)
 * \param monrDataBuffer Buffer to hold the message
 * \param bufferLength Length of the buffer
 * \param debug Flag for enabling of debugging
 * \return Value according to ::ISOMessageReturnValue
 */
ssize_t encodeMONRMessage(const struct timeval *objectTime, const CartesianPosition position,
						  const SpeedType speed, const AccelerationType acceleration,
						  const unsigned char driveDirection, const unsigned char objectState,
						  const unsigned char readyToArm, const unsigned char objectErrorState,
						  char *monrDataBuffer, const size_t bufferLength, const char debug) {
	MONRType MONRData;

	memset(monrDataBuffer, 0, bufferLength);

	const uint16_t MONRStructSize = (uint16_t) (sizeof (MONRData) - sizeof (MONRData.header)
												- sizeof (MONRData.footer.Crc) -
												sizeof (MONRData.monrStructValueID)
												- sizeof (MONRData.monrStructContentLength));

	// If buffer too small to hold MONR data, generate an error
	if (bufferLength < sizeof (MONRType)) {
		fprintf(stderr, "Buffer too small to hold necessary MONR data\n");
		return -1;
	}

	// Constuct the header
	MONRData.header = buildISOHeader(MESSAGE_ID_MONR, sizeof (MONRType), debug);

	// Fill contents
	MONRData.monrStructValueID = VALUE_ID_MONR_STRUCT;
	MONRData.monrStructContentLength = MONRStructSize;

	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(objectTime);

	MONRData.gpsQmsOfWeek =
		GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	if (position.isPositionValid) {
		MONRData.xPosition = (int32_t) (position.xCoord_m * POSITION_ONE_METER_VALUE);
		MONRData.yPosition = (int32_t) (position.yCoord_m * POSITION_ONE_METER_VALUE);
		MONRData.zPosition = (int32_t) (position.zCoord_m * POSITION_ONE_METER_VALUE);
	}
	else {
		errno = EINVAL;
		fprintf(stderr, "Position is a required field in MONR messages\n");
		return -1;
	}

	if (position.isHeadingValid) {
		MONRData.heading = (uint16_t) (mapHostHeadingToISOHeading(position.heading_rad)
									   * 180.0 / M_PI * HEADING_ONE_DEGREE_VALUE);
	}
	else {
		MONRData.heading = HEADING_UNAVAILABLE_VALUE;
	}

	if (speed.isLongitudinalValid) {
		MONRData.longitudinalSpeed = (int16_t) (speed.longitudinal_m_s * SPEED_ONE_METER_PER_SECOND_VALUE);
	}
	else {
		errno = EINVAL;
		fprintf(stderr, "Longitudinal speed is a required field in MONR messages\n");
		return -1;
	}
	MONRData.lateralSpeed =
		speed.isLateralValid ? (int16_t) (speed.lateral_m_s *
										  SPEED_ONE_METER_PER_SECOND_VALUE) : SPEED_UNAVAILABLE_VALUE;
	MONRData.longitudinalAcc = acceleration.isLongitudinalValid ?
		(int16_t) (acceleration.longitudinal_m_s2 *
				   ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) : ACCELERATION_UNAVAILABLE_VALUE;
	MONRData.lateralAcc =
		acceleration.isLateralValid ? (int16_t) (acceleration.lateral_m_s2 *
												 ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE) :
		ACCELERATION_UNAVAILABLE_VALUE;

	MONRData.driveDirection = driveDirection;
	MONRData.state = objectState;
	MONRData.readyToArm = readyToArm;
	MONRData.errorStatus = objectErrorState;

	if (debug) {
		printf("MONR message:\n\tMONR struct value ID: 0x%x\n\t"
			   "MONR struct content length: %u\n\t"
			   "GPS second of week: %u [¼ ms]\n\t"
			   "X-position: %u [mm]\n\t"
			   "Y-position: %u [mm]\n\t"
			   "Z-position: %u [mm]\n\t"
			   "Heading: %u [0,01 deg]\n\t"
			   "Longitudinal Speed: %u [0,01 m/s]\n\t"
			   "Lateral Speed: %u [0,01 m/s]\n\t"
			   "Longitudinal Acceleration: %u [0,001 m/s²]\n\t"
			   "Lateral Acceleration: %u [0,001 m/s²]\n\t"
			   "Driving Direction: 0x%x\n\t"
			   "Object State: 0x%x\n\t"
			   "Ready To Arm: 0x%x\n\t"
			   "Object Error State: 0x%x\n",
			   MONRData.monrStructValueID,
			   MONRData.monrStructContentLength,
			   MONRData.gpsQmsOfWeek,
			   MONRData.xPosition,
			   MONRData.yPosition,
			   MONRData.zPosition,
			   MONRData.heading,
			   MONRData.longitudinalSpeed,
			   MONRData.lateralSpeed,
			   MONRData.longitudinalAcc,
			   MONRData.lateralAcc,
			   MONRData.driveDirection, MONRData.state, MONRData.readyToArm, MONRData.errorStatus);
	}

	// Convert from host endianness to little endian
	MONRData.monrStructValueID = htole16(MONRData.monrStructValueID);
	MONRData.monrStructContentLength = htole16(MONRData.monrStructContentLength);
	MONRData.gpsQmsOfWeek = htole32(MONRData.gpsQmsOfWeek);
	MONRData.xPosition = (int32_t) htole32(MONRData.xPosition);
	MONRData.yPosition = (int32_t) htole32(MONRData.yPosition);
	MONRData.zPosition = (int32_t) htole32(MONRData.zPosition);
	MONRData.heading = htole16(MONRData.heading);
	MONRData.longitudinalSpeed = (int16_t) htole16(MONRData.longitudinalSpeed);
	MONRData.lateralSpeed = (int16_t) htole16(MONRData.lateralSpeed);
	MONRData.longitudinalAcc = (int16_t) htole16(MONRData.longitudinalAcc);
	MONRData.lateralAcc = (int16_t) htole16(MONRData.lateralAcc);



	// Construct footer
	MONRData.footer = buildISOFooter(&MONRData, sizeof (MONRData), debug);

	// Copy struct onto the databuffer
	memcpy(monrDataBuffer, &MONRData, sizeof (MONRData));

	if (debug) {
		printf("Byte data[%lu]: ", sizeof (MONRData));
		unsigned int i;

		for (i = 0; i < sizeof (MONRData); i++) {
			if (i > 0)
				printf(":");
			printf("%02X", (unsigned char)monrDataBuffer[i]);
		}
		printf("\n");
	}

	return sizeof (MONRType);
}

/*!
 * \brief decodeRCMMessage Fills RCCM data elements from a buffer of raw data
 * \param rcmmDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param rcmmData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeRCMMMessage(
		const char *rcmmDataBuffer,
		const size_t bufferLength,
		RemoteControlManoeuvreMessageType* rcmmData,
		const char debug) {

	RCMMType RCMMData;
	const char *p = rcmmDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (rcmmDataBuffer == NULL || rcmmData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to RCMM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&RCMMData, 0, sizeof (RCMMData));
	memset(rcmmData, 0, sizeof(*rcmmData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &RCMMData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (RCMMData.header);

	// If message is not a RCMM message, generate an error
	if (RCMMData.header.MessageIdU16 != MESSAGE_ID_RCMM) {
		fprintf(stderr, "Attempted to pass non-RCMM message into RCMM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (RCMMData.header.MessageLengthU32 > sizeof (RCMMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "RCMM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - rcmmDataBuffer < RCMMData.header.MessageLengthU32 + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);
		switch (valueID) {
		case VALUE_ID_RCMM_CONTROL_STATUS:
			memcpy(&RCMMData.controlStatus, p, sizeof (RCMMData.controlStatus));
			RCMMData.controlStatusValueID = valueID;
			RCMMData.controlStatusContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.controlStatus);
			break;
		case VALUE_ID_RCMM_SPEED_METER_PER_SECOND:
			memcpy(&RCMMData.speed, p, sizeof (RCMMData.speed));
			RCMMData.speed = (int16_t)le16toh (RCMMData.speed);
			RCMMData.speedValueID = valueID;
			RCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.speed);
			break;
		case VALUE_ID_RCMM_STEERING_ANGLE:
			memcpy(&RCMMData.steering, p, sizeof (RCMMData.steering));
			RCMMData.steering = (int16_t)le16toh (RCMMData.steering);
			RCMMData.steeringValueID = valueID;
			RCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.steering);
			break;
		case VALUE_ID_RCMM_STEERING_PERCENTAGE:
			memcpy(&RCMMData.steering, p, sizeof (RCMMData.steering));
			RCMMData.steering = (int16_t)le16toh (RCMMData.steering); 
			RCMMData.steeringValueID = valueID;
			RCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.steering);
			break;
		case VALUE_ID_RCMM_SPEED_PERCENTAGE:
			memcpy(&RCMMData.speed, p, sizeof (RCMMData.speed));
			RCMMData.speed = (int16_t)le16toh (RCMMData.speed);
			RCMMData.speedValueID = valueID;
			RCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.speed);
			break;
		case VALUE_ID_RCMM_CONTROL:
			memcpy(&RCMMData.command, p, sizeof (RCMMData.command));
			RCMMData.commandValueID = valueID;
			RCMMData.commandContentLength = contentLength;
			expectedContentLength = sizeof (RCMMData.command);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known RCMM value IDs\n", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld\n",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - rcmmDataBuffer), &RCMMData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding RCMM footer\n");
		return retval;
	}

	p += sizeof (RCMMData.footer);
	if ((retval = verifyChecksum(rcmmDataBuffer, RCMMData.header.MessageLengthU32 + sizeof (HeaderType),
								 RCMMData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "RCMM checksum error\n");
		return retval;
	}

	if (debug) {
		printf("RCMM message:\n");
		printContent(RCMMData.controlStatusValueID, RCMMData.controlStatusContentLength,
					 &RCMMData.controlStatus, &RCMMControlStatusDescription);
		if (RCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_ANGLE)
			printContent(RCMMData.steeringValueID, RCMMData.steeringContentLength,
					 	&RCMMData.steering, &RCMMSteeringDescriptionDeg);
		else if (RCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_PERCENTAGE)
			printContent(RCMMData.steeringValueID, RCMMData.steeringContentLength,
					 	&RCMMData.steering, &RCMMSteeringDescriptionPct);
		if (RCMMData.speedValueID == VALUE_ID_RCMM_SPEED_METER_PER_SECOND)
			printContent(RCMMData.speedValueID, RCMMData.speedContentLength,
					 	&RCMMData.speed, &RCMMSpeedDescription_m_s);
		else if (RCMMData.speedValueID == VALUE_ID_RCMM_SPEED_PERCENTAGE)
			printContent(RCMMData.speedValueID, RCMMData.speedContentLength,
				&RCMMData.speed, &RCMMSpeedDescriptionPct);
	}

	retval = convertRCMMToHostRepresentation(&RCMMData, rcmmData);

	return retval < 0 ? retval : p - rcmmDataBuffer;
}
/**
 * @brief Convert a RCMM message to host represntation
 * 
 * @param RCMMData Data struct containing ISO formated data
 * @param rcmmData Output data struct
 * @return ISOMessageReturnValue 
 */
ISOMessageReturnValue convertRCMMToHostRepresentation(RCMMType * RCMMData, 
		RemoteControlManoeuvreMessageType* rcmmData) {
	
	if (RCMMData == NULL ||  rcmmData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "RCMM input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	// Fill steering values and check out of range values
	rcmmData->status = RCMMData->controlStatus;
	if (RCMMData->steering != STEERING_ANGLE_UNAVAILABLE_VALUE && RCMMData->steeringValueID) {
		if(RCMMData->steeringValueID == VALUE_ID_RCMM_STEERING_ANGLE) {
			if (RCMMData->steering <= STEERING_ANGLE_MAX_VALUE_DEG
			&& RCMMData->steering >= STEERING_ANGLE_MIN_VALUE_DEG) {
				rcmmData->isSteeringManoeuvreValid= 1;
				rcmmData->steeringManoeuvre.rad = RCMMData->steering / STEERING_ANGLE_ONE_DEGREE_VALUE * (M_PI / 180.0);
				rcmmData->steeringUnit = ISO_UNIT_TYPE_STEERING_DEGREES; 
			}
			else {
				fprintf(stderr, "Steering angle value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else if(RCMMData->steeringValueID == VALUE_ID_RCMM_STEERING_PERCENTAGE) {
			if (RCMMData->steering <= MAX_VALUE_PERCENTAGE
			&& RCMMData->steering >= MIN_VALUE_PERCENTAGE) {
				rcmmData->isSteeringManoeuvreValid = 1;
				rcmmData->steeringManoeuvre.pct = RCMMData->steering;
				rcmmData->steeringUnit = ISO_UNIT_TYPE_STEERING_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Steering percentage value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else {
			fprintf(stderr, "Steering Value ID error\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}

	// Fill speed values and check out of range values
	if (RCMMData->speed != SPEED_UNAVAILABLE_VALUE && RCMMData->speedValueID) {
		if(RCMMData->speedValueID == VALUE_ID_RCMM_SPEED_METER_PER_SECOND) {
				rcmmData->isSpeedManoeuvreValid = 1;
				rcmmData->speedManoeuvre.m_s = RCMMData->speed / SPEED_ONE_METER_PER_SECOND_VALUE;
				rcmmData->speedUnit = ISO_UNIT_TYPE_SPEED_METER_SECOND; 
			}
		else if(RCMMData->speedValueID == VALUE_ID_RCMM_SPEED_PERCENTAGE) {
			if (RCMMData->speed <= MAX_VALUE_PERCENTAGE 
				&& RCMMData->speed >= MIN_VALUE_PERCENTAGE) {
				rcmmData->isSpeedManoeuvreValid = 1;
				rcmmData->speedManoeuvre.pct = RCMMData->speed;
				rcmmData->speedUnit = ISO_UNIT_TYPE_SPEED_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Speed percentage value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
			
		}
		else {
			fprintf(stderr, "Receiver ID not supplied in RDCA message\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}
	return MESSAGE_OK;

}

/*!
 * \brief encodeRCMMMessage Fills an ISO RCMM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param rcmmData Struct containing relevant RCMM data
 * \param rcmmDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeRCMMMessage(const RemoteControlManoeuvreMessageType* rcmmData,
		char* rcmmDataBuffer,
		const size_t bufferLength,
		const char debug) {

	RCMMType RCMMData;

	memset(rcmmDataBuffer, 0, bufferLength);
	char* p = rcmmDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (rcmmDataBuffer == NULL) {
		fprintf(stderr, "RCMM data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold RCMM data, generate an error
	if (bufferLength < sizeof (RCMMType)) {
		fprintf(stderr, "Buffer too small to hold necessary RCMM data\n");
		return -1;
	}
	// Construct header
	RCMMData.header = buildISOHeader(MESSAGE_ID_RCMM, sizeof (RCMMData), debug);
	memcpy(p, &RCMMData.header, sizeof (RCMMData.header));
	p += sizeof (RCMMData.header);
	remainingBytes -= sizeof (RCMMData.header);

	if (debug) {
			printf("RCMM message:\n");
	}
	retval |= encodeContent(VALUE_ID_RCMM_CONTROL_STATUS, &rcmmData->status, &p,
						sizeof (rcmmData->status), &remainingBytes, &RCMMControlStatusDescription, debug);
	
	if (rcmmData->command != MANOEUVRE_NONE) {
		RCMMData.command = (uint8_t)(rcmmData->command);
		retval |= encodeContent(VALUE_ID_RCMM_CONTROL, &RCMMData.command, &p,
								sizeof (RCMMData.command), &remainingBytes, &RCMMCommandDescription, debug);
	}

	if (rcmmData->isSteeringManoeuvreValid && rcmmData->steeringUnit == ISO_UNIT_TYPE_STEERING_DEGREES) {
		if(rcmmData->steeringManoeuvre.rad <= STEERING_ANGLE_MAX_VALUE_RAD && rcmmData->steeringManoeuvre.rad >= STEERING_ANGLE_MIN_VALUE_RAD) {
			RCMMData.steering = (int16_t) (rcmmData->steeringManoeuvre.rad * (180.0 / M_PI) * STEERING_ANGLE_ONE_DEGREE_VALUE);
			retval |= encodeContent(VALUE_ID_RCMM_STEERING_ANGLE, &RCMMData.steering, &p,
							  sizeof (RCMMData.steering), &remainingBytes, &RCMMSteeringDescriptionDeg, debug);
		}
		else {
			fprintf(stderr, "Steering value is out of bounds for angle value\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	else if (rcmmData->isSteeringManoeuvreValid && rcmmData->steeringUnit == ISO_UNIT_TYPE_STEERING_PERCENTAGE) {
		if(rcmmData->steeringManoeuvre.pct <= MAX_VALUE_PERCENTAGE && rcmmData->steeringManoeuvre.pct >= MIN_VALUE_PERCENTAGE) {
			RCMMData.steering = (int16_t) (rcmmData->steeringManoeuvre.pct);
			retval |= encodeContent(VALUE_ID_RCMM_STEERING_PERCENTAGE, &RCMMData.steering, &p,
						 	  sizeof(RCMMData.steering), &remainingBytes, &RCMMSteeringDescriptionPct, debug);		
		}
		else {
			fprintf(stderr, "Steering value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}

	if(rcmmData->isSpeedManoeuvreValid && rcmmData->speedUnit == ISO_UNIT_TYPE_SPEED_METER_SECOND) {
		RCMMData.speed = (int16_t) (rcmmData->speedManoeuvre.m_s *SPEED_ONE_METER_PER_SECOND_VALUE);
		retval |= encodeContent(VALUE_ID_RCMM_SPEED_METER_PER_SECOND, &RCMMData.speed, &p,
								sizeof(RCMMData.speed), &remainingBytes, &RCMMSpeedDescription_m_s, debug);
	}
	else if(rcmmData->isSpeedManoeuvreValid && rcmmData->speedUnit == ISO_UNIT_TYPE_SPEED_PERCENTAGE) {
		if (rcmmData->speedManoeuvre.pct <= MAX_VALUE_PERCENTAGE && rcmmData->speedManoeuvre.pct >= MIN_VALUE_PERCENTAGE) {
			RCMMData.speed = (int16_t) (rcmmData->speedManoeuvre.pct);
			retval |= encodeContent(VALUE_ID_RCMM_SPEED_PERCENTAGE, &RCMMData.speed, &p,
			sizeof(RCMMData.speed), &remainingBytes, &RCMMSpeedDescriptionPct, debug);
		}
		else {
			fprintf(stderr, "Speed value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary RCMM data\n");
		return -1;
	}
	
	// Construct footer
	RCMMData.footer = buildISOFooter(&RCMMData, (size_t) (p-rcmmDataBuffer)+ sizeof(RCMMData.footer), debug);
	
	memcpy(p, &RCMMData.footer, sizeof (RCMMData.footer));
	p += sizeof (RCMMData.footer);
	remainingBytes -= sizeof (RCMMData.footer);

	if(debug)
	{
		printf("RCMM message data (size = %lu):\n", sizeof (RCMMType));
		for(size_t i = 0; i < sizeof (RCMMType); i++) printf("%x ", *(rcmmDataBuffer+i));
		printf("\n");
	}

	return p - rcmmDataBuffer;
}


/*!
 * \brief decodeMONRMessage Fills a monitor data struct from a buffer of raw data
 * \param monrDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used to guess GPS week of MONR message
 * \param objectID Optional output ID of object sending the parsed MONR data
 * \param monitorData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return Number of bytes decoded, or negative value according to ::ISOMessageReturnValue
 */
ssize_t decodeMONRMessage(const char *monrDataBuffer,
										const size_t bufferLength,
										const struct timeval currentTime,
										uint32_t * objectID,
										ObjectMonitorType * monitorData, const char debug) {

	MONRType MONRData;
	const char *p = monrDataBuffer;
	const uint16_t ExpectedMONRStructSize = (uint16_t) (sizeof (MONRData) - sizeof (MONRData.header)
														- sizeof (MONRData.footer.Crc) -
														sizeof (MONRData.monrStructValueID)
														- sizeof (MONRData.monrStructContentLength));
	ssize_t retval = MESSAGE_OK;

	if (monitorData == NULL || monrDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to MONR parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(monitorData, 0, sizeof (*monitorData));
	*objectID = 0;

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &MONRData.header, debug)) != MESSAGE_OK) {
		memset(monitorData, 0, sizeof (*monitorData));
		return retval;
	}
	p += sizeof (MONRData.header);
	*objectID = MONRData.header.TransmitterIdU8;

	// If message is not a MONR message, generate an error
	if (MONRData.header.MessageIdU16 != MESSAGE_ID_MONR) {
		fprintf(stderr, "Attempted to pass non-MONR message into MONR parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	// Decode content header
	memcpy(&MONRData.monrStructValueID, p, sizeof (MONRData.monrStructValueID));
	p += sizeof (MONRData.monrStructValueID);
	MONRData.monrStructValueID = le16toh(MONRData.monrStructValueID);

	// If content is not a MONR struct or an unexpected size, generate an error
	if (MONRData.monrStructValueID != VALUE_ID_MONR_STRUCT) {
		fprintf(stderr, "Attempted to pass non-MONR struct into MONR parsing function\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	memcpy(&MONRData.monrStructContentLength, p, sizeof (MONRData.monrStructContentLength));
	p += sizeof (MONRData.monrStructContentLength);
	MONRData.monrStructContentLength = le16toh(MONRData.monrStructContentLength);

	if (MONRData.monrStructContentLength != ExpectedMONRStructSize) {
		fprintf(stderr, "MONR content length %u differs from the expected length %u\n",
				MONRData.monrStructContentLength, ExpectedMONRStructSize);
		return MESSAGE_LENGTH_ERROR;
	}

	// Decode content
	memcpy(&MONRData.gpsQmsOfWeek, p, sizeof (MONRData.gpsQmsOfWeek));
	p += sizeof (MONRData.gpsQmsOfWeek);
	MONRData.gpsQmsOfWeek = le32toh(MONRData.gpsQmsOfWeek);

	memcpy(&MONRData.xPosition, p, sizeof (MONRData.xPosition));
	p += sizeof (MONRData.xPosition);
	MONRData.xPosition = (int32_t) le32toh(MONRData.xPosition);

	memcpy(&MONRData.yPosition, p, sizeof (MONRData.yPosition));
	p += sizeof (MONRData.yPosition);
	MONRData.yPosition = (int32_t) le32toh(MONRData.yPosition);

	memcpy(&MONRData.zPosition, p, sizeof (MONRData.zPosition));
	p += sizeof (MONRData.zPosition);
	MONRData.zPosition = (int32_t) le32toh(MONRData.zPosition);

	memcpy(&MONRData.heading, p, sizeof (MONRData.heading));
	p += sizeof (MONRData.heading);
	MONRData.heading = le16toh(MONRData.heading);

	memcpy(&MONRData.longitudinalSpeed, p, sizeof (MONRData.longitudinalSpeed));
	p += sizeof (MONRData.longitudinalSpeed);
	MONRData.longitudinalSpeed = (int16_t) le16toh(MONRData.longitudinalSpeed);

	memcpy(&MONRData.lateralSpeed, p, sizeof (MONRData.lateralSpeed));
	p += sizeof (MONRData.lateralSpeed);
	MONRData.lateralSpeed = (int16_t) le16toh(MONRData.lateralSpeed);

	memcpy(&MONRData.longitudinalAcc, p, sizeof (MONRData.longitudinalAcc));
	p += sizeof (MONRData.longitudinalAcc);
	MONRData.longitudinalAcc = (int16_t) le16toh(MONRData.longitudinalAcc);

	memcpy(&MONRData.lateralAcc, p, sizeof (MONRData.lateralAcc));
	p += sizeof (MONRData.lateralAcc);
	MONRData.lateralAcc = (int16_t) le16toh(MONRData.lateralAcc);

	memcpy(&MONRData.driveDirection, p, sizeof (MONRData.driveDirection));
	p += sizeof (MONRData.driveDirection);

	memcpy(&MONRData.state, p, sizeof (MONRData.state));
	p += sizeof (MONRData.state);

	memcpy(&MONRData.readyToArm, p, sizeof (MONRData.readyToArm));
	p += sizeof (MONRData.readyToArm);

	memcpy(&MONRData.errorStatus, p, sizeof (MONRData.errorStatus));
	p += sizeof (MONRData.errorStatus);

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - monrDataBuffer), &MONRData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding MONR footer\n");
		return retval;
	}
	p += sizeof (MONRData.footer);

	if ((retval = verifyChecksum(&MONRData, sizeof (MONRData) - sizeof (MONRData.footer),
								 MONRData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "MONR checksum error\n");
		return retval;
	}

	if (debug) {
		printf("MONR:\n");
		printf("SyncWord = %x\n", MONRData.header.SyncWordU16);
		printf("TransmitterId = %d\n", MONRData.header.TransmitterIdU8);
		printf("PackageCounter = %d\n", MONRData.header.MessageCounterU8);
		printf("AckReq = %d\n", MONRData.header.AckReqProtVerU8);
		printf("MessageId = %d\n", MONRData.header.MessageIdU16);
		printf("MessageLength = %d\n", MONRData.header.MessageLengthU32);
		printf("ValueId = %d\n", MONRData.monrStructValueID);
		printf("ContentLength = %d\n", MONRData.monrStructContentLength);
		printf("GPSSOW = %d\n", MONRData.gpsQmsOfWeek);
		printf("XPosition = %d\n", MONRData.xPosition);
		printf("YPosition = %d\n", MONRData.yPosition);
		printf("ZPosition = %d\n", MONRData.zPosition);
		printf("Heading = %d\n", MONRData.heading);
		printf("LongitudinalSpeed = %d\n", MONRData.longitudinalSpeed);
		printf("LateralSpeed = %d\n", MONRData.lateralSpeed);
		printf("LongitudinalAcc = %d\n", MONRData.longitudinalAcc);
		printf("LateralAcc = %d\n", MONRData.lateralAcc);
		printf("DriveDirection = %d\n", MONRData.driveDirection);
		printf("State = %d\n", MONRData.state);
		printf("ReadyToArm = %d\n", MONRData.readyToArm);
		printf("ErrorStatus = %d\n", MONRData.errorStatus);
	}

	// Fill output struct with parsed data
	convertMONRToHostRepresentation(&MONRData, &currentTime, monitorData);

	return retval < 0 ? retval : p - monrDataBuffer;
}


/*!
 * \brief convertMONRToHostRepresentation Converts a MONR message to the internal representation for
 * object monitoring data
 * \param MONRData MONR message to be converted
 * \param currentTime Current system time, used to guess GPS week of MONR message
 * \param monitorData Monitor data in which result is to be placed
 */
void convertMONRToHostRepresentation(const MONRType * MONRData,
									 const struct timeval *currentTime, ObjectMonitorType * monitorData) {

	// Timestamp
	monitorData->isTimestampValid = MONRData->gpsQmsOfWeek != GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	if (monitorData->isTimestampValid) {
		int32_t GPSWeek = getAsGPSWeek(currentTime);

		if (GPSWeek < 0) {
			monitorData->isTimestampValid = false;
		}
		else {
			monitorData->isTimestampValid = setToGPStime(&monitorData->timestamp,
														 (uint16_t) GPSWeek, MONRData->gpsQmsOfWeek) >= 0;
		}
	}

	// Position / heading
	monitorData->position.xCoord_m = (double)(MONRData->xPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.yCoord_m = (double)(MONRData->yPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.zCoord_m = (double)(MONRData->zPosition) / POSITION_ONE_METER_VALUE;
	monitorData->position.isPositionValid = true;
	monitorData->position.isHeadingValid = MONRData->heading != HEADING_UNAVAILABLE_VALUE;
	if (monitorData->position.isHeadingValid) {
		monitorData->position.heading_rad =
			mapISOHeadingToHostHeading(MONRData->heading / 100.0 * M_PI / 180.0);
	}

	// Velocity
	monitorData->speed.isLongitudinalValid = MONRData->longitudinalSpeed != SPEED_UNAVAILABLE_VALUE;
	monitorData->speed.longitudinal_m_s = monitorData->speed.isLongitudinalValid ?
		(double)(MONRData->longitudinalSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE : 0;
	monitorData->speed.isLateralValid = MONRData->lateralSpeed != SPEED_UNAVAILABLE_VALUE;
	monitorData->speed.lateral_m_s = monitorData->speed.isLateralValid ?
		(double)(MONRData->lateralSpeed) / SPEED_ONE_METER_PER_SECOND_VALUE : 0;

	// Acceleration
	monitorData->acceleration.isLongitudinalValid =
		MONRData->longitudinalAcc != ACCELERATION_UNAVAILABLE_VALUE;
	monitorData->acceleration.longitudinal_m_s2 =
		monitorData->acceleration.isLongitudinalValid ? (double)(MONRData->longitudinalAcc) /
		ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE : 0;
	monitorData->acceleration.isLateralValid = MONRData->lateralAcc != ACCELERATION_UNAVAILABLE_VALUE;
	monitorData->acceleration.lateral_m_s2 = monitorData->acceleration.isLateralValid ?
		(double)(MONRData->lateralAcc) / ACCELERATION_ONE_METER_PER_SECOND_SQUARED_VALUE : 0;

	// Drive direction
	switch (MONRData->driveDirection) {
	case ISO_DRIVE_DIRECTION_FORWARD:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_FORWARD;
		break;
	case ISO_DRIVE_DIRECTION_BACKWARD:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_BACKWARD;
		break;
	case ISO_DRIVE_DIRECTION_UNAVAILABLE:
	default:
		monitorData->drivingDirection = OBJECT_DRIVE_DIRECTION_UNAVAILABLE;
	}

	// State
	switch (MONRData->state) {
	case ISO_OBJECT_STATE_INIT:
		monitorData->state = OBJECT_STATE_INIT;
		break;
	case ISO_OBJECT_STATE_DISARMED:
		monitorData->state = OBJECT_STATE_DISARMED;
		break;
	case ISO_OBJECT_STATE_ARMED:
		monitorData->state = OBJECT_STATE_ARMED;
		break;
	case ISO_OBJECT_STATE_RUNNING:
		monitorData->state = OBJECT_STATE_RUNNING;
		break;
	case ISO_OBJECT_STATE_POSTRUN:
		monitorData->state = OBJECT_STATE_POSTRUN;
		break;
	case ISO_OBJECT_STATE_ABORTING:
		monitorData->state = OBJECT_STATE_ABORTING;
		break;
	case ISO_OBJECT_STATE_REMOTE_CONTROLLED:
		monitorData->state = OBJECT_STATE_REMOTE_CONTROL;
		break;
	case ISO_OBJECT_STATE_OFF:
	default:
		monitorData->state = OBJECT_STATE_UNKNOWN;
		break;
	}

	// Ready to arm
	switch (MONRData->readyToArm) {
	case ISO_READY_TO_ARM:
		monitorData->armReadiness = OBJECT_READY_TO_ARM;
		break;
	case ISO_NOT_READY_TO_ARM:
		monitorData->armReadiness = OBJECT_NOT_READY_TO_ARM;
		break;
	case ISO_READY_TO_ARM_UNAVAILABLE:
	default:
		monitorData->armReadiness = OBJECT_READY_TO_ARM_UNAVAILABLE;
	}

	// Error status
	monitorData->error.engineFault = MONRData->errorStatus & BITMASK_ERROR_ENGINE_FAULT;
	monitorData->error.abortRequest = MONRData->errorStatus & BITMASK_ERROR_ABORT_REQUEST;
	monitorData->error.batteryFault = MONRData->errorStatus & BITMASK_ERROR_BATTERY_FAULT;
	monitorData->error.unknownError = MONRData->errorStatus & BITMASK_ERROR_OTHER
		|| MONRData->errorStatus & BITMASK_ERROR_VENDOR_SPECIFIC;
	monitorData->error.syncPointEnded = MONRData->errorStatus & BITMASK_ERROR_SYNC_POINT_ENDED;
	monitorData->error.outsideGeofence = MONRData->errorStatus & BITMASK_ERROR_OUTSIDE_GEOFENCE;
	monitorData->error.badPositioningAccuracy =
		MONRData->errorStatus & BITMASK_ERROR_BAD_POSITIONING_ACCURACY;

	return;
}


/*!
 * \brief encodeSYPMMessage Fills an ISO SYPM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param synchronizationTime Time along trajectory at which objects are to be synchronized
 * \param freezeTime Time along trajectory after which no further adaptation to the master is allowed
 * \param mtspDataBuffer Buffer to which SYPM message is to be written
 * \param bufferLength Size of buffer to which SYPM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of an error
 */
ssize_t encodeSYPMMessage(const struct timeval synchronizationTime, const struct timeval freezeTime,
						  char *sypmDataBuffer, const size_t bufferLength, const char debug) {

	SYPMType SYPMData;

	// If buffer too small to hold SYPM data, generate an error
	if (bufferLength < sizeof (SYPMType)) {
		fprintf(stderr, "Buffer too small to hold necessary SYPM data\n");
		return -1;
	}

	// Construct header
	SYPMData.header = buildISOHeader(MESSAGE_ID_SYPM, sizeof (SYPMData), debug);

	// Fill contents
	SYPMData.syncPointTimeValueID = VALUE_ID_SYPM_SYNC_POINT_TIME;
	SYPMData.syncPointTimeContentLength = sizeof (SYPMData.syncPointTime);
	SYPMData.syncPointTime =
		(uint32_t) (synchronizationTime.tv_sec * 1000 + synchronizationTime.tv_usec / 1000);

	SYPMData.freezeTimeValueID = VALUE_ID_SYPM_FREEZE_TIME;
	SYPMData.freezeTimeContentLength = sizeof (SYPMData.freezeTime);
	SYPMData.freezeTime = (uint32_t) (freezeTime.tv_sec * 1000 + freezeTime.tv_usec / 1000);

	if (debug) {
		printf("SYPM message:\n\tSynchronization point time value ID: 0x%x\n\t"
			   "Synchronization point time content length: %u\n\t"
			   "Synchronization point time: %u [ms]\n\t"
			   "Freeze time value ID: 0x%x\n\t"
			   "Freeze time content length: %u\n\t"
			   "Freeze time: %u [ms]\n", SYPMData.syncPointTimeValueID,
			   SYPMData.syncPointTimeContentLength, SYPMData.syncPointTime,
			   SYPMData.freezeTimeValueID, SYPMData.freezeTimeContentLength, SYPMData.freezeTime);
	}

	// Switch from host endianness to little endian
	SYPMData.syncPointTimeValueID = htole16(SYPMData.syncPointTimeValueID);
	SYPMData.syncPointTimeContentLength = htole16(SYPMData.syncPointTimeContentLength);
	SYPMData.syncPointTime = htole16(SYPMData.syncPointTime);
	SYPMData.freezeTimeValueID = htole16(SYPMData.freezeTimeValueID);
	SYPMData.freezeTimeContentLength = htole16(SYPMData.freezeTimeContentLength);
	SYPMData.freezeTime = htobe16(SYPMData.freezeTime);

	// Construct footer
	SYPMData.footer = buildISOFooter(&SYPMData, sizeof (SYPMData), debug);

	memcpy(sypmDataBuffer, &SYPMData, sizeof (SYPMData));

	return sizeof (SYPMType);
}

/*!
 * \brief encodeMTSPMessage Fills an ISO MTSP struct with relevant data fields, and corresponding value IDs and content lengths
 * \param estSyncPointTime Estimated time when the master object will reach the synchronization point
 * \param mtspDataBuffer Buffer to which MTSP message is to be written
 * \param bufferLength Size of buffer to which MTSP message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of an error
 */
ssize_t encodeMTSPMessage(const struct timeval *estSyncPointTime, char *mtspDataBuffer,
						  const size_t bufferLength, const char debug) {

	MTSPType MTSPData;

	memset(mtspDataBuffer, 0, bufferLength);

	// If buffer too small to hold MTSP data, generate an error
	if (bufferLength < sizeof (MTSPType)) {
		fprintf(stderr, "Buffer too small to hold necessary MTSP data\n");
		return -1;
	}

	// Construct header
	MTSPData.header = buildISOHeader(MESSAGE_ID_MTSP, sizeof (MTSPData), debug);

	// Fill contents
	MTSPData.estSyncPointTimeValueID = VALUE_ID_MTSP_EST_SYNC_POINT_TIME;
	MTSPData.estSyncPointTimeContentLength = sizeof (MTSPData.estSyncPointTime);
	int64_t syncPtTime = getAsGPSQuarterMillisecondOfWeek(estSyncPointTime);

	MTSPData.estSyncPointTime = estSyncPointTime == NULL || syncPtTime < 0 ?
		GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : (uint32_t) syncPtTime;

	if (debug) {
		printf("MTSP message:\n\t"
			   "Estimated sync point time value ID: 0x%x\n\t"
			   "Estimated sync point time content length: %u\n\t"
			   "Estimated sync point time: %u [¼ ms]\n",
			   MTSPData.estSyncPointTimeValueID, MTSPData.estSyncPointTimeContentLength,
			   MTSPData.estSyncPointTime);
	}

	// Switch from host endianness to little endian
	MTSPData.estSyncPointTimeValueID = htole16(MTSPData.estSyncPointTimeValueID);
	MTSPData.estSyncPointTimeContentLength = htole16(MTSPData.estSyncPointTimeContentLength);
	MTSPData.estSyncPointTime = htole32(MTSPData.estSyncPointTime);

	// Construct footer
	MTSPData.footer = buildISOFooter(&MTSPData, sizeof (MTSPData), debug);

	memcpy(mtspDataBuffer, &MTSPData, sizeof (MTSPData));

	return sizeof (MTSPType);
}


/*!
 * \brief encodeTRCMMessage Fills an ISO TRCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param triggerID ID of the trigger to be configured
 * \param triggerType Type of the trigger to be configured according to ::TriggerType_t
 * \param param1 First parameter of the trigger to be configured according to ::TriggerTypeParameter_t
 * \param param2 Second parameter of the trigger to be configured ::TriggerTypeParameter_t
 * \param param3 Third parameter of the trigger to be configured ::TriggerTypeParameter_t
 * \param trcmDataBuffer Buffer to which TRCM message is to be written
 * \param bufferLength Size of buffer to which TRCM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeTRCMMessage(const uint16_t * triggerID, const TriggerType_t * triggerType,
						  const TriggerTypeParameter_t * param1, const TriggerTypeParameter_t * param2,
						  const TriggerTypeParameter_t * param3, char *trcmDataBuffer,
						  const size_t bufferLength, const char debug) {

	TRCMType TRCMData;

	memset(trcmDataBuffer, 0, bufferLength);

	// If buffer too small to hold TRCM data, generate an error
	if (bufferLength < sizeof (TRCMType)) {
		fprintf(stderr, "Buffer too small to hold necessary TRCM data\n");
		return -1;
	}

	// Construct header
	TRCMData.header = buildISOHeader(MESSAGE_ID_TRCM, sizeof (TRCMData), debug);

	// Fill contents
	TRCMData.triggerIDValueID = VALUE_ID_TRCM_TRIGGER_ID;
	TRCMData.triggerIDContentLength = sizeof (TRCMData.triggerID);
	TRCMData.triggerID = triggerID == NULL ? TRIGGER_ID_UNAVAILABLE : *triggerID;

	TRCMData.triggerTypeValueID = VALUE_ID_TRCM_TRIGGER_TYPE;
	TRCMData.triggerTypeContentLength = sizeof (TRCMData.triggerType);
	TRCMData.triggerType = triggerType == NULL ? TRIGGER_TYPE_UNAVAILABLE : (uint16_t) (*triggerType);

	TRCMData.triggerTypeParameter1ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM1;
	TRCMData.triggerTypeParameter2ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM2;
	TRCMData.triggerTypeParameter3ValueID = VALUE_ID_TRCM_TRIGGER_TYPE_PARAM3;

	TRCMData.triggerTypeParameter1ContentLength = sizeof (TRCMData.triggerTypeParameter1);
	TRCMData.triggerTypeParameter2ContentLength = sizeof (TRCMData.triggerTypeParameter2);
	TRCMData.triggerTypeParameter3ContentLength = sizeof (TRCMData.triggerTypeParameter3);

	TRCMData.triggerTypeParameter1 =
		param1 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param1);
	TRCMData.triggerTypeParameter2 =
		param2 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param2);
	TRCMData.triggerTypeParameter3 =
		param3 == NULL ? TRIGGER_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param3);

	if (debug) {
		printf("TRCM message:\n\tTrigger ID value ID: 0x%x\n\tTrigger ID content length: %u\n\t"
			   "Trigger ID: %u\n\tTrigger type value ID: 0x%x\n\tTrigger type content length: %u\n\t"
			   "Trigger type: %u\n\tTrigger type parameter 1 value ID: 0x%x\n\tTrigger type parameter 1 content length: %u\n\t"
			   "Trigger type parameter 1: %u\n\tTrigger type parameter 2 value ID: 0x%x\n\tTrigger type parameter 2 content length: %u\n\t"
			   "Trigger type parameter 2: %u\n\tTrigger type parameter 3 value ID: 0x%x\n\tTrigger type parameter 3 content length: %u"
			   "Trigger type parameter 3: %u\n\t", TRCMData.triggerIDValueID,
			   TRCMData.triggerIDContentLength, TRCMData.triggerID, TRCMData.triggerTypeValueID,
			   TRCMData.triggerTypeContentLength, TRCMData.triggerType,
			   TRCMData.triggerTypeParameter1ValueID, TRCMData.triggerTypeParameter1ContentLength,
			   TRCMData.triggerTypeParameter1, TRCMData.triggerTypeParameter2ValueID,
			   TRCMData.triggerTypeParameter2ContentLength, TRCMData.triggerTypeParameter2,
			   TRCMData.triggerTypeParameter3ValueID, TRCMData.triggerTypeParameter3ContentLength,
			   TRCMData.triggerTypeParameter3);
	}

	// Switch from host endianness to little endian
	TRCMData.triggerIDValueID = htole16(TRCMData.triggerIDValueID);
	TRCMData.triggerIDContentLength = htole16(TRCMData.triggerIDContentLength);
	TRCMData.triggerID = htole16(TRCMData.triggerID);

	TRCMData.triggerTypeValueID = htole16(TRCMData.triggerTypeValueID);
	TRCMData.triggerTypeContentLength = htole16(TRCMData.triggerTypeContentLength);
	TRCMData.triggerType = htole16(TRCMData.triggerType);

	TRCMData.triggerTypeParameter1ValueID = htole16(TRCMData.triggerTypeParameter1ValueID);
	TRCMData.triggerTypeParameter2ValueID = htole16(TRCMData.triggerTypeParameter2ValueID);
	TRCMData.triggerTypeParameter3ValueID = htole16(TRCMData.triggerTypeParameter3ValueID);

	TRCMData.triggerTypeParameter1ContentLength = htole16(TRCMData.triggerTypeParameter1ContentLength);
	TRCMData.triggerTypeParameter2ContentLength = htole16(TRCMData.triggerTypeParameter2ContentLength);
	TRCMData.triggerTypeParameter3ContentLength = htole16(TRCMData.triggerTypeParameter3ContentLength);

	TRCMData.triggerTypeParameter1 = htole32(TRCMData.triggerTypeParameter1);
	TRCMData.triggerTypeParameter2 = htole32(TRCMData.triggerTypeParameter2);
	TRCMData.triggerTypeParameter3 = htole32(TRCMData.triggerTypeParameter3);

	// Construct footer
	TRCMData.footer = buildISOFooter(&TRCMData, sizeof (TRCMData), debug);

	memcpy(trcmDataBuffer, &TRCMData, sizeof (TRCMData));

	return sizeof (TRCMData);
}





/*!
 * \brief encodeACCMMessage Fills an ISO ACCM struct with relevant data fields, and corresponding value IDs and content lengths
 * \param actionID ID of the action to be configured
 * \param actionType Type of the action to be configured according to ::ActionType_t
 * \param param1 First parameter of the action to be configured according to ::ActionTypeParameter_t
 * \param param2 Second parameter of the action to be configured ::ActionTypeParameter_t
 * \param param3 Third parameter of the action to be configured ::ActionTypeParameter_t
 * \param trcmDataBuffer Buffer to which ACCM message is to be written
 * \param bufferLength Size of buffer to which ACCM message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeACCMMessage(const uint16_t * actionID, const ActionType_t * actionType,
						  const ActionTypeParameter_t * param1, const ActionTypeParameter_t * param2,
						  const ActionTypeParameter_t * param3, char *accmDataBuffer,
						  const size_t bufferLength, const char debug) {

	ACCMType ACCMData;

	memset(accmDataBuffer, 0, bufferLength);

	// If buffer too small to hold ACCM data, generate an error
	if (bufferLength < sizeof (ACCMType)) {
		fprintf(stderr, "Buffer too small to hold necessary ACCM data\n");
		return -1;
	}

	// Construct header
	ACCMData.header = buildISOHeader(MESSAGE_ID_ACCM, sizeof (ACCMData), debug);

	// Fill contents
	ACCMData.actionIDValueID = VALUE_ID_ACCM_ACTION_ID;
	ACCMData.actionIDContentLength = sizeof (ACCMData.actionID);
	ACCMData.actionID = actionID == NULL ? ACTION_ID_UNAVAILABLE : *actionID;

	ACCMData.actionTypeValueID = VALUE_ID_ACCM_ACTION_TYPE;
	ACCMData.actionTypeContentLength = sizeof (ACCMData.actionType);
	ACCMData.actionType = actionType == NULL ? ACTION_TYPE_UNAVAILABLE : (uint16_t) (*actionType);

	ACCMData.actionTypeParameter1ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM1;
	ACCMData.actionTypeParameter2ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM2;
	ACCMData.actionTypeParameter3ValueID = VALUE_ID_ACCM_ACTION_TYPE_PARAM3;

	ACCMData.actionTypeParameter1ContentLength = sizeof (ACCMData.actionTypeParameter1);
	ACCMData.actionTypeParameter2ContentLength = sizeof (ACCMData.actionTypeParameter2);
	ACCMData.actionTypeParameter3ContentLength = sizeof (ACCMData.actionTypeParameter3);

	ACCMData.actionTypeParameter1 = param1 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param1);
	ACCMData.actionTypeParameter2 = param2 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param2);
	ACCMData.actionTypeParameter3 = param3 == NULL ? ACTION_TYPE_PARAMETER_UNAVAILABLE : (uint32_t) (*param3);

	if (debug) {
		printf("ACCM message:\n\tAction ID value ID: 0x%x\n\tAction ID content length: %u\n\t"
			   "Action ID: %u\n\tAction type value ID: 0x%x\n\tAction type content length: %u\n\t"
			   "Action type: %u\n\tAction type parameter 1 value ID: 0x%x\n\tAction type parameter 1 content length: %u\n\t"
			   "Action type parameter 1: %u\n\tAction type parameter 2 value ID: 0x%x\n\tAction type parameter 2 content length: %u\n\t"
			   "Action type parameter 2: %u\n\tAction type parameter 3 value ID: 0x%x\n\tAction type parameter 3 content length: %u"
			   "Action type parameter 3: %u\n\t", ACCMData.actionIDValueID, ACCMData.actionIDContentLength,
			   ACCMData.actionID, ACCMData.actionTypeValueID, ACCMData.actionTypeContentLength,
			   ACCMData.actionType, ACCMData.actionTypeParameter1ValueID,
			   ACCMData.actionTypeParameter1ContentLength, ACCMData.actionTypeParameter1,
			   ACCMData.actionTypeParameter2ValueID, ACCMData.actionTypeParameter2ContentLength,
			   ACCMData.actionTypeParameter2, ACCMData.actionTypeParameter3ValueID,
			   ACCMData.actionTypeParameter3ContentLength, ACCMData.actionTypeParameter3);
	}

	// Switch from host endianness to little endian
	ACCMData.actionIDValueID = htole16(ACCMData.actionIDValueID);
	ACCMData.actionIDContentLength = htole16(ACCMData.actionIDContentLength);
	ACCMData.actionID = htole16(ACCMData.actionID);

	ACCMData.actionTypeValueID = htole16(ACCMData.actionTypeValueID);
	ACCMData.actionTypeContentLength = htole16(ACCMData.actionTypeContentLength);
	ACCMData.actionType = htole16(ACCMData.actionType);

	ACCMData.actionTypeParameter1ValueID = htole16(ACCMData.actionTypeParameter1ValueID);
	ACCMData.actionTypeParameter2ValueID = htole16(ACCMData.actionTypeParameter2ValueID);
	ACCMData.actionTypeParameter3ValueID = htole16(ACCMData.actionTypeParameter3ValueID);

	ACCMData.actionTypeParameter1ContentLength = htole16(ACCMData.actionTypeParameter1ContentLength);
	ACCMData.actionTypeParameter2ContentLength = htole16(ACCMData.actionTypeParameter2ContentLength);
	ACCMData.actionTypeParameter3ContentLength = htole16(ACCMData.actionTypeParameter3ContentLength);

	ACCMData.actionTypeParameter1 = htole32(ACCMData.actionTypeParameter1);
	ACCMData.actionTypeParameter2 = htole32(ACCMData.actionTypeParameter2);
	ACCMData.actionTypeParameter3 = htole32(ACCMData.actionTypeParameter3);

	// Construct footer
	ACCMData.footer = buildISOFooter(&ACCMData, sizeof (ACCMData), debug);

	memcpy(accmDataBuffer, &ACCMData, sizeof (ACCMData));

	return sizeof (ACCMData);
}



/*!
 * \brief encodeEXACMessage Fills an ISO EXAC struct with relevant data fields, and corresponding value IDs and content lengths
 * \param actionID ID of the action to be executed
 * \param executionTime Time when the action is to be executed
 * \param exacDataBuffer Buffer to which EXAC message is to be written
 * \param bufferLength Size of buffer to which EXAC message is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written or -1 in case of an error
 */
ssize_t encodeEXACMessage(const uint16_t * actionID, const struct timeval *executionTime,
						  char *exacDataBuffer, const size_t bufferLength, const char debug) {

	EXACType EXACData;

	memset(exacDataBuffer, 0, bufferLength);

	// If buffer too small to hold EXAC data, generate an error
	if (bufferLength < sizeof (EXACType)) {
		fprintf(stderr, "Buffer too small to hold necessary EXAC data\n");
		return -1;
	}

	// Construct header
	EXACData.header = buildISOHeader(MESSAGE_ID_EXAC, sizeof (EXACData), debug);

	// Fill contents
	EXACData.actionIDValueID = VALUE_ID_EXAC_ACTION_ID;
	EXACData.actionIDContentLength = sizeof (EXACData.actionID);
	EXACData.actionID = actionID == NULL ? ACTION_ID_UNAVAILABLE : *actionID;

	EXACData.executionTime_qmsoWValueID = VALUE_ID_EXAC_ACTION_EXECUTE_TIME;
	EXACData.executionTime_qmsoWContentLength = sizeof (EXACData.executionTime_qmsoW);
	int64_t execTime = getAsGPSQuarterMillisecondOfWeek(executionTime);

	EXACData.executionTime_qmsoW =
		executionTime == NULL || execTime < 0 ? GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE : (uint32_t) execTime;

	if (debug) {
		printf
			("EXAC message:\n\tAction ID value ID: 0x%x\n\tAction ID content length: %u\n\tAction ID: %u\n\t"
			 "Action execute time value ID: 0x%x\n\tAction execute time content length: %u\n\tAction execute time: %u [¼ ms]\n",
			 EXACData.actionIDValueID, EXACData.actionIDContentLength, EXACData.actionID,
			 EXACData.executionTime_qmsoWValueID, EXACData.executionTime_qmsoWContentLength,
			 EXACData.executionTime_qmsoW);
	}

	// Switch from host endianness to little endian
	EXACData.actionIDValueID = htole16(EXACData.actionIDValueID);
	EXACData.actionIDContentLength = htole16(EXACData.actionIDContentLength);
	EXACData.actionID = htole16(EXACData.actionID);
	EXACData.executionTime_qmsoWValueID = htole16(EXACData.executionTime_qmsoWValueID);
	EXACData.executionTime_qmsoWContentLength = htole16(EXACData.executionTime_qmsoWContentLength);
	EXACData.executionTime_qmsoW = htole32(EXACData.executionTime_qmsoW);

	// Construct footer
	EXACData.footer = buildISOFooter(&EXACData, sizeof (EXACData), debug);

	memcpy(exacDataBuffer, &EXACData, sizeof (EXACData));

	return sizeof (EXACType);
}

/*!
 * \brief encodeINSUPMessage Fills an ISO vendor specific (RISE) INSUP struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param command Command to send to supervisor
 * \param insupDataBuffer Data buffer to which INSUP is to be written
 * \param bufferLength Length of data buffer to which INSUP is to be written
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeINSUPMessage(const SupervisorCommandType command, char *insupDataBuffer,
						   const size_t bufferLength, const char debug) {
	INSUPType INSUPData;

	memset(insupDataBuffer, 0, bufferLength);

	// If buffer too small to hold EXAC data, generate an error
	if (bufferLength < sizeof (INSUPType)) {
		fprintf(stderr, "Buffer too small to hold necessary INSUP data\n");
		return -1;
	}

	// Construct header
	INSUPData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_RISE_INSUP, sizeof (INSUPData), debug);

	// Fill contents
	INSUPData.modeValueID = VALUE_ID_INSUP_MODE;
	INSUPData.modeContentLength = sizeof (INSUPData.mode);
	INSUPData.mode = (uint8_t) command;

	if (debug) {
		printf("INSUP message:\n\tMode value ID: 0x%x\n\t"
			   "Mode content length: %u\n\tMode: %u\n", INSUPData.modeValueID,
			   INSUPData.modeContentLength, INSUPData.mode);
	}

	// Switch from host endianness to little endian
	INSUPData.modeValueID = htole16(INSUPData.modeValueID);
	INSUPData.modeContentLength = htole16(INSUPData.modeContentLength);

	// Construct footer
	INSUPData.footer = buildISOFooter(&INSUPData, sizeof (INSUPData), debug);

	memcpy(insupDataBuffer, &INSUPData, sizeof (INSUPData));

	return sizeof (INSUPData);
}


/*!
 * \brief encodePODIMessage Fills an ISO vendor specific (AstaZero) PODI struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param peerObjectData Struct containing relevant PODI data
 * \param podiDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodePODIMessage(const PeerObjectInjectionType* peerObjectData, 
		char* podiDataBuffer,
		const size_t bufferLength,
		const char debug) {

	PODIType PODIData;

	memset(podiDataBuffer, 0, bufferLength);
	char* p = podiDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (peerObjectData == NULL) {
		fprintf(stderr, "PODI data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold PODI data, generate an error
	if (bufferLength < sizeof (PODIType)) {
		fprintf(stderr, "Buffer too small to hold necessary PODI data\n");
		return -1;
	}

	// Construct header
	PODIData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_PODI, sizeof (PODIData), debug);
	memcpy(p, &PODIData.header, sizeof (PODIData.header));
	p += sizeof (PODIData.header);
	remainingBytes -= sizeof (PODIData.header);

	if (debug) {
			printf("PODI message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_PODI_FOREIGN_TRANSMITTER_ID, &peerObjectData->foreignTransmitterID, &p,
						  sizeof (peerObjectData->foreignTransmitterID), &remainingBytes, &PODIForeignTransmitterIdDescription, debug);
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(&peerObjectData->dataTimestamp);
	PODIData.gpsQmsOfWeek = GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_GPS_QMS_OF_WEEK, &PODIData.gpsQmsOfWeek, &p,
						  sizeof (PODIData.gpsQmsOfWeek), &remainingBytes, &PODIGpsQmsOfWeekDescription, debug);
	PODIData.objectState = (uint8_t) peerObjectData->state;
	retval |= encodeContent(VALUE_ID_PODI_OBJECT_STATE, &PODIData.objectState, &p,
						  sizeof (PODIData.objectState), &remainingBytes, &PODIObjectStateDescription, debug);
	if (!peerObjectData->position.isPositionValid) {
		errno = EINVAL;
		fprintf(stderr, "Position is a required field in PODI messages\n");
		return -1;
	}
	PODIData.xPosition = (int32_t) (peerObjectData->position.xCoord_m * POSITION_ONE_METER_VALUE);
	PODIData.yPosition = (int32_t) (peerObjectData->position.yCoord_m * POSITION_ONE_METER_VALUE);
	PODIData.zPosition = (int32_t) (peerObjectData->position.zCoord_m * POSITION_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_PODI_X_POSITION, &PODIData.xPosition, &p,
						  sizeof (PODIData.xPosition), &remainingBytes, &PODIxPositionDescription, debug);
	retval |= encodeContent(VALUE_ID_PODI_Y_POSITION, &PODIData.yPosition, &p,
						  sizeof (PODIData.yPosition), &remainingBytes, &PODIyPositionDescription, debug);
	retval |= encodeContent(VALUE_ID_PODI_Z_POSITION, &PODIData.zPosition, &p,
						  sizeof (PODIData.zPosition), &remainingBytes, &PODIzPositionDescription, debug);
	PODIData.heading = peerObjectData->position.isHeadingValid ?
				(uint16_t) (mapHostHeadingToISOHeading(peerObjectData->position.heading_rad)
					* 180.0 / M_PI * HEADING_ONE_DEGREE_VALUE)
			  : HEADING_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_HEADING, &PODIData.heading, &p,
						  sizeof (PODIData.heading), &remainingBytes, &PODIHeadingDescription, debug);
	PODIData.pitch = peerObjectData->isPitchValid ?
				(uint16_t)(peerObjectData->pitch_rad * 180.0 / M_PI * PITCH_ONE_DEGREE_VALUE)
			  : PITCH_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_PITCH, &PODIData.pitch, &p,
						  sizeof (PODIData.pitch), &remainingBytes, &PODIPitchDescription, debug);
	PODIData.roll = peerObjectData->isRollValid ?
				(uint16_t)(peerObjectData->roll_rad * 180.0 / M_PI * ROLL_ONE_DEGREE_VALUE)
			  : ROLL_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_ROLL, &PODIData.roll, &p,
						  sizeof (PODIData.roll), &remainingBytes, &PODIRollDescription, debug);
	PODIData.longitudinalSpeed = peerObjectData->speed.isLongitudinalValid ?
				(int16_t) (peerObjectData->speed.longitudinal_m_s * SPEED_ONE_METER_PER_SECOND_VALUE)
			  : SPEED_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_LONGITUDINAL_SPEED, &PODIData.longitudinalSpeed, &p,
						  sizeof (PODIData.longitudinalSpeed), &remainingBytes, &PODILongitudinalSpeedDescription, debug);
	PODIData.lateralSpeed = peerObjectData->speed.isLateralValid ?
				(int16_t) (peerObjectData->speed.lateral_m_s * SPEED_ONE_METER_PER_SECOND_VALUE)
			  : SPEED_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_PODI_LATERAL_SPEED, &PODIData.lateralSpeed, &p,
						  sizeof (PODIData.lateralSpeed), &remainingBytes, &PODILongitudinalSpeedDescription, debug);

	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary PODI data\n");
		return -1;
	}

	if (debug) {
		printf("PODI message:\n\t<<debug printout not implemented>>\n");
	}

	// Construct footer
	PODIData.footer = buildISOFooter(&PODIData, sizeof (PODIData), debug);
	memcpy(p, &PODIData.footer, sizeof (PODIData.footer));
	p += sizeof (PODIData.footer);
	remainingBytes -= sizeof (PODIData.footer);

	if(debug)
	{
		printf("PODI message data (size = %d):\n", sizeof (PODIType));
		for(int i = 0; i < sizeof (PODIType); i++) printf("%x ", *(podiDataBuffer+i));
		printf("\n");
	}

	return p - podiDataBuffer;
}


/*!
 * \brief decodePODIMessage Fills PODI data elements from a buffer of raw data
 * \param podiDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used to guess GPS week of PODI message
 * \param peerData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodePODIMessage(
		const char *podiDataBuffer,
		const size_t bufferLength,
		const struct timeval currentTime,
		PeerObjectInjectionType* peerData,
		const char debug) {

	PODIType PODIData;
	const char *p = podiDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (podiDataBuffer == NULL || peerData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to PODI parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&PODIData, 0, sizeof (PODIData));
	memset(peerData, 0, sizeof (*peerData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &PODIData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (PODIData.header);

	// If message is not a PODI message, generate an error
	if (PODIData.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_PODI) {
		fprintf(stderr, "Attempted to pass non-PODI message into PODI parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (PODIData.header.MessageLengthU32 > sizeof (PODIType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "PODI message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - podiDataBuffer < PODIData.header.MessageLengthU32 + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_PODI_FOREIGN_TRANSMITTER_ID:
			memcpy(&PODIData.foreignTransmitterID, p, sizeof (PODIData.foreignTransmitterID));
			PODIData.foreignTransmitterIDValueID = valueID;
			PODIData.foreignTransmitterIDContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.foreignTransmitterID);
			break;
		case VALUE_ID_PODI_GPS_QMS_OF_WEEK:
			memcpy(&PODIData.gpsQmsOfWeek, p, sizeof (PODIData.gpsQmsOfWeek));
			PODIData.gpsQmsOfWeek = le32toh(PODIData.gpsQmsOfWeek);
			PODIData.gpsQmsOfWeekValueID = valueID;
			PODIData.gpsQmsOfWeekContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.gpsQmsOfWeek);
			break;
		case VALUE_ID_PODI_X_POSITION:
			memcpy(&PODIData.xPosition, p, sizeof (PODIData.xPosition));
			PODIData.xPosition = le32toh(PODIData.xPosition);
			PODIData.xPositionValueID = valueID;
			PODIData.xPositionContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.xPosition);
			break;
		case VALUE_ID_PODI_Y_POSITION:
			memcpy(&PODIData.yPosition, p, sizeof (PODIData.yPosition));
			PODIData.yPosition = le32toh(PODIData.yPosition);
			PODIData.yPositionValueID = valueID;
			PODIData.yPositionContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.yPosition);
			break;
		case VALUE_ID_PODI_Z_POSITION:
			memcpy(&PODIData.zPosition, p, sizeof (PODIData.zPosition));
			PODIData.zPosition = le32toh(PODIData.zPosition);
			PODIData.zPositionValueID = valueID;
			PODIData.zPositionContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.zPosition);
			break;
		case VALUE_ID_PODI_OBJECT_STATE:
			memcpy(&PODIData.objectState, p, sizeof (PODIData.objectState));
			PODIData.objectStateValueID = valueID;
			PODIData.objectStateContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.objectState);
			break;
		case VALUE_ID_PODI_HEADING:
			memcpy(&PODIData.heading, p, sizeof (PODIData.heading));
			PODIData.heading = le16toh(PODIData.heading);
			PODIData.headingValueID = valueID;
			PODIData.headingContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.heading);
			break;
		case VALUE_ID_PODI_PITCH:
			memcpy(&PODIData.pitch, p, sizeof (PODIData.pitch));
			PODIData.pitch = le16toh(PODIData.pitch);
			PODIData.pitchValueID = valueID;
			PODIData.pitchContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.pitch);
			break;
		case VALUE_ID_PODI_ROLL:
			memcpy(&PODIData.roll, p, sizeof (PODIData.roll));
			PODIData.roll = le16toh(PODIData.roll);
			PODIData.rollValueID = valueID;
			PODIData.rollContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.roll);
			break;
		case VALUE_ID_PODI_LATERAL_SPEED:
			memcpy(&PODIData.lateralSpeed, p, sizeof (PODIData.lateralSpeed));
			PODIData.lateralSpeed = le16toh(PODIData.lateralSpeed);
			PODIData.lateralSpeedValueID = valueID;
			PODIData.lateralSpeedContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.lateralSpeed);
			break;
		case VALUE_ID_PODI_LONGITUDINAL_SPEED:
			memcpy(&PODIData.longitudinalSpeed, p, sizeof (PODIData.longitudinalSpeed));
			PODIData.longitudinalSpeed = le16toh(PODIData.longitudinalSpeed);
			PODIData.longitudinalSpeedValueID = valueID;
			PODIData.longitudinalSpeedContentLength = contentLength;
			expectedContentLength = sizeof (PODIData.longitudinalSpeed);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known PODI value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - podiDataBuffer), &PODIData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding PODI footer\n");
		return retval;
	}
	p += sizeof (PODIData.footer);

	/*if ((retval = verifyChecksum(podiDataBuffer, PODIData.header.MessageLengthU32 + sizeof (HeaderType),
								 PODIData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "PODI checksum error\n");
		return retval;
	}*/

	if (debug) {
		printf("PODI message:\n");
		printf("\tForeign transmitter ID value ID: 0x%x\n", PODIData.foreignTransmitterIDValueID);
		printf("\tForeign transmitter ID content length: %u\n", PODIData.foreignTransmitterIDContentLength);
		printf("\tForeign transmitter ID: %u\n", PODIData.foreignTransmitterID);
		printf("\tGPS second of week value ID: 0x%x\n", PODIData.gpsQmsOfWeekValueID);
		printf("\tGPS second of week content length: %u\n", PODIData.gpsQmsOfWeekContentLength);
		printf("\tGPS second of week: %u [¼ ms]\n", PODIData.gpsQmsOfWeek);
		printf("\tObject state value ID: 0x%x\n", PODIData.objectStateValueID);
		printf("\tObject state content length: %u\n", PODIData.objectStateContentLength);
		printf("\tObject state: %u\n", PODIData.objectState);
		printf("\tx position value ID: 0x%x\n", PODIData.xPositionValueID);
		printf("\tx position content length: %u\n", PODIData.xPositionContentLength);
		printf("\tx position: %u\n", PODIData.xPosition);
		printf("\ty position value ID: 0x%x\n", PODIData.yPositionValueID);
		printf("\ty position content length: %u\n", PODIData.yPositionContentLength);
		printf("\ty position: %u\n", PODIData.yPosition);
		printf("\tz position value ID: 0x%x\n", PODIData.zPositionValueID);
		printf("\tz position content length: %u\n", PODIData.zPositionContentLength);
		printf("\tz position: %u\n", PODIData.zPosition);
		printf("\tHeading value ID: 0x%x\n", PODIData.headingValueID);
		printf("\tHeading content length: %u\n", PODIData.headingContentLength);
		printf("\tHeading: %u\n", PODIData.heading);
		printf("\tPitch value ID: 0x%x\n", PODIData.pitchValueID);
		printf("\tPitch content length: %u\n", PODIData.pitchContentLength);
		printf("\tPitch: %u\n", PODIData.pitch);
		printf("\tRoll value ID: 0x%x\n", PODIData.rollValueID);
		printf("\tRoll content length: %u\n", PODIData.rollContentLength);
		printf("\tRoll: %u\n", PODIData.roll);
		printf("\tLongitudinal speed value ID: 0x%x\n", PODIData.longitudinalSpeedValueID);
		printf("\tLongitudinal speed content length: %u\n", PODIData.longitudinalSpeedContentLength);
		printf("\tLongitudinal speed: %u\n", PODIData.longitudinalSpeed);
		printf("\tLateral speed value ID: 0x%x\n", PODIData.lateralSpeedValueID);
		printf("\tLateral speed content length: %u\n", PODIData.lateralSpeedContentLength);
		printf("\tLateral speed: %u\n", PODIData.lateralSpeed);
	}

	retval = convertPODIToHostRepresentation(&PODIData, &currentTime, peerData);

	return retval < 0 ? retval : p - podiDataBuffer;
}

/*!
 * \brief convertPODIToHostRepresentation Converts a PODI message to SI representation
 * \param PODIData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param peerData Output data struct, for SI representation data
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue convertPODIToHostRepresentation(PODIType* PODIData,
		const struct timeval* currentTime,
		PeerObjectInjectionType* peerData) {

	if (PODIData == NULL || currentTime == NULL || peerData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "PODI input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	if (!PODIData->foreignTransmitterIDValueID) {
		fprintf(stderr, "Foreign transmitter ID not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	peerData->foreignTransmitterID = PODIData->foreignTransmitterID;

	if (!PODIData->gpsQmsOfWeekValueID
			|| PODIData->gpsQmsOfWeek == GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE) {
		fprintf(stderr, "Timestamp not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	setToGPStime(&peerData->dataTimestamp, (uint16_t) getAsGPSWeek(currentTime), PODIData->gpsQmsOfWeek);

	peerData->state = PODIData->objectStateValueID ?
				PODIData->objectState : OBJECT_STATE_UNKNOWN;

	if (!PODIData->xPositionValueID
			|| !PODIData->yPositionValueID
			|| !PODIData->zPositionValueID) {
		fprintf(stderr, "Position not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	peerData->position.isPositionValid = 1;
	peerData->position.xCoord_m = PODIData->xPosition / POSITION_ONE_METER_VALUE;
	peerData->position.yCoord_m = PODIData->yPosition / POSITION_ONE_METER_VALUE;
	peerData->position.zCoord_m = PODIData->zPosition / POSITION_ONE_METER_VALUE;

	if (!PODIData->headingValueID) {
		fprintf(stderr, "Heading not supplied in PODI message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	peerData->position.isHeadingValid = PODIData->heading != HEADING_UNAVAILABLE_VALUE;
	peerData->position.heading_rad = mapISOHeadingToHostHeading(
				PODIData->heading / HEADING_ONE_DEGREE_VALUE * M_PI / 180.0);

	if (PODIData->rollValueID && PODIData->roll != ROLL_UNAVAILABLE_VALUE) {
		peerData->isRollValid = 1;
		peerData->roll_rad = PODIData->roll / ROLL_ONE_DEGREE_VALUE * M_PI / 180.0;
	}
	else {
		peerData->isRollValid = 0;
	}

	if (PODIData->pitchValueID && PODIData->pitch != PITCH_UNAVAILABLE_VALUE) {
		peerData->isPitchValid = 1;
		peerData->pitch_rad = PODIData->pitch / PITCH_ONE_DEGREE_VALUE * M_PI / 180.0;
	}
	else {
		peerData->isRollValid = 0;
	}

	// Until these are clearly defined, force them to be invalid
	peerData->isRollValid = 0;
	peerData->isPitchValid = 0;

	return MESSAGE_OK;
}

/*!
 * \brief decodeOPROMessage Decodes a buffer containing OPRO data into an object properties struct
 * \param objectPropertiesData Struct to be filled
 * \param oproDataBuffer Buffer containing data to be decoded
 * \param bufferLength Size of buffer containing data to be decoded
 * \param debug Parameter for enabling debugging
 * \return Value according to ::ISOMessageReturnValue
 */
ssize_t decodeOPROMessage(
		ObjectPropertiesType * objectPropertiesData,
		const char *oproDataBuffer,
		const size_t bufferLength,
		const char debug) {
	OPROType OPROData;
	const char *p = oproDataBuffer;

	uint16_t valueID;
	uint16_t contentLength;

	if (objectPropertiesData == NULL || oproDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	ssize_t retval = MESSAGE_OK;

	memset(objectPropertiesData, 0, sizeof (*objectPropertiesData));
	memset(&OPROData, 0, sizeof (OPROData));

	if ((retval = decodeISOHeader(p, bufferLength, &OPROData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (OPROData.header);

	// If message is not a OPRO message, generate an error
	if (OPROData.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO) {
		fprintf(stderr, "Attempted to pass non-OPRO message into OPRO parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}


	// Decode contents
	while ((size_t) (p - oproDataBuffer) < OPROData.header.MessageLengthU32 + sizeof (OPROData.header)) {
		// Decode value ID and length
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);

		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		// Handle contents
		switch (valueID) {
		case VALUE_ID_OPRO_OBJECT_TYPE:
			memcpy(&OPROData.objectTypeValueID, &valueID, sizeof (OPROData.objectTypeValueID));
			memcpy(&OPROData.objectTypeContentLength, &contentLength,
				   sizeof (OPROData.objectTypeContentLength));
			memcpy(&OPROData.objectType, p, sizeof (OPROData.objectType));
			break;
		case VALUE_ID_OPRO_ACTOR_TYPE:
			memcpy(&OPROData.actorTypeValueID, &valueID, sizeof (OPROData.actorTypeValueID));
			memcpy(&OPROData.actorTypeContentLength, &contentLength,
				   sizeof (OPROData.actorTypeContentLength));
			memcpy(&OPROData.actorType, p, sizeof (OPROData.actorType));
			break;
		case VALUE_ID_OPRO_OPERATION_MODE:
			memcpy(&OPROData.operationModeValueID, &valueID, sizeof (OPROData.operationModeValueID));
			memcpy(&OPROData.operationModeContentLength, &contentLength,
				   sizeof (OPROData.operationModeContentLength));
			memcpy(&OPROData.operationMode, p, sizeof (OPROData.operationMode));
			break;
		case VALUE_ID_OPRO_MASS:
			memcpy(&OPROData.massValueID, &valueID, sizeof (OPROData.massValueID));
			memcpy(&OPROData.massContentLength, &contentLength, sizeof (OPROData.massContentLength));
			memcpy(&OPROData.mass, p, sizeof (OPROData.mass));
			OPROData.mass = le32toh(OPROData.mass);
			break;
		case VALUE_ID_OPRO_OBJECT_LENGTH_X:
			memcpy(&OPROData.objectLengthXValueID, &valueID, sizeof (OPROData.objectLengthXValueID));
			memcpy(&OPROData.objectLengthXContentLength, &contentLength,
				   sizeof (OPROData.objectLengthXContentLength));
			memcpy(&OPROData.objectLengthX, p, sizeof (OPROData.objectLengthX));
			OPROData.objectLengthX = le32toh(OPROData.objectLengthX);
			break;
		case VALUE_ID_OPRO_OBJECT_LENGTH_Y:
			memcpy(&OPROData.objectLengthYValueID, &valueID, sizeof (OPROData.objectLengthYValueID));
			memcpy(&OPROData.objectLengthYContentLength, &contentLength,
				   sizeof (OPROData.objectLengthYContentLength));
			memcpy(&OPROData.objectLengthY, p, sizeof (OPROData.objectLengthY));
			OPROData.objectLengthY = le32toh(OPROData.objectLengthY);
			break;
		case VALUE_ID_OPRO_OBJECT_LENGTH_Z:
			memcpy(&OPROData.objectLengthZValueID, &valueID, sizeof (OPROData.objectLengthZValueID));
			memcpy(&OPROData.objectLengthZContentLength, &contentLength,
				   sizeof (OPROData.objectLengthZContentLength));
			memcpy(&OPROData.objectLengthZ, p, sizeof (OPROData.objectLengthZ));
			OPROData.objectLengthZ = le32toh(OPROData.objectLengthZ);
			break;
		case VALUE_ID_OPRO_POSITION_DISPLACEMENT_X:
			memcpy(&OPROData.positionDisplacementXValueID, &valueID,
				   sizeof (OPROData.positionDisplacementXValueID));
			memcpy(&OPROData.positionDisplacementXContentLength, &contentLength,
				   sizeof (OPROData.positionDisplacementXContentLength));
			memcpy(&OPROData.positionDisplacementX, p, sizeof (OPROData.positionDisplacementX));
			OPROData.positionDisplacementX = le16toh(OPROData.positionDisplacementX);
			break;
		case VALUE_ID_OPRO_POSITION_DISPLACEMENT_Y:
			memcpy(&OPROData.positionDisplacementYValueID, &valueID,
				   sizeof (OPROData.positionDisplacementYValueID));
			memcpy(&OPROData.positionDisplacementYContentLength, &contentLength,
				   sizeof (OPROData.positionDisplacementYContentLength));
			memcpy(&OPROData.positionDisplacementY, p, sizeof (OPROData.positionDisplacementY));
			OPROData.positionDisplacementY = le16toh(OPROData.positionDisplacementY);
			break;
		case VALUE_ID_OPRO_POSITION_DISPLACEMENT_Z:
			memcpy(&OPROData.positionDisplacementZValueID, &valueID,
				   sizeof (OPROData.positionDisplacementZValueID));
			memcpy(&OPROData.positionDisplacementZContentLength, &contentLength,
				   sizeof (OPROData.positionDisplacementZContentLength));
			memcpy(&OPROData.positionDisplacementZ, p, sizeof (OPROData.positionDisplacementZ));
			OPROData.positionDisplacementZ = le16toh(OPROData.positionDisplacementZ);
			break;

		default:
			printf("Unable to handle OPRO value ID 0x%x\n", valueID);
			break;
		}
		p += contentLength;
	}


	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - oproDataBuffer), &OPROData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding OPRO footer\n");
		return retval;
	}

	if ((retval = verifyChecksum(oproDataBuffer, OPROData.header.MessageLengthU32 + sizeof (OPROData.header),
								 OPROData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "OPRO checksum error\n");
		return retval;
	}

	if (debug) {
		printf("OPRO message:\n");
		printf("\tObject type value ID: 0x%x\n", OPROData.objectTypeValueID);
		printf("\tObject type content length: %u\n", OPROData.objectTypeContentLength);
		printf("\tObject type: %u\n", OPROData.objectType);
		printf("\tActor type value ID: 0x%x\n", OPROData.actorTypeValueID);
		printf("\tActor type content length: %u\n", OPROData.actorTypeContentLength);
		printf("\tActor type: %u\n", OPROData.actorType);
		printf("\tOperation mode value ID: 0x%x\n", OPROData.operationModeValueID);
		printf("\tOperation mode content length: %u\n", OPROData.operationModeContentLength);
		printf("\tOperation mode: %u\n", OPROData.operationMode);
		printf("\tMass value ID: 0x%x\n", OPROData.massValueID);
		printf("\tMass content length: %u\n", OPROData.massContentLength);
		printf("\tMass: %u [g]\n", OPROData.mass);
		printf("\tObject length X value ID: 0x%x\n", OPROData.objectLengthXValueID);
		printf("\tObject length X content length: %u\n", OPROData.objectLengthXContentLength);
		printf("\tObject length X: %u [mm]\n", OPROData.objectLengthX);
		printf("\tObject length Y value ID: 0x%x\n", OPROData.objectLengthYValueID);
		printf("\tObject length Y content length: %u\n", OPROData.objectLengthYContentLength);
		printf("\tObject length Y: %u [mm]\n", OPROData.objectLengthY);
		printf("\tObject length Z value ID: 0x%x\n", OPROData.objectLengthZValueID);
		printf("\tObject length Z content length: %u\n", OPROData.objectLengthZContentLength);
		printf("\tObject length Z: %u [mm]\n", OPROData.objectLengthZ);
		printf("\tPosition displacement X value ID: 0x%x\n", OPROData.positionDisplacementXValueID);
		printf("\tPosition displacement X content length: %u\n", OPROData.positionDisplacementXContentLength);
		printf("\tPosition displacement X: %d [mm]\n", OPROData.positionDisplacementX);
		printf("\tPosition displacement Y value ID: 0x%x\n", OPROData.positionDisplacementYValueID);
		printf("\tPosition displacement Y content length: %u\n", OPROData.positionDisplacementYContentLength);
		printf("\tPosition displacement Y: %d [mm]\n", OPROData.positionDisplacementY);
		printf("\tPosition displacement Z value ID: 0x%x\n", OPROData.positionDisplacementZValueID);
		printf("\tPosition displacement Z content length: %u\n", OPROData.positionDisplacementZContentLength);
		printf("\tPosition displacement Z: %d [mm]\n", OPROData.positionDisplacementZ);
	}

	// Fill output struct with parsed data
	retval = convertOPROToHostRepresentation(&OPROData, objectPropertiesData);
	return retval;
}

/*!
 * \brief encodeOPROMessage Fills an ISO vendor specific (AstaZero) OPRO struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param objectPropertiesData Struct containing relevant OPRO data
 * \param oproDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeOPROMessage(
		const ObjectPropertiesType* objectPropertiesData,
		char *oproDataBuffer,
		const size_t bufferLength,
		const char debug) {
	OPROType OPROData;

	memset(oproDataBuffer, 0, bufferLength);
	char* p = oproDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (objectPropertiesData == NULL) {
		fprintf(stderr, "OPRO data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold OPRO data, generate an error
	if (bufferLength < sizeof (OPROType)) {
		fprintf(stderr, "Buffer too small to hold necessary OPRO data\n");
		return -1;
	}

	// Construct header
	OPROData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_OPRO, sizeof (OPROData), debug);
	memcpy(p, &OPROData.header, sizeof (OPROData.header));
	p += sizeof (OPROData.header);
	remainingBytes -= sizeof (OPROData.header);

	if (debug) {
			printf("OPRO message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_TYPE, &objectPropertiesData->objectType, &p,
							sizeof (OPROData.objectType), &remainingBytes, &OPROObjectTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_OPRO_ACTOR_TYPE, &objectPropertiesData->actorType, &p,
							sizeof (OPROData.actorType), &remainingBytes, &OPROActorTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_OPRO_OPERATION_MODE, &objectPropertiesData->operationMode, &p,
							sizeof (OPROData.operationMode), &remainingBytes, &OPROOperationModeDescription, debug);

	if (objectPropertiesData->isMassValid) OPROData.mass = (uint32_t)(objectPropertiesData->mass_kg * MASS_ONE_KILOGRAM_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_MASS, &OPROData.mass, &p,
								sizeof (OPROData.mass), &remainingBytes, &OPROMassDescription, debug);

	if (objectPropertiesData->isObjectXDimensionValid) OPROData.objectLengthX = (uint32_t)(objectPropertiesData->objectXDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_LENGTH_X, &OPROData.objectLengthX, &p,
								sizeof (OPROData.objectLengthX), &remainingBytes, &OPROObjectLengthXDescription, debug);
 
	if (objectPropertiesData->isObjectYDimensionValid) OPROData.objectLengthY = (uint32_t)(objectPropertiesData->objectYDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_LENGTH_Y, &OPROData.objectLengthY, &p,
								sizeof (OPROData.objectLengthY), &remainingBytes, &OPROObjectLengthYDescription, debug);

	if (objectPropertiesData->isObjectZDimensionValid) OPROData.objectLengthZ = (uint32_t)(objectPropertiesData->objectZDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_OBJECT_LENGTH_Z, &OPROData.objectLengthZ, &p,
								sizeof (OPROData.objectLengthZ), &remainingBytes, &OPROObjectLengthZDescription, debug);

	if (objectPropertiesData->isObjectXDisplacementValid) OPROData.positionDisplacementX = (int16_t)(objectPropertiesData->positionDisplacementX_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_POSITION_DISPLACEMENT_X, &OPROData.positionDisplacementX, &p,
								sizeof (OPROData.positionDisplacementX), &remainingBytes, &OPROPositionDisplacementXDescription, debug);

	if (objectPropertiesData->isObjectYDisplacementValid) OPROData.positionDisplacementY = (int16_t)(objectPropertiesData->positionDisplacementY_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_POSITION_DISPLACEMENT_Y, &OPROData.positionDisplacementY, &p,
								sizeof (OPROData.positionDisplacementY), &remainingBytes, &OPROPositionDisplacementYDescription, debug);

	if (objectPropertiesData->isObjectZDisplacementValid) OPROData.positionDisplacementZ = (int16_t)(objectPropertiesData->positionDisplacementZ_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_OPRO_POSITION_DISPLACEMENT_Z, &OPROData.positionDisplacementZ, &p,
								sizeof (OPROData.positionDisplacementZ), &remainingBytes, &OPROPositionDisplacementZDescription, debug);

	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary OPRO data\n");
		return -1;
	}
	
	// Construct footer
	OPROData.footer = buildISOFooter(oproDataBuffer, (size_t)(p - oproDataBuffer) + sizeof (FooterType), debug);
	memcpy(p, &OPROData.footer, sizeof (OPROData.footer));
	p += sizeof (OPROData.footer);
	remainingBytes -= sizeof (OPROData.footer);
	if(debug)
	{
		printf("OPRO message data (size = %d):\n", sizeof (OPROType));
		for(int i = 0; i < sizeof (OPROType); i++) printf("%x ", *(oproDataBuffer+i));
		printf("\n");
	}
	return p - oproDataBuffer;
}

/*!
 * \brief encodeFOPRMessage Fills an ISO vendor specific (AstaZero) FOPR struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param foreignObjectPropertiesData Struct containing relevant OPRO data
 * \param foprDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeFOPRMessage(
		const ForeignObjectPropertiesType* foreignObjectPropertiesData,
		char *foprDataBuffer,
		const size_t bufferLength,
		const char debug) {
	FOPRType FOPRData;

	memset(foprDataBuffer, 0, bufferLength);
	char* p = foprDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (foreignObjectPropertiesData == NULL) {
		fprintf(stderr, "FOPR data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold FOPR data, generate an error
	if (bufferLength < sizeof (FOPRType)) {
		fprintf(stderr, "Buffer too small to hold necessary FOPR data\n");
		return -1;
	}

	// Construct header
	FOPRData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_FOPR, sizeof (FOPRData), debug);
	memcpy(p, &FOPRData.header, sizeof (FOPRData.header));
	p += sizeof (FOPRData.header);
	remainingBytes -= sizeof (FOPRData.header);

	if (debug) {
		printf("FOPR message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_FOPR_FOREIGN_TRANSMITTER_ID, &foreignObjectPropertiesData->foreignTransmitterID, &p,
							sizeof (FOPRData.foreignTransmitterID), &remainingBytes, &FOPRForeignTransmitterIDDescription, debug);

	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_TYPE, &foreignObjectPropertiesData->objectType, &p,
							sizeof (FOPRData.objectType), &remainingBytes, &FOPRObjectTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_FOPR_ACTOR_TYPE, &foreignObjectPropertiesData->actorType, &p,
							sizeof (FOPRData.actorType), &remainingBytes, &FOPRActorTypeDescription, debug);

	retval |= encodeContent(VALUE_ID_FOPR_OPERATION_MODE, &foreignObjectPropertiesData->operationMode, &p,
							sizeof (FOPRData.operationMode), &remainingBytes, &FOPROperationModeDescription, debug);

	if (foreignObjectPropertiesData->isMassValid) FOPRData.mass = (uint32_t)(foreignObjectPropertiesData->mass_kg * MASS_ONE_KILOGRAM_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_MASS, &FOPRData.mass, &p,
								sizeof (FOPRData.mass), &remainingBytes, &FOPRMassDescription, debug);

	if (foreignObjectPropertiesData->isObjectXDimensionValid) FOPRData.objectLengthX = (uint32_t)(foreignObjectPropertiesData->objectXDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_LENGTH_X, &FOPRData.objectLengthX, &p,
								sizeof (FOPRData.objectLengthX), &remainingBytes, &FOPRObjectLengthXDescription, debug);
 
	if (foreignObjectPropertiesData->isObjectYDimensionValid) FOPRData.objectLengthY = (uint32_t)(foreignObjectPropertiesData->objectYDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_LENGTH_Y, &FOPRData.objectLengthY, &p,
								sizeof (FOPRData.objectLengthY), &remainingBytes, &FOPRObjectLengthYDescription, debug);

	if (foreignObjectPropertiesData->isObjectZDimensionValid) FOPRData.objectLengthZ = (uint32_t)(foreignObjectPropertiesData->objectZDimension_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_OBJECT_LENGTH_Z, &FOPRData.objectLengthZ, &p,
								sizeof (FOPRData.objectLengthZ), &remainingBytes, &FOPRObjectLengthZDescription, debug);

	if (foreignObjectPropertiesData->isObjectXDisplacementValid) FOPRData.positionDisplacementX = (int16_t)(foreignObjectPropertiesData->positionDisplacementX_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_POSITION_DISPLACEMENT_X, &FOPRData.positionDisplacementX, &p,
								sizeof (FOPRData.positionDisplacementX), &remainingBytes, &FOPRPositionDisplacementXDescription, debug);

	if (foreignObjectPropertiesData->isObjectYDisplacementValid) FOPRData.positionDisplacementY = (int16_t)(foreignObjectPropertiesData->positionDisplacementY_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_POSITION_DISPLACEMENT_Y, &FOPRData.positionDisplacementY, &p,
								sizeof (FOPRData.positionDisplacementY), &remainingBytes, &FOPRPositionDisplacementYDescription, debug);

	if (foreignObjectPropertiesData->isObjectZDisplacementValid) FOPRData.positionDisplacementZ = (int16_t)(foreignObjectPropertiesData->positionDisplacementZ_m * LENGTH_ONE_METER_VALUE);
	retval |= encodeContent(VALUE_ID_FOPR_POSITION_DISPLACEMENT_Z, &FOPRData.positionDisplacementZ, &p,
								sizeof (FOPRData.positionDisplacementZ), &remainingBytes, &FOPRPositionDisplacementZDescription, debug);

	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary OPRO data\n");
		return -1;
	}
	
	// Construct footer
	FOPRData.footer = buildISOFooter(foprDataBuffer, (size_t)(p - foprDataBuffer) + sizeof (FooterType), debug);
	memcpy(p, &FOPRData.footer, sizeof (FOPRData.footer));
	p += sizeof (FOPRData.footer);
	remainingBytes -= sizeof (FOPRData.footer);
	if(debug)
	{
		printf("FOPR message data (size = %d):\n", sizeof (FOPRType));
		for(int i = 0; i < sizeof (FOPRType); i++) printf("%x ", *(foprDataBuffer+i));
		printf("\n");
	}
	return p - foprDataBuffer;
}

/*!
 * \brief convertOPROToHostRepresentation Converts a OPRO message to SI representation
 * \param OPROData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param peerData Output data struct, for SI representation data
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue convertOPROToHostRepresentation(const OPROType* OPROData,
		ObjectPropertiesType* objectProperties) {

	if (OPROData == NULL || objectProperties == NULL) {
		errno = EINVAL;
		fprintf(stderr, "OPRO input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	objectProperties->isMassValid = OPROData->massValueID && OPROData->mass != MASS_UNAVAILABLE_VALUE;
	objectProperties->isObjectXDimensionValid = OPROData->objectLengthXValueID && OPROData->objectLengthX != LENGTH_UNAVAILABLE_VALUE;
	objectProperties->isObjectYDimensionValid = OPROData->objectLengthYValueID && OPROData->objectLengthY != LENGTH_UNAVAILABLE_VALUE;
	objectProperties->isObjectZDimensionValid = OPROData->objectLengthZValueID && OPROData->objectLengthZ != LENGTH_UNAVAILABLE_VALUE;
	objectProperties->isObjectXDisplacementValid = OPROData->objectLengthXValueID && OPROData->positionDisplacementX != POSITION_UNAVAILABLE_VALUE;
	objectProperties->isObjectYDisplacementValid = OPROData->objectLengthYValueID && OPROData->positionDisplacementY != POSITION_UNAVAILABLE_VALUE;
	objectProperties->isObjectZDisplacementValid = OPROData->objectLengthZValueID && OPROData->positionDisplacementZ != POSITION_UNAVAILABLE_VALUE;

	objectProperties->objectID = OPROData->header.TransmitterIdU8;

	objectProperties->objectType = OPROData->objectTypeValueID ? OPROData->objectType : OBJECT_CATEGORY_UNKNOWN;
	objectProperties->actorType = OPROData->actorTypeValueID ? OPROData->actorType : ACTOR_TYPE_UNKNOWN;
	objectProperties->operationMode = OPROData->operationModeValueID ? OPROData->operationMode : OPERATION_MODE_UNKNOWN;

	objectProperties->mass_kg = objectProperties->isMassValid ? (double)(OPROData->mass) / MASS_ONE_KILOGRAM_VALUE : 0;
	objectProperties->objectXDimension_m = objectProperties->isObjectXDimensionValid ?
				(double)(OPROData->objectLengthX) / LENGTH_ONE_METER_VALUE : 0;
	objectProperties->objectYDimension_m = objectProperties->isObjectYDimensionValid ?
				(double)(OPROData->objectLengthY) / LENGTH_ONE_METER_VALUE : 0;
	objectProperties->objectZDimension_m = objectProperties->isObjectZDimensionValid ?
				(double)(OPROData->objectLengthZ) / LENGTH_ONE_METER_VALUE : 0;
	objectProperties->positionDisplacementX_m = objectProperties->isObjectXDisplacementValid ?
				(double)(OPROData->positionDisplacementX) / POSITION_ONE_METER_VALUE : 0;
	objectProperties->positionDisplacementY_m = objectProperties->isObjectYDisplacementValid ?
				(double)(OPROData->positionDisplacementY) / POSITION_ONE_METER_VALUE : 0;
	objectProperties->positionDisplacementZ_m = objectProperties->isObjectZDisplacementValid ?
				(double)(OPROData->positionDisplacementZ) / POSITION_ONE_METER_VALUE : 0;

	return MESSAGE_OK;
}


/*!
 * \brief convertFOPRToHostRepresentation Converts a FOPR message to SI representation
 * \param FOPRData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param peerData Output data struct, for SI representation data
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue convertFOPRToHostRepresentation(const FOPRType* FOPRData,
		ForeignObjectPropertiesType* foreignObjectProperties) {

	if (FOPRData == NULL || foreignObjectProperties == NULL) {
		errno = EINVAL;
		fprintf(stderr, "FOPR input pointer error");
		return ISO_FUNCTION_ERROR;
	}
	
	foreignObjectProperties->isMassValid = FOPRData->massValueID && FOPRData->mass != MASS_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectXDimensionValid = FOPRData->objectLengthXValueID && FOPRData->objectLengthX != LENGTH_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectYDimensionValid = FOPRData->objectLengthYValueID && FOPRData->objectLengthY != LENGTH_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectZDimensionValid = FOPRData->objectLengthZValueID && FOPRData->objectLengthZ != LENGTH_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectXDisplacementValid = FOPRData->objectLengthXValueID && FOPRData->positionDisplacementX != POSITION_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectYDisplacementValid = FOPRData->objectLengthYValueID && FOPRData->positionDisplacementY != POSITION_UNAVAILABLE_VALUE;
	foreignObjectProperties->isObjectZDisplacementValid = FOPRData->objectLengthZValueID && FOPRData->positionDisplacementZ != POSITION_UNAVAILABLE_VALUE;

	foreignObjectProperties->foreignTransmitterID = FOPRData->header.TransmitterIdU8;

	foreignObjectProperties->objectType = FOPRData->objectTypeValueID ? FOPRData->objectType : OBJECT_CATEGORY_UNKNOWN;
	foreignObjectProperties->actorType = FOPRData->actorTypeValueID ? FOPRData->actorType : ACTOR_TYPE_UNKNOWN;
	foreignObjectProperties->operationMode = FOPRData->operationModeValueID ? FOPRData->operationMode : OPERATION_MODE_UNKNOWN;

	foreignObjectProperties->mass_kg = foreignObjectProperties->isMassValid ? (double)(FOPRData->mass) / MASS_ONE_KILOGRAM_VALUE : 0;
	foreignObjectProperties->objectXDimension_m = foreignObjectProperties->isObjectXDimensionValid ?
				(double)(FOPRData->objectLengthX) / LENGTH_ONE_METER_VALUE : 0;
	foreignObjectProperties->objectYDimension_m = foreignObjectProperties->isObjectYDimensionValid ?
				(double)(FOPRData->objectLengthY) / LENGTH_ONE_METER_VALUE : 0;
	foreignObjectProperties->objectZDimension_m = foreignObjectProperties->isObjectZDimensionValid ?
				(double)(FOPRData->objectLengthZ) / LENGTH_ONE_METER_VALUE : 0;
	foreignObjectProperties->positionDisplacementX_m = foreignObjectProperties->isObjectXDisplacementValid ?
				(double)(FOPRData->positionDisplacementX) / POSITION_ONE_METER_VALUE : 0;
	foreignObjectProperties->positionDisplacementY_m = foreignObjectProperties->isObjectYDisplacementValid ?
				(double)(FOPRData->positionDisplacementY) / POSITION_ONE_METER_VALUE : 0;
	foreignObjectProperties->positionDisplacementZ_m = foreignObjectProperties->isObjectZDisplacementValid ?
				(double)(FOPRData->positionDisplacementZ) / POSITION_ONE_METER_VALUE : 0;

	return MESSAGE_OK;
}

/*!
 * \brief decodeFOPRMessage Decodes a buffer containing FOPR data into an object properties struct
 * \param foreignObjectPropertiesData Struct to be filled
 * \param foprDataBuffer Buffer containing data to be decoded
 * \param bufferLength Size of buffer containing data to be decoded
 * \param debug Parameter for enabling debugging
 * \return Value according to ::ISOMessageReturnValue
 */
ssize_t decodeFOPRMessage(
		ForeignObjectPropertiesType * foreignObjectPropertiesData,
		const char *foprDataBuffer,
		const size_t bufferLength,
		const char debug) {
	FOPRType FOPRData;
	const char *p = foprDataBuffer;

	uint16_t valueID;
	uint16_t contentLength;

	if(debug)
	{
		printf("FOPR message data (size = %d):\n", sizeof (FOPRType));
		for(int i = 0; i < sizeof (FOPRType); i++) printf("%x ", *(foprDataBuffer+i));
		printf("\n");
	}

	if (foreignObjectPropertiesData == NULL || foprDataBuffer == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	ssize_t retval = MESSAGE_OK;

	memset(foreignObjectPropertiesData, 0, sizeof (*foreignObjectPropertiesData));
	memset(&FOPRData, 0, sizeof (FOPRData));

	if ((retval = decodeISOHeader(p, bufferLength, &FOPRData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (FOPRData.header);

	// If message is not a FOPR message, generate an error
	if (FOPRData.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_FOPR) {
		fprintf(stderr, "Attempted to pass non-FOPR message into FOPR parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}


	// Decode contents
	while ((size_t) (p - foprDataBuffer) < FOPRData.header.MessageLengthU32 + sizeof (FOPRData.header)) {
		// Decode value ID and length
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);

		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		// Handle contents
		switch (valueID) {
		case VALUE_ID_FOPR_FOREIGN_TRANSMITTER_ID:
			memcpy(&FOPRData.foreignTransmitterIDValueID, &valueID, sizeof (FOPRData.foreignTransmitterIDValueID));
			memcpy(&FOPRData.foreignTransmitterIDContentLength, &contentLength,
				   sizeof (FOPRData.foreignTransmitterIDContentLength));
			memcpy(&FOPRData.foreignTransmitterID, p, sizeof (FOPRData.foreignTransmitterID));
			break;
		case VALUE_ID_FOPR_OBJECT_TYPE:
			memcpy(&FOPRData.objectTypeValueID, &valueID, sizeof (FOPRData.objectTypeValueID));
			memcpy(&FOPRData.objectTypeContentLength, &contentLength,
				   sizeof (FOPRData.objectTypeContentLength));
			memcpy(&FOPRData.objectType, p, sizeof (FOPRData.objectType));
			break;
		case VALUE_ID_FOPR_ACTOR_TYPE:
			memcpy(&FOPRData.actorTypeValueID, &valueID, sizeof (FOPRData.actorTypeValueID));
			memcpy(&FOPRData.actorTypeContentLength, &contentLength,
				   sizeof (FOPRData.actorTypeContentLength));
			memcpy(&FOPRData.actorType, p, sizeof (FOPRData.actorType));
			break;
		case VALUE_ID_FOPR_OPERATION_MODE:
			memcpy(&FOPRData.operationModeValueID, &valueID, sizeof (FOPRData.operationModeValueID));
			memcpy(&FOPRData.operationModeContentLength, &contentLength,
				   sizeof (FOPRData.operationModeContentLength));
			memcpy(&FOPRData.operationMode, p, sizeof (FOPRData.operationMode));
			break;
		case VALUE_ID_FOPR_MASS:
			memcpy(&FOPRData.massValueID, &valueID, sizeof (FOPRData.massValueID));
			memcpy(&FOPRData.massContentLength, &contentLength, sizeof (FOPRData.massContentLength));
			memcpy(&FOPRData.mass, p, sizeof (FOPRData.mass));
			FOPRData.mass = le32toh(FOPRData.mass);
			break;
		case VALUE_ID_FOPR_OBJECT_LENGTH_X:
			memcpy(&FOPRData.objectLengthXValueID, &valueID, sizeof (FOPRData.objectLengthXValueID));
			memcpy(&FOPRData.objectLengthXContentLength, &contentLength,
				   sizeof (FOPRData.objectLengthXContentLength));
			memcpy(&FOPRData.objectLengthX, p, sizeof (FOPRData.objectLengthX));
			FOPRData.objectLengthX = le32toh(FOPRData.objectLengthX);
			break;
		case VALUE_ID_FOPR_OBJECT_LENGTH_Y:
			memcpy(&FOPRData.objectLengthYValueID, &valueID, sizeof (FOPRData.objectLengthYValueID));
			memcpy(&FOPRData.objectLengthYContentLength, &contentLength,
				   sizeof (FOPRData.objectLengthYContentLength));
			memcpy(&FOPRData.objectLengthY, p, sizeof (FOPRData.objectLengthY));
			FOPRData.objectLengthY = le32toh(FOPRData.objectLengthY);
			break;
		case VALUE_ID_FOPR_OBJECT_LENGTH_Z:
			memcpy(&FOPRData.objectLengthZValueID, &valueID, sizeof (FOPRData.objectLengthZValueID));
			memcpy(&FOPRData.objectLengthZContentLength, &contentLength,
				   sizeof (FOPRData.objectLengthZContentLength));
			memcpy(&FOPRData.objectLengthZ, p, sizeof (FOPRData.objectLengthZ));
			FOPRData.objectLengthZ = le32toh(FOPRData.objectLengthZ);
			break;
		case VALUE_ID_FOPR_POSITION_DISPLACEMENT_X:
			memcpy(&FOPRData.positionDisplacementXValueID, &valueID,
				   sizeof (FOPRData.positionDisplacementXValueID));
			memcpy(&FOPRData.positionDisplacementXContentLength, &contentLength,
				   sizeof (FOPRData.positionDisplacementXContentLength));
			memcpy(&FOPRData.positionDisplacementX, p, sizeof (FOPRData.positionDisplacementX));
			FOPRData.positionDisplacementX = le16toh(FOPRData.positionDisplacementX);
			break;
		case VALUE_ID_FOPR_POSITION_DISPLACEMENT_Y:
			memcpy(&FOPRData.positionDisplacementYValueID, &valueID,
				   sizeof (FOPRData.positionDisplacementYValueID));
			memcpy(&FOPRData.positionDisplacementYContentLength, &contentLength,
				   sizeof (FOPRData.positionDisplacementYContentLength));
			memcpy(&FOPRData.positionDisplacementY, p, sizeof (FOPRData.positionDisplacementY));
			FOPRData.positionDisplacementY = le16toh(FOPRData.positionDisplacementY);
			break;
		case VALUE_ID_FOPR_POSITION_DISPLACEMENT_Z:
			memcpy(&FOPRData.positionDisplacementZValueID, &valueID,
				   sizeof (FOPRData.positionDisplacementZValueID));
			memcpy(&FOPRData.positionDisplacementZContentLength, &contentLength,
				   sizeof (FOPRData.positionDisplacementZContentLength));
			memcpy(&FOPRData.positionDisplacementZ, p, sizeof (FOPRData.positionDisplacementZ));
			FOPRData.positionDisplacementZ = le16toh(FOPRData.positionDisplacementZ);
			break;

		default:
			printf("Unable to handle FOPR value ID 0x%x\n", valueID);
			break;
		}
		p += contentLength;
	}


	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - foprDataBuffer), &FOPRData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding FOPR footer\n");
		return retval;
	}

	if ((retval = verifyChecksum(foprDataBuffer, FOPRData.header.MessageLengthU32 + sizeof (FOPRData.header),
								 FOPRData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "FOPR checksum error\n");
		return retval;
	}

	if (debug) {
		printf("FOPR message:\n");
		printf("\tForeign transmitter value ID: 0x%x\n", FOPRData.foreignTransmitterIDValueID);
		printf("\tForeign transmitter ID content length: %u\n", FOPRData.foreignTransmitterIDContentLength);
		printf("\tForeign transmitter ID: %u\n", FOPRData.foreignTransmitterID);

		printf("\tObject type value ID: 0x%x\n", FOPRData.objectTypeValueID);
		printf("\tObject type content length: %u\n", FOPRData.objectTypeContentLength);
		printf("\tObject type: %u\n", FOPRData.objectType);
		printf("\tActor type value ID: 0x%x\n", FOPRData.actorTypeValueID);
		printf("\tActor type content length: %u\n", FOPRData.actorTypeContentLength);
		printf("\tActor type: %u\n", FOPRData.actorType);
		printf("\tOperation mode value ID: 0x%x\n", FOPRData.operationModeValueID);
		printf("\tOperation mode content length: %u\n", FOPRData.operationModeContentLength);
		printf("\tOperation mode: %u\n", FOPRData.operationMode);
		printf("\tObject length X value ID: 0x%x\n", FOPRData.objectLengthXValueID);
		printf("\tObject length X content length: %u\n", FOPRData.objectLengthXContentLength);
		printf("\tObject length X: %u [mm]\n", FOPRData.objectLengthX);
		printf("\tObject length Y value ID: 0x%x\n", FOPRData.objectLengthYValueID);
		printf("\tObject length Y content length: %u\n", FOPRData.objectLengthYContentLength);
		printf("\tObject length Y: %u [mm]\n", FOPRData.objectLengthY);
		printf("\tObject length Z value ID: 0x%x\n", FOPRData.objectLengthZValueID);
		printf("\tObject length Z content length: %u\n", FOPRData.objectLengthZContentLength);
		printf("\tObject length Z: %u [mm]\n", FOPRData.objectLengthZ);
		printf("\tPosition displacement X value ID: 0x%x\n", FOPRData.positionDisplacementX);
		printf("\tPosition displacement X content length: %u\n", FOPRData.positionDisplacementX);
		printf("\tPosition displacement X: %d [mm]\n", FOPRData.positionDisplacementX);
		printf("\tPosition displacement Y value ID: 0x%x\n", FOPRData.positionDisplacementY);
		printf("\tPosition displacement Y content length: %u\n", FOPRData.positionDisplacementY);
		printf("\tPosition displacement Y: %d [mm]\n", FOPRData.positionDisplacementY);
		printf("\tPosition displacement Z value ID: 0x%x\n", FOPRData.positionDisplacementZ);
		printf("\tPosition displacement Z content length: %u\n", FOPRData.positionDisplacementZ);
		printf("\tPosition displacement Z: %d [mm]\n", FOPRData.positionDisplacementZ);
	}

	// Fill output struct with parsed data
	retval = convertFOPRToHostRepresentation(&FOPRData, foreignObjectPropertiesData);
	return retval < 0 ? retval : p - foprDataBuffer;
}

/*!
 * \brief encodeGDRMMessage Constructs an ISO GDRM message (General Data Request Message)
 * \param gdrmData Struct containing relevant GDRM data
 * \param gdrmDataBuffer Data buffer in which to place encoded GDRM message
 * \param bufferLength Size of data buffer in which to place encoded GDRM message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeGDRMMessage(const GdrmMessageDataType *gdrmData, char *gdrmDataBuffer, const size_t bufferLength,
						  const char debug) {

	 GDRMType GDRMData;

	 memset(gdrmDataBuffer, 0, bufferLength);
	 char* p = gdrmDataBuffer;
	 size_t remainingBytes = bufferLength;
	 int retval = 0;

	 if (gdrmData == NULL) {
		 fprintf(stderr, "GDRM data input pointer error\n");
		 return -1;
	 }

	 // If buffer too small to hold GDRM data, generate an error
	 if (bufferLength < sizeof (GDRMType)) {
		 fprintf(stderr, "Buffer too small to hold necessary GDRM data\n");
		 return -1;
	 }

	 // Construct header
	 GDRMData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_GDRM, sizeof (GDRMData), debug);
	 memcpy(p, &GDRMData.header, sizeof (GDRMData.header));
	 p += sizeof (GDRMData.header);
	 remainingBytes -= sizeof (GDRMData.header);

	 if (debug) {
			 printf("GDRM message:\n");
	 }
	 // Fill contents
	 retval |= encodeContent(VALUE_ID_GDRM_DATA_CODE, &gdrmData->dataCode, &p,
						   sizeof (gdrmData->dataCode), &remainingBytes, &GDRMDataCodeDescription, debug);


	 if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		 fprintf(stderr, "Buffer too small to hold necessary GDRM data\n");
		 return -1;
	 }

	 if (debug) {
		 printf("GDRM message:\n\t<<debug printout not implemented>>\n");
	 }

	 // Construct footer
	 GDRMData.footer = buildISOFooter(&GDRMData, sizeof (GDRMData), debug);
	 memcpy(p, &GDRMData.footer, sizeof (GDRMData.footer));
	 p += sizeof (GDRMData.footer);
	 remainingBytes -= sizeof (GDRMData.footer);

	 if(debug)
	 {
		 printf("GDRM message data (size = %d):\n", sizeof (GDRMType));
		 for(int i = 0; i < sizeof (GDRMType); i++) printf("%x ", *(gdrmDataBuffer+i));
		 printf("\n");
	 }

	 return p - gdrmDataBuffer;
 }


 /*!
  * \brief decodeGDRMMessage Fills GDRM data elements from a buffer of raw data
  * \param gdrmDataBuffer Raw data to be decoded
  * \param bufferLength Number of bytes in buffer of raw data to be decoded
  * \param gdrmData Struct to be filled
  * \param debug Flag for enabling of debugging
  * \return value according to ::ISOMessageReturnValue
  */
 ISOMessageReturnValue decodeGDRMMessage(const char *gdrmDataBuffer,										const size_t bufferLength,
										 GdrmMessageDataType* gdrmData,
										 const char debug) {

	 GDRMType GDRMData;
	 const char *p = gdrmDataBuffer;
	 ISOMessageReturnValue retval = MESSAGE_OK;
	 uint16_t valueID = 0;
	 uint16_t contentLength = 0;
	 ssize_t expectedContentLength = 0;

	 if (gdrmDataBuffer == NULL || gdrmData == NULL) {
		 errno = EINVAL;
		 fprintf(stderr, "Input pointers to GDRM parsing function cannot be null\n");
		 return ISO_FUNCTION_ERROR;
	 }

	 memset(&GDRMData, 0, sizeof (GDRMData));
	 memset(gdrmData, 0, sizeof (*gdrmData));

	 // Decode ISO header
	 if ((retval = decodeISOHeader(p, bufferLength, &GDRMData.header, debug)) != MESSAGE_OK) {
		 return retval;
	 }
	 p += sizeof (GDRMData.header);

	 // If message is not a GDRM message, generate an error
	 if (GDRMData.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_GDRM) {
		 fprintf(stderr, "Attempted to pass non-GDRM message into GDRM parsing function\n");
		 return MESSAGE_TYPE_ERROR;
	 }


	 if (GDRMData.header.MessageLengthU32 > sizeof (GDRMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		 fprintf(stderr, "GDRM message exceeds expected message length\n");
		 return MESSAGE_LENGTH_ERROR;
	 }

	 //If data is added to the message add the retreiving code here

	 // Decode footer
	 if ((retval =
		  decodeISOFooter(p, bufferLength - (size_t) (p - gdrmDataBuffer), &GDRMData.footer,
						  debug)) != MESSAGE_OK) {
		 fprintf(stderr, "Error decoding GDRM footer\n");
		 return retval;
	 }


	 if (debug) {
		 printf("GDRM message:\n");
		 printf("\tMessage id: 0x%x\n", GDRMData.header.MessageIdU16);
	 }

	 retval = convertGDRMToHostRepresentation(&GDRMData, gdrmData);

	 return retval;
 }



 /*!
  * \brief convertGDRMToHostRepresentation Converts a GDRM message to be used by host
  * \param GDRMData Data struct containing ISO formatted data
  * \param gdrmData Output data struct, to be used by host
  * \return Value according to ::ISOMessageReturnValue
  */
 ISOMessageReturnValue convertGDRMToHostRepresentation(GDRMType* GDRMData,
		 GdrmMessageDataType* gdrmData) {


	 if (GDRMData == NULL || gdrmData == NULL) {
		 errno = EINVAL;
		 fprintf(stderr, "GDRM input pointer error");
		 return ISO_FUNCTION_ERROR;
	 }

	 gdrmData->dataCode = GDRMData->DataCode;


	 return MESSAGE_OK;
 }




/*!
 * \brief encodeDCTIMessage Constructs an ISO DCTI message (Direct Control Transmitter Id)
 * \param dctiData Struct containing relevant DCTI data
 * \param dctiDataBuffer Data buffer in which to place encoded DCTI message
 * \param bufferLength Size of data buffer in which to place encoded DCTI message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeDCTIMessage(const DctiMessageDataType *dctiData,
						char *dctiDataBuffer, const size_t bufferLength, const char debug) {

	DCTIType DCTIData;

	memset(dctiDataBuffer, 0, bufferLength);
	char* p = dctiDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (dctiData == NULL) {
		fprintf(stderr, "DCTI data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold DCTI data, generate an error
	if (bufferLength < sizeof (DCTIType)) {
		fprintf(stderr, "Buffer too small to hold necessary DCTI data\n");
		return -1;
	}

	// Construct header
	DCTIData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCTI, sizeof (DCTIData), debug);
	memcpy(p, &DCTIData.header, sizeof (DCTIData.header));
	p += sizeof (DCTIData.header);
	remainingBytes -= sizeof (DCTIData.header);

	if (debug) {
			printf("DCTI message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_DCTI_TOTAL_COUNT, &dctiData->totalCount, &p,
						  sizeof (dctiData->totalCount), &remainingBytes, &DCTITotalCountDescription, debug);
	retval |= encodeContent(VALUE_ID_DCTI_COUNTER, &dctiData->counter, &p,
						  sizeof (dctiData->counter), &remainingBytes, &DCTICounterDescription, debug);
	retval |= encodeContent(VALUE_ID_DCTI_TRANSMITTER_ID, &dctiData->transmitterID, &p,
						  sizeof (dctiData->transmitterID), &remainingBytes, &DCTITransmitterIdDescription, debug);


	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary DCTI data\n");
		return -1;
	}

	if (debug) {
		printf("DCTI message:\n\t<<debug printout not implemented>>\n");
	}

	// Construct footer
	DCTIData.footer = buildISOFooter(&DCTIData, sizeof (DCTIData), debug);
	memcpy(p, &DCTIData.footer, sizeof (DCTIData.footer));
	p += sizeof (DCTIData.footer);
	remainingBytes -= sizeof (DCTIData.footer);

	if(debug)
	{
		printf("DCTI message data (size = %d):\n", sizeof (DCTIType));
		for(int i = 0; i < sizeof (DCTIType); i++) printf("%x ", *(dctiDataBuffer+i));
		printf("\n");
	}

	return p - dctiDataBuffer;


}


/*!
 * \brief decodeDCTIMessage Fills HEAB data elements from a buffer of raw data
 * \param dctiDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param dctiData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue decodeDCTIMessage(const char *dctiDataBuffer,
										const size_t bufferLength,
										DctiMessageDataType* dctiData,
										const char debug) {
	DCTIType DCTIData;
	const char *p = dctiDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (dctiDataBuffer == NULL || dctiData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to DCTI parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&DCTIData, 0, sizeof (DCTIData));
	memset(dctiData, 0, sizeof (*dctiData));
	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &DCTIData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (DCTIData.header);

	// If message is not a PODI message, generate an error
	if (DCTIData.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCTI) {
		fprintf(stderr, "Attempted to pass non-DCTI message into DCTI parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (DCTIData.header.MessageLengthU32 > sizeof (DCTIType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "PODI message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - dctiDataBuffer < DCTIData.header.MessageLengthU32 + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_DCTI_TOTAL_COUNT:
			memcpy(&DCTIData.TotalCount, p, sizeof (DCTIData.TotalCount));
			DCTIData.TotalCountValueIdU16 = valueID;
			DCTIData.TotalCountContentLengthU16 = contentLength;
			expectedContentLength = sizeof (DCTIData.TotalCount);
			break;
		case VALUE_ID_DCTI_COUNTER:
			memcpy(&DCTIData.Counter, p, sizeof (DCTIData.Counter));
			DCTIData.CounterValueIdU16 = valueID;
			DCTIData.CounterContentLengthU16 = contentLength;
			expectedContentLength = sizeof (DCTIData.Counter);
			break;
		case VALUE_ID_DCTI_TRANSMITTER_ID:
			memcpy(&DCTIData.TransmitterID, p, sizeof (DCTIData.TransmitterID));
			DCTIData.TransmitterIDValueIdU16 = valueID;
			DCTIData.TransmitterIDContentLengthU16 = contentLength;
			expectedContentLength = sizeof (DCTIData.TransmitterID);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known DCTI value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - dctiDataBuffer), &DCTIData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding DCTI footer\n");
		return retval;
	}
	p += sizeof (DCTIData.footer);

	/*if ((retval = verifyChecksum(dctiDataBuffer, DCTIData.header.MessageLengthU32 + sizeof (HeaderType),
								 DCTIData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "DCTI checksum error\n");
		return retval;
	}*/

	if (debug) {
		printf("DCTI message:\n");
		printf("\tTotal count value ID: 0x%x\n", DCTIData.TotalCountValueIdU16);
		printf("\tTotal count content length: %u\n", DCTIData.TotalCountContentLengthU16);
		printf("\tTotal count: %u\n", DCTIData.TotalCount);
		printf("\tCounter value ID: 0x%x\n", DCTIData.CounterValueIdU16);
		printf("\tCounter content length: %u\n", DCTIData.CounterContentLengthU16);
		printf("\tCounter: %u\n", DCTIData.Counter);
		printf("\tTransmitter id value ID: 0x%x\n", DCTIData.TransmitterIDValueIdU16);
		printf("\tTransmitter id content length: %u\n", DCTIData.TransmitterIDContentLengthU16);
		printf("\tTransmitter id: %u\n", DCTIData.TransmitterID);
	}

	retval = convertDCTIToHostRepresentation(&DCTIData, dctiData);

	return retval < 0 ? retval : p - dctiDataBuffer;
}


/*!
 * \brief convertDCTIToHostRepresentation Converts a DCTI message for client usage
 * \param DCTIData Data struct containing ISO formatted data
 * \param dctiData Output data struct
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue convertDCTIToHostRepresentation(DCTIType* DCTIData,
		DctiMessageDataType* dctiData) {

	if (DCTIData == NULL || dctiData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "DCTI input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	dctiData->totalCount = DCTIData->TotalCount;
	dctiData->counter = DCTIData->Counter;
	dctiData->transmitterID = DCTIData->TransmitterID;


	return MESSAGE_OK;
}

/*!
 * \brief encodeRDCAMessage Constructs an ISO RDCA message (Request Direct Control Action)
 * \param rdcaData Requested action data
 * \param rdcaDataBuffer Data buffer in which to place encoded RDCA message
 * \param bufferLength Size of data buffer in which to place encoded RDCA message
 * \param debug Flag for enabling debugging
 * \return number of bytes written to the data buffer, or -1 if an error occurred
 */
ssize_t encodeRDCAMessage(const RequestControlActionType *rdcaData,
						  char *rdcaDataBuffer,
						  const size_t bufferLength, 
						  const char debug) {
	RDCAType RDCAData;

	memset(rdcaDataBuffer, 0, bufferLength);
	char* p = rdcaDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (rdcaData == NULL || rdcaDataBuffer == NULL) {
		fprintf(stderr, "RDCA data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold RDCA data, generate an error
	if (bufferLength < sizeof (RDCAType)) {
		fprintf(stderr, "Buffer too small to hold necessary RDCA data\n");
		return -1;
	}
	
	
	size_t unusedMemory = 0;
	if (!rdcaData->isSteeringActionValid) {
		unusedMemory += sizeof (RDCAData.steeringActionValueID)
				+ sizeof (RDCAData.steeringActionContentLength)
				+ sizeof (RDCAData.steeringAction);
	}
	if (!rdcaData->isSpeedActionValid) {
			unusedMemory += sizeof (RDCAData.speedActionValueID)
				+ sizeof (RDCAData.speedActionContentLength)	
				+ sizeof (RDCAData.speedAction);
	}
	// Construct header
	RDCAData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_RDCA, (uint32_t)(sizeof (RDCAData)-unusedMemory), debug);
	memcpy(p, &RDCAData.header, sizeof (RDCAData.header));
	p += sizeof (RDCAData.header);
	remainingBytes -= sizeof (RDCAData.header);

	if (debug) {
		printf("RDCA message:\n");
	}
	// Fill contents
	RDCAData.intendedReceiverID = rdcaData->executingID;
	retval |= encodeContent(VALUE_ID_RDCA_INTENDED_RECEIVER, &RDCAData.intendedReceiverID, &p,
							sizeof (RDCAData.intendedReceiverID), &remainingBytes, &RDCAIntendedReceiverDescription, debug);
							
	int64_t GPSQmsOfWeek = getAsGPSQuarterMillisecondOfWeek(&rdcaData->dataTimestamp);
	RDCAData.gpsQmsOfWeek = GPSQmsOfWeek >= 0 ? (uint32_t) GPSQmsOfWeek : GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE;
	retval |= encodeContent(VALUE_ID_RDCA_GPS_QMS_OF_WEEK, &RDCAData.gpsQmsOfWeek, &p,
						  sizeof (RDCAData.gpsQmsOfWeek), &remainingBytes, &RDCAGpsQmsOfWeekDescription, debug);
	
	
	if (rdcaData->isSteeringActionValid && rdcaData->steeringUnit == ISO_UNIT_TYPE_STEERING_DEGREES) {	
		if(rdcaData->steeringAction.rad <= STEERING_ANGLE_MAX_VALUE_RAD && rdcaData->steeringAction.rad >= STEERING_ANGLE_MIN_VALUE_RAD) { 
			RDCAData.steeringAction = (int16_t) (rdcaData->steeringAction.rad * (180.0 / M_PI) * STEERING_ANGLE_ONE_DEGREE_VALUE);
			retval |= encodeContent(VALUE_ID_RDCA_STEERING_ANGLE, &RDCAData.steeringAction, &p,
								sizeof (RDCAData.steeringAction), &remainingBytes, &RDCASteeringDescriptionDeg, debug);	

		}
		else {
			fprintf(stderr, "Steering value is out of bounds for angle value\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	else if (rdcaData->isSteeringActionValid && rdcaData->steeringUnit == ISO_UNIT_TYPE_STEERING_PERCENTAGE) {
		if(rdcaData->steeringAction.pct <= MAX_VALUE_PERCENTAGE && rdcaData->steeringAction.pct >= MIN_VALUE_PERCENTAGE) {
			RDCAData.steeringAction = (int16_t) (rdcaData->steeringAction.pct);
			retval |= encodeContent(VALUE_ID_RDCA_STEERING_PERCENTAGE, &RDCAData.steeringAction, &p,
								sizeof(RDCAData.steeringAction), &remainingBytes, &RDCASteeringDescriptionPct, debug);	
		}
		else {
			fprintf(stderr, "Steering value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}

	if(rdcaData->isSpeedActionValid && rdcaData->speedUnit == ISO_UNIT_TYPE_SPEED_METER_SECOND) {
		RDCAData.speedAction = (int16_t) (rdcaData->speedAction.m_s *SPEED_ONE_METER_PER_SECOND_VALUE);
		retval |= encodeContent(VALUE_ID_RDCA_SPEED_METER_PER_SECOND, &RDCAData.speedAction, &p,
								sizeof(RDCAData.speedAction), &remainingBytes, &RDCASpeedDescription_m_s, debug);
	}
	// TODO: Fix so error state is in else and approv in if
	else if(rdcaData->isSpeedActionValid && rdcaData->speedUnit == ISO_UNIT_TYPE_SPEED_PERCENTAGE) {
		if (rdcaData->speedAction.pct <= MAX_VALUE_PERCENTAGE && rdcaData->speedAction.pct >= MIN_VALUE_PERCENTAGE) {

			RDCAData.speedAction = (int16_t) (rdcaData->speedAction.pct);

		retval |= encodeContent(VALUE_ID_RDCA_SPEED_PERCENTAGE, &RDCAData.speedAction, &p,
								sizeof(RDCAData.speedAction), &remainingBytes, &RDCASpeedDescriptionPct, debug);
		}
		else {
			fprintf(stderr, "Speed value is out of bounds for percentage\n");
			return MESSAGE_CONTENT_OUT_OF_RANGE;
		}
	}
	
	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary RDCA data\n");
		return -1;
	}

	// Construct footer
	RDCAData.footer = buildISOFooter(&RDCAData, (size_t) (p-rdcaDataBuffer) + sizeof(RDCAData.footer), debug);
	memcpy(p, &RDCAData.footer, sizeof (RDCAData.footer));
	p += sizeof (RDCAData.footer);
	remainingBytes -= sizeof (RDCAData.footer);

	if(debug) {
		printf("RDCA message data (size = %lu):\n", sizeof (RDCAType));
		for(size_t i = 0; i < sizeof (RDCAType); i++) printf("%x ", *(rdcaDataBuffer+i));
		printf("\n");
	}

	return p - rdcaDataBuffer;
}


/*!
 * \brief decodeRDCAMessage Fills RDCA data elements from a buffer of raw data
 * \param rdcaDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param currentTime Current system time, used to guess GPS week of PODI message
 * \param peerData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeRDCAMessage(
		const char *rdcaDataBuffer,
		RequestControlActionType *rdcaData,
		const size_t bufferLength,
		const struct timeval currentTime,
		const char debug) {

	RDCAType RDCAData;
	const char *p = rdcaDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (rdcaDataBuffer == NULL || rdcaData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to RDCA parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&RDCAData, 0, sizeof (RDCAData));
	memset(rdcaData, 0, sizeof (*rdcaData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &RDCAData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (RDCAData.header);

	// If message is not a RDCA message, generate an error
	if (RDCAData.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_RDCA) {
		fprintf(stderr, "Attempted to pass non-RDCA message into RDA parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (RDCAData.header.MessageLengthU32 > sizeof (RDCAType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "RDCA message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - rdcaDataBuffer < RDCAData.header.MessageLengthU32 + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_RDCA_GPS_QMS_OF_WEEK:
			memcpy(&RDCAData.gpsQmsOfWeek, p, sizeof (RDCAData.gpsQmsOfWeek));
			RDCAData.gpsQmsOfWeek = le32toh(RDCAData.gpsQmsOfWeek);
			RDCAData.gpsQmsOfWeekValueID = valueID;
			RDCAData.gpsQmsOfWeekContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.gpsQmsOfWeek);
			break;
		case VALUE_ID_RDCA_STEERING_ANGLE:
			memcpy(&RDCAData.steeringAction, p, sizeof (RDCAData.steeringAction));
			RDCAData.steeringAction = (int16_t)le16toh(RDCAData.steeringAction);
			RDCAData.steeringActionValueID = valueID;
			RDCAData.steeringActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.steeringAction);
			break;
		case VALUE_ID_RDCA_INTENDED_RECEIVER:
			memcpy(&RDCAData.intendedReceiverID, p, sizeof (RDCAData.intendedReceiverID));
			RDCAData.intendedReceiverID = le32toh(RDCAData.intendedReceiverID);
			RDCAData.intendedReceiverIDValueID = valueID;
			RDCAData.intendedReceiverIDContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.intendedReceiverID);
			break;
		case VALUE_ID_RDCA_STEERING_PERCENTAGE:
			memcpy(&RDCAData.steeringAction, p, sizeof (RDCAData.steeringAction));
			RDCAData.steeringAction = (int16_t)le16toh(RDCAData.steeringAction);
			RDCAData.steeringActionValueID = valueID;
			RDCAData.steeringActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.steeringAction);
			break;
		case VALUE_ID_RDCA_SPEED_METER_PER_SECOND:
			memcpy(&RDCAData.speedAction, p, sizeof (RDCAData.speedAction));
			RDCAData.speedAction = (int16_t)le16toh(RDCAData.speedAction);
			RDCAData.speedActionValueID = valueID;
			RDCAData.speedActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.speedAction);
			break;
		case VALUE_ID_RDCA_SPEED_PERCENTAGE:
			memcpy(&RDCAData.speedAction, p, sizeof (RDCAData.speedAction));
			RDCAData.speedAction = (int16_t)le16toh(RDCAData.speedAction);
			RDCAData.speedActionValueID = valueID;
			RDCAData.speedActionContentLength = contentLength;
			expectedContentLength = sizeof (RDCAData.speedAction);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known RDCA value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - rdcaDataBuffer), &RDCAData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding RDCA footer\n");
		return retval;
	}
	p += sizeof (RDCAData.footer);

	/*if ((retval = verifyChecksum(rdcaDataBuffer, RDCAData.header.MessageLengthU32 + sizeof (HeaderType),
								 RDCAData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "RDCA checksum error\n");
		return retval;
	}*/

	if (debug) {
		printf("RDCA message:\n");
		printContent(RDCAData.gpsQmsOfWeekValueID, RDCAData.gpsQmsOfWeekContentLength,
					&RDCAData.gpsQmsOfWeek, &RDCAGpsQmsOfWeekDescription);
		if (RDCAData.steeringActionValueID == VALUE_ID_RDCA_STEERING_ANGLE)
			printContent(RDCAData.steeringActionValueID, RDCAData.steeringActionContentLength,
						&RDCAData.steeringAction, &RDCASteeringDescriptionDeg);
		else if (RDCAData.steeringActionValueID == VALUE_ID_RDCA_STEERING_PERCENTAGE)
			printContent(RDCAData.steeringActionValueID, RDCAData.steeringActionContentLength,
						&RDCAData.steeringAction, &RDCASteeringDescriptionPct);
		if (RDCAData.speedActionValueID == VALUE_ID_RDCA_SPEED_METER_PER_SECOND)
			printContent(RDCAData.speedActionValueID, RDCAData.speedActionContentLength,
						&RDCAData.speedAction, &RDCASpeedDescription_m_s);
		else if (RDCAData.speedActionValueID == VALUE_ID_RDCA_SPEED_PERCENTAGE)
			printContent(RDCAData.speedActionValueID, RDCAData.speedActionContentLength,
						&RDCAData.speedAction, &RDCASpeedDescriptionPct);
	}

	retval = convertRDCAToHostRepresentation(&RDCAData, &currentTime, rdcaData);

	return retval < 0 ? retval : p - rdcaDataBuffer;
}



/*!
 * \brief convertRDCAToHostRepresentation Converts a RDCA message to SI representation
 * \param RDCAData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param rdcaData Output data struct
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue convertRDCAToHostRepresentation(RDCAType* RDCAData,
			const struct timeval* currentTime, RequestControlActionType* rdcaData) {

	if (RDCAData == NULL ||  rdcaData == NULL || currentTime == NULL) {
		errno = EINVAL;
		fprintf(stderr, "RDCA input pointer error\n");
		return ISO_FUNCTION_ERROR;
	}

	if (!RDCAData->gpsQmsOfWeekValueID
			|| RDCAData->gpsQmsOfWeek == GPS_SECOND_OF_WEEK_UNAVAILABLE_VALUE) {
		fprintf(stderr, "Timestamp not supplied in RDCA message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}
	if (!RDCAData->gpsQmsOfWeekValueID) {
		fprintf(stderr, "Receiver ID not supplied in RDCA message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}
	
	
	setToGPStime(&rdcaData->dataTimestamp, (uint16_t) getAsGPSWeek(currentTime), RDCAData->gpsQmsOfWeek);
	// Fill steering values and check out of range values
	rdcaData->executingID = RDCAData->intendedReceiverID;
	if (RDCAData->steeringAction != STEERING_ANGLE_UNAVAILABLE_VALUE && RDCAData->steeringActionValueID) {
		if(RDCAData->steeringActionValueID == VALUE_ID_RDCA_STEERING_ANGLE) {
			if (RDCAData->steeringAction <= STEERING_ANGLE_MAX_VALUE_DEG
			&& RDCAData->steeringAction >= STEERING_ANGLE_MIN_VALUE_DEG) {
				rdcaData->isSteeringActionValid = 1;
				rdcaData->steeringAction.rad = RDCAData->steeringAction / STEERING_ANGLE_ONE_DEGREE_VALUE * M_PI / 180.0;
				rdcaData->steeringUnit = ISO_UNIT_TYPE_STEERING_DEGREES; 
			}
			else {
				fprintf(stderr, "Steering angle value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else if(RDCAData->steeringActionValueID == VALUE_ID_RDCA_STEERING_PERCENTAGE) {
			if (RDCAData->steeringAction <= MAX_VALUE_PERCENTAGE
			&& RDCAData->steeringAction >= MIN_VALUE_PERCENTAGE) {
				rdcaData->isSteeringActionValid = 1;
				rdcaData->steeringAction.pct = RDCAData->steeringAction;
				rdcaData->steeringUnit = ISO_UNIT_TYPE_STEERING_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Steering percentage value is out of bounds\n");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
		}
		else {
			fprintf(stderr, "Steering Value ID error\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}

	// Fill speed values and check out of range values
	if (RDCAData->speedAction != SPEED_UNAVAILABLE_VALUE && RDCAData->speedActionValueID) {
		if(RDCAData->speedActionValueID == VALUE_ID_RDCA_SPEED_METER_PER_SECOND) {
				rdcaData->isSpeedActionValid = 1;
				rdcaData->speedAction.m_s = RDCAData->speedAction /SPEED_ONE_METER_PER_SECOND_VALUE;
				rdcaData->speedUnit = ISO_UNIT_TYPE_SPEED_METER_SECOND; 
			}
		else if(RDCAData->speedActionValueID == VALUE_ID_RDCA_SPEED_PERCENTAGE) {
			if (RDCAData->speedAction <= MAX_VALUE_PERCENTAGE 
				&& RDCAData->speedAction >= MIN_VALUE_PERCENTAGE) {
				rdcaData->isSpeedActionValid = 1;
				rdcaData->speedAction.pct = RDCAData->speedAction;
				rdcaData->speedUnit = ISO_UNIT_TYPE_SPEED_PERCENTAGE;
			}
			else {
				fprintf(stderr, "Speed percentage value is out of bounds");
				return MESSAGE_CONTENT_OUT_OF_RANGE;
			}
			
		}
		else {
			fprintf(stderr, "Receiver ID not supplied in RDCA message\n");
			return MESSAGE_VALUE_ID_ERROR;
		}
	}
	
	return MESSAGE_OK;
}


/*!
 * \brief decodeGREMMessage Fills GREM data elements from a buffer of raw data
 * \param gremDataBuffer Raw data to be decoded
 * \param bufferLength Number of bytes in buffer of raw data to be decoded
 * \param gremData Struct to be filled
 * \param debug Flag for enabling of debugging
 * \return value according to ::ISOMessageReturnValue
 */
ssize_t decodeGREMMessage(
		const char *gremDataBuffer,
		const size_t bufferLength,
		GeneralResponseMessageType* gremData,
		const char debug) {

	GREMType GREMdata;
	const char *p = gremDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;

	if (gremDataBuffer == NULL || gremData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to GREM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&GREMdata, 0, sizeof (GREMdata));
	memset(gremData, 0, sizeof (*gremData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &GREMdata.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (GREMdata.header);

	// If message is not a GREM message, generate an error
	if (GREMdata.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_GREM) {
		fprintf(stderr, "Attempted to pass non-GREM message into GREM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (GREMdata.header.MessageLengthU32 > sizeof (GREMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "GREM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - gremDataBuffer < GREMdata.header.MessageLengthU32 + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);

		switch (valueID) {
		case VALUE_ID_GREM_STATUS:
			memcpy(&GREMdata.status, p, sizeof (GREMdata.status));
			GREMdata.statusValueID = valueID;
			GREMdata.statusContentLength = contentLength;
			expectedContentLength = sizeof (GREMdata.statusValueID);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known GREM value IDs", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - gremDataBuffer), &GREMdata.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding GREM footer\n");
		return retval;
	}
	p += sizeof (GREMdata.footer);

	if (debug) {
		printf("GREM message:\n");
		printf("\tStatus transmitter ID value ID: 0x%x\n", GREMdata.statusValueID);
		printf("\tStatus transmitter ID content length: %u\n", GREMdata.statusContentLength);
		printf("\tStatus: %u\n", GREMdata.status);
	}

	// Fill output struct with parsed data
	retval = convertGREMoHostRepresentation(&GREMdata, gremData);
}

/*!
 * \brief convertPODIToHostRepresentation Converts a PODI message to SI representation
 * \param PODIData Data struct containing ISO formatted data
 * \param currentTime Current system time, used for determining the GPS week
 * \param peerData Output data struct, for SI representation data
 * \return Value according to ::ISOMessageReturnValue
 */
ISOMessageReturnValue convertGREMoHostRepresentation(GREMType* GREMdata,
		GeneralResponseMessageType* gremData) {

	if (GREMdata == NULL || gremData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "GREM input pointer error");
		return ISO_FUNCTION_ERROR;
	}

	if (!GREMdata->statusValueID ) {
		fprintf(stderr, "Status Value ID not supplied in GREM message\n");
		return MESSAGE_VALUE_ID_ERROR;
	}

	gremData->status = GREMdata->status;

	return MESSAGE_OK;
}


/*!
 * \brief encodeGREMMessage Fills an ISO vendor specific (AstaZero) GREM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param gremObjectData Struct containing relevant GREM data
 * \param gremDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeGREMMessage(const GeneralResponseMessageType* gremObjectData,
		char* gremDataBuffer,
		const size_t bufferLength,
		const char debug) {

	GREMType GREMData;

	memset(gremDataBuffer, 0, bufferLength);
	char* p = gremDataBuffer;
	size_t remainingBytes = bufferLength;
	int retval = 0;

	if (gremObjectData == NULL) {
		fprintf(stderr, "GREM data input pointer error\n");
		return -1;
	}

	// If buffer too small to hold GREM data, generate an error
	if (bufferLength < sizeof (GREMType)) {
		fprintf(stderr, "Buffer too small to hold necessary GREM data\n");
		return -1;
	}

	// Construct header
	GREMData.header = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_GREM, sizeof (GREMData), debug);
	memcpy(p, &GREMData.header, sizeof (GREMData.header));
	p += sizeof (GREMData.header);
	remainingBytes -= sizeof (GREMData.header);

	if (debug) {
			printf("GREM message:\n");
	}
	// Fill contents
	retval |= encodeContent(VALUE_ID_GREM_STATUS, &gremObjectData->status, &p,
						  sizeof (gremObjectData->status), &remainingBytes, &GREMStatusDescription, debug);

	if (retval != 0 || remainingBytes < sizeof (FooterType)) {
		fprintf(stderr, "Buffer too small to hold necessary GREM data\n");
		return -1;
	}

	if (debug) {
		printf("GREM message:\n\t<<debug printout not implemented>>\n");
	}

	// Construct footer
	GREMData.footer = buildISOFooter(&GREMData, sizeof (GREMData), debug);
	memcpy(p, &GREMData.footer, sizeof (GREMData.footer));
	p += sizeof (GREMData.footer);
	remainingBytes -= sizeof (GREMData.footer);


	return p - gremDataBuffer;
}


/*!
 * \brief mapISOHeadingToHostHeading Converts between ISO NED heading to internal heading measured from the test x axis
 * \param isoHeading_rad Heading measured according to ISO specification, in radians
 * \return Heading, in radians, measured from the test x axis
 */
double_t mapISOHeadingToHostHeading(const double_t isoHeading_rad) {
	// TODO: Reevaluate this when ISO specification is updated with new heading and rotated coordinate system

	double_t retval = isoHeading_rad;

	// Host heading is CCW while ISO is CW
	retval = -retval;
	// Host heading is measured from the x axis while ISO is measured from the y axis
	retval = retval + M_PI / 2.0;
	// Ensure angle lies between 0 and 2pi
	while (retval < 0.0) {
		retval += 2.0 * M_PI;
	}
	while (retval >= 2.0 * M_PI) {
		retval -= 2.0 * M_PI;
	}
	return retval;
}

/*!
 * \brief mapHostHeadingToISOHeading Converts between internal heading measured from the test x axis to ISO NED heading
 * \param isoHeading_rad Heading measured form test x axis, in radians
 * \return Heading, in radians, measured as specified by ISO 22133
 */
double_t mapHostHeadingToISOHeading(const double_t hostHeading_rad) {
	// TODO: Reevaluate this when ISO specification is updated with new heading and rotated coordinate system

	double_t retval = hostHeading_rad;

	// Host heading is CCW while ISO is CW
	retval = -retval;
	// Host heading is measured from the x axis while ISO is measured from the y axis
	retval = retval + M_PI / 2.0;
	// Ensure angle lies between 0 and 2pi
	while (retval < 0.0) {
		retval += 2.0 * M_PI;
	}
	while (retval >= 2.0 * M_PI) {
		retval -= 2.0 * M_PI;
	}
	return retval;
}

/*!
 * \brief setToGPStime Sets an Epoch timestamp according to supplied GPS time data
 * \param time Output epoch timestamp
 * \param GPSWeek GPS week
 * \param GPSqmsOfWeek GPS quarter millisecond of GPSWeek
 * \return 0 on success, -1 otherwise
 */
int8_t setToGPStime(struct timeval * time, const uint16_t GPSWeek, const uint32_t GPSqmsOfWeek) {
	if (time == NULL) {
		errno = EINVAL;
		return -1;
	}
	// Conserve precision of quarter milliseconds when building time struct
	uint64_t GPSqms = (uint64_t) (GPSWeek) * WEEK_TIME_QMS + (uint64_t) (GPSqmsOfWeek);
	uint64_t UTCqms = GPSqms + 4 * MS_TIME_DIFF_UTC_GPS - 4 * MS_LEAP_SEC_DIFF_UTC_GPS;

	time->tv_sec = (time_t) (UTCqms / 4000);
	time->tv_usec = (time_t) ((UTCqms % 4000) * 250);
	return 0;
}

/*!
 * \brief getAsGPSms Converts a timestamp into GPS milliseconds
 * \param time Epoch timestamp
 * \return GPS milliseconds, or 0 in case of an error
 */
uint64_t getAsGPSms(const struct timeval * time) {
	if (time == NULL) {
		errno = EINVAL;
		return 0;
	}
	if (time->tv_sec < 0) {
		errno = ERANGE;
		return 0;
	}
	return (uint64_t) (time->tv_sec * 1000 + time->tv_usec / 1000)
		- MS_TIME_DIFF_UTC_GPS;
}


/*!
 * \brief getAsGPSweek Converts a timestamp into the corresponding GPS week
 * \param time Epoch timestamp
 * \return Time represented as number of weeks, or -1 if error
 */
int32_t getAsGPSWeek(const struct timeval * time) {
	uint64_t GPSms;

	if ((GPSms = getAsGPSms(time)) == 0) {
		return -1;
	}
	return (uint16_t) (GPSms / WEEK_TIME_MS);
}

/*!
 * \brief getAsGPSQuarterMillisecondOfWeek Converts a timestamp into quarter
 *			milliseconds of the GPS week it represents.
 * \param time Epoch timestamp
 * \return Time represented as quarter milliseconds of week, or -1 if error
 */
int64_t getAsGPSQuarterMillisecondOfWeek(const struct timeval * time) {
	if (time == NULL) {
		errno = EINVAL;
		return -1;
	}
	uint64_t UTCqms = (uint64_t)(((int64_t)time->tv_sec)*4000 + ((int64_t)time->tv_usec)/250);
	uint64_t GPSqms = UTCqms - (uint64_t)(4*MS_TIME_DIFF_UTC_GPS - 4*MS_LEAP_SEC_DIFF_UTC_GPS);

	return (int64_t) (GPSqms % WEEK_TIME_QMS);
}

/*!
 * \brief encodeDCMMessage Fills an ISO vendor specific (AstaZero) DCMM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * \param command Struct containing relevant DCMM data
 * \param dcmmDataBuffer Data buffer to which message is to be printed
 * \param bufferLength Available memory in data buffer
 * \param debug Flag for enabling debugging
 * \return Number of bytes written to buffer, or -1 in case of error
 */
ssize_t encodeDCMMMessage(const RemoteControlManoeuvreMessageType* command,
		char* dcmmDataBuffer,
		const size_t bufferLength,
		const char debug) {

	HeaderType DCMMHeader;
	FooterType DCMMFooter;
	RCMMType DCMMData;

	DCMMHeader = buildISOHeader(MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCMM, sizeof(DCMMData), debug);

	ssize_t retval =  encodeRCMMMessage(command, dcmmDataBuffer, bufferLength, debug);
	if (retval < 0) {
		fprintf(stderr, "DCMM wrapper error\n");
		return retval;
	}
	memcpy(dcmmDataBuffer, &DCMMHeader, sizeof(DCMMHeader) );

	DCMMFooter = buildISOFooter(dcmmDataBuffer, retval, debug);
	memcpy(dcmmDataBuffer + retval - sizeof(DCMMFooter), &DCMMFooter, sizeof(DCMMFooter));

	return retval;
}
/**
 * @brief decodeDCMMessage Fills an ISO vendor specific (AstaZero) DCMM struct with relevant data fields,
 *		and corresponding value IDs and content lengths
 * @param dcmmDataBuffer Data buffer to which message is to be printed
 * @param bufferLength Available memory in data buffer
 * @param dcmmData Struct containing relevant DCMM data
 * @param debug Flag for enabling debugging
 * @return Number of bytes written to buffer, or -1 in case of error 
 */
ssize_t decodeDCMMMessage(const char * dcmmDataBuffer,
		const size_t bufferLength,
		RemoteControlManoeuvreMessageType* dcmmData,
		const char debug) {

	RCMMType DCMMData;
	const char *p = dcmmDataBuffer;
	ssize_t retval = MESSAGE_OK;
	uint16_t valueID = 0;
	uint16_t contentLength = 0;
	ssize_t expectedContentLength = 0;
	
	if (dcmmDataBuffer == NULL || dcmmData == NULL) {
		errno = EINVAL;
		fprintf(stderr, "Input pointers to DCMM parsing function cannot be null\n");
		return ISO_FUNCTION_ERROR;
	}

	memset(&DCMMData, 0, sizeof (DCMMData));
	memset(dcmmData, 0, sizeof(*dcmmData));

	// Decode ISO header
	if ((retval = decodeISOHeader(p, bufferLength, &DCMMData.header, debug)) != MESSAGE_OK) {
		return retval;
	}
	p += sizeof (DCMMData.header);

	// If message is not a RCMM message, generate an error
	if (DCMMData.header.MessageIdU16 != MESSAGE_ID_VENDOR_SPECIFIC_ASTAZERO_DCMM) {
		fprintf(stderr, "Attempted to pass non-DCMM message into DCMM parsing function\n");
		return MESSAGE_TYPE_ERROR;
	}

	if (DCMMData.header.MessageLengthU32 > sizeof (RCMMType) - sizeof (HeaderType) - sizeof (FooterType)) {
		fprintf(stderr, "DCMM message exceeds expected message length\n");
		return MESSAGE_LENGTH_ERROR;
	}

	while (p - dcmmDataBuffer < DCMMData.header.MessageLengthU32 + sizeof (HeaderType)) {
		memcpy(&valueID, p, sizeof (valueID));
		p += sizeof (valueID);
		memcpy(&contentLength, p, sizeof (contentLength));
		p += sizeof (contentLength);
		valueID = le16toh(valueID);
		contentLength = le16toh(contentLength);
		switch (valueID) {
		case VALUE_ID_RCMM_CONTROL_STATUS:
			memcpy(&DCMMData.controlStatus, p, sizeof (DCMMData.controlStatus));
			DCMMData.controlStatusValueID = valueID;
			DCMMData.controlStatusContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.controlStatus);
			break;
		case VALUE_ID_RCMM_SPEED_METER_PER_SECOND:
			memcpy(&DCMMData.speed, p, sizeof (DCMMData.speed));
			DCMMData.speed = (int16_t)le16toh (DCMMData.speed);
			DCMMData.speedValueID = valueID;
			DCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.speed);
			break;
		case VALUE_ID_RCMM_STEERING_ANGLE:
			memcpy(&DCMMData.steering, p, sizeof (DCMMData.steering));
			DCMMData.steering = (int16_t)le16toh (DCMMData.steering);
			DCMMData.steeringValueID = valueID;
			DCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.steering);
			break;
		case VALUE_ID_RCMM_STEERING_PERCENTAGE:
			memcpy(&DCMMData.steering, p, sizeof (DCMMData.steering));
			DCMMData.steering = (int16_t)le16toh (DCMMData.steering); 
			DCMMData.steeringValueID = valueID;
			DCMMData.steeringContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.steering);
			break;
		case VALUE_ID_RCMM_SPEED_PERCENTAGE:
			memcpy(&DCMMData.speed, p, sizeof (DCMMData.speed));
			DCMMData.speed = (int16_t)le16toh (DCMMData.speed);
			DCMMData.speedValueID = valueID;
			DCMMData.speedContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.speed);
			break;
		case VALUE_ID_RCMM_CONTROL:
			memcpy(&DCMMData.command, p, sizeof (DCMMData.command));
			DCMMData.commandValueID = valueID;
			DCMMData.commandContentLength = contentLength;
			expectedContentLength = sizeof (DCMMData.command);
			break;
		default:
			fprintf(stderr, "Value ID 0x%x does not match any known DCMM value IDs\n", valueID);
			return MESSAGE_VALUE_ID_ERROR;
		}

		p += contentLength;
		if (contentLength != expectedContentLength) {
			fprintf(stderr, "Content length %u for value ID 0x%x does not match the expected %ld\n",
					contentLength, valueID, expectedContentLength);
			return MESSAGE_LENGTH_ERROR;
		}
	}

	// Decode footer
	if ((retval =
		 decodeISOFooter(p, bufferLength - (size_t) (p - dcmmDataBuffer), &DCMMData.footer,
						 debug)) != MESSAGE_OK) {
		fprintf(stderr, "Error decoding DCMM footer\n");
		return retval;
	}

	p += sizeof (DCMMData.footer);
	if ((retval = verifyChecksum(dcmmDataBuffer, DCMMData.header.MessageLengthU32 + sizeof (HeaderType),
								 DCMMData.footer.Crc, debug)) == MESSAGE_CRC_ERROR) {
		fprintf(stderr, "DCMM checksum error\n");
		return retval;
	}

	if (debug) {
		printf("DCMM message:\n");
		printContent(DCMMData.controlStatusValueID, DCMMData.controlStatusContentLength,
					 &DCMMData.controlStatus, &RCMMControlStatusDescription);
		if (DCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_ANGLE)
			printContent(DCMMData.steeringValueID, DCMMData.steeringContentLength,
					 	&DCMMData.steering, &RCMMSteeringDescriptionDeg);
		else if (DCMMData.steeringValueID == VALUE_ID_RCMM_STEERING_PERCENTAGE)
			printContent(DCMMData.steeringValueID, DCMMData.steeringContentLength,
					 	&DCMMData.steering, &RCMMSteeringDescriptionPct);
		if (DCMMData.speedValueID == VALUE_ID_RCMM_SPEED_METER_PER_SECOND)
			printContent(DCMMData.speedValueID, DCMMData.speedContentLength,
					 	&DCMMData.speed, &RCMMSpeedDescription_m_s);
		else if (DCMMData.speedValueID == VALUE_ID_RCMM_SPEED_PERCENTAGE)
			printContent(DCMMData.speedValueID, DCMMData.speedContentLength,
				&DCMMData.speed, &RCMMSpeedDescriptionPct);
	}

	retval = convertRCMMToHostRepresentation(&DCMMData, dcmmData);

	return retval < 0 ? retval : p - dcmmDataBuffer;
	
}
