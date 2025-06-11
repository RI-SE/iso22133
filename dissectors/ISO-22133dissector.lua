-- ISO22133 protocol dissector for Wireshark
ISO_proto = Proto("ISO-22133","ISO:22133-1 Protocol")
MSG_LIST =
{[0x0001] = "TRAJ",
[0x0002] = "OSEM",
[0x0003] = "OSTM",
[0x0004] = "STRT",
[0x0005] = "HEAB",
[0x0006] = "MONR",
[0x0007] = "MONR2",
[0x0008] = "SOWM",
[0x0009] = "INFO",
[0x0011] = "TRCM",
[0x0012] = "ACCM",
[0x0013] = "TREO",
[0x0014] = "EXAC",
[0x0015] = "CATA",
[0xA100] = "OPRO",
[0xA103] = "SYPM",
[0xA104] = "MTSP"}

--Header
sync_word = ProtoField.uint16("ISO_proto.sync_word", "sync_word", base.uint16)
message_length = ProtoField.uint32("ISO_proto.message_length", "message_Length", base.DEC )
ack_req_prot_ver = ProtoField.uint8("ack_request", "ack_req", ftypes.DEC, { [0] ="noAck", [1]="ackReq"},0x80) --BOOLEAN
transmitter_id = ProtoField.uint32("ISO_proto.transmitter_id", "transmitter_id", base.uint32)
receiver_id = ProtoField.uint32("ISO_proto.receiver_id", "receiver_id", base.uint32)
message_counter = ProtoField.uint8("ISO_proto.message_counter", "message_counter", base.uint8)
message_id = ProtoField.uint16("ISO_proto.message_id", "message_id", base.HEX, MSG_LIST)


--Message
value_id = ProtoField.uint16("ISO_proto.value_id", "value_id", base.uint16)
content_length = ProtoField.uint16("ISO_proto.content_length", "content_length", base.uint16)
data = ProtoField.bytes("ISO_proto.data", "data", base.bytes)

--TRAJ (Trajectory Object Message)
trajectory_id = ProtoField.uint16("ISO_proto.trajectory_id", "trajectory_id", base.uint16)
trajectory_name = ProtoField.string("ISO_proto.trajectory_name", "trajectory_name", base.ASCII)
trajectory_version = ProtoField.uint16("ISO_proto.trajectory_version", "trajectory_version", base.uint16)
trajectory_relativeTime = ProtoField.uint32("ISO_proto.trajectory_relativeTime", "trajectory_relativeTime", base.uint32)
trajectory_name_value_id = ProtoField.uint16("ISO_proto.trajectory_name_value_id", "trajectory_name_value_id", base.uint16)
trajectory_name_content_length = ProtoField.uint16("ISO_proto.trajectory_name_content_length", "trajectory_name_content_length", base.uint16)
trajectory_info_value_id = ProtoField.uint16("ISO_proto.trajectory_info_value_id", "trajectory_info_value_id", base.uint16)
trajectory_info_content_length = ProtoField.uint16("ISO_proto.trajectory_info_content_length", "trajectory_info_content_length", base.uint16)
trajectory_info = ProtoField.uint8("ISO_proto.trajectory_info", "trajectory_info", base.uint8)
trajectory_point_value_id = ProtoField.uint16("ISO_proto.trajectory_point_value_id", "trajectory_point_value_id", base.uint16)
trajectory_point_content_length = ProtoField.uint16("ISO_proto.trajectory_point_content_length","trajectory_point_content_length",base.uint16)
relative_time = ProtoField.uint32("ISO_proto.relative_time", "relative_time", base.uint32)
x_position = ProtoField.int32("ISO_proto.x_position", "x_position", base.int32)
y_position = ProtoField.int32("ISO_proto.y_position", "y_position", base.int32)
z_position = ProtoField.int32("ISO_proto.z_position", "z_position", base.int32)
longitudinal_speed = ProtoField.uint16("ISO_proto.longitudinal_speed", "longitudinal_speed", base.uint16)
lateral_speed = ProtoField.uint16("ISO_proto.lateral_speed", "lateral_speed", base.uint16)
longitudinal_acceleration = ProtoField.uint16("ISO_proto.longitudinal_acceleration", "longitudinal_acceleration", base.uint16)
lateral_acceleration = ProtoField.uint16("ISO_proto.lateral_acceleration", "lateral_acceleration", base.uint16)
curvature = ProtoField.float("ISO_proto.curvature", "curvature", base.float32)


xpos = ProtoField.int32("ISO_proto.xpos", "xpos", base.int32)
ypos = ProtoField.int32("ISO_proto.ypos", "ypos", base.int32)
zpos = ProtoField.int32("ISO_proto.zpos", "zpos", base.int32)
yaw = ProtoField.uint16("ISO_proto.yaw", "yaw", base.uint16)
pitch = ProtoField.uint16("ISO_proto.pitch", "pitch", base.int16)
roll = ProtoField.uint16("ISO_proto.roll", "roll", base.int16)
longSpeed = ProtoField.int16("ISO_proto.longSpeed", "longSpeed", base.int16)
latSpeed = ProtoField.uint16("ISO_proto.latSpeed", "latSpeed", base.int16)
longAcc = ProtoField.uint16("ISO_proto.longAcc", "longAcc", base.int16)
latAcc = ProtoField.int16("ISO_proto.latAcc", "latAcc", base.int16)
trajectory_curvature = ProtoField.float("ISO_proto.trajectory_curvature", "trajectory_curvature", base.float32)


--OSEM (Object Setting Message)
OSEM_latitude = ProtoField.int64("ISO_proto.OSEM_latitude", "OSEM_latitude", base.int64) --48
OSEM_longitude = ProtoField.int64("ISO_proto.OSEM_longitude", "OSEM_longitude", base.int64) --48
OSEM_altitude = ProtoField.int32("ISO_proto.OSEM_altitude", "OSEM_altitude", base.int32)
DateISO8601 = ProtoField.uint32("ISO_proto.DateISO8601", "DateISO8601", base.uint32)
OSEM_rotation = ProtoField.uint16("ISO_proto.OSEM_rotation", "rotation", base.DEC)
OSEM_coordinateSystem = ProtoField.uint8("ISO_proto.OSEM_coordinateSystem", "coordinateSystem", base.DEC)
OSEM_leapSeconds = ProtoField.uint8("ISO_proto.OSEM_leapSeconds", "leapSeconds", base.DEC)
OSEM_maxYawDeviation = ProtoField.uint16("ISO_proto.OSEM_maxYawDeviation", "maxYawDeviation", base.DEC)
OSEM_maxPositionError = ProtoField.uint16("ISO_proto.OSEM_maxPositionError", "maxPositionError", base.DEC)
OSEM_heabTimeout = ProtoField.uint16("ISO_proto.OSEM_heabTimeout", "heabTimeout", base.DEC)
OSEM_testMode = ProtoField.uint8("ISO_proto.OSEM_testMode", "testMode", base.DEC)
OSEM_monrRate = ProtoField.uint8("ISO_proto.OSEM_monrRate", "monrRate", base.DEC)
OSEM_monr2Rate = ProtoField.uint8("ISO_proto.OSEM_monr2Rate", "monr2Rate", base.DEC)
OSEM_heabRate = ProtoField.uint8("ISO_proto.OSEM_heabRate", "heabRate", base.DEC)
OSEM_maxMessageLength = ProtoField.uint32("ISO_proto.OSEM_maxMessageLength", "maxMessageLength", base.DEC)
OSEM_timeServerIP = ProtoField.uint32("ISO_proto.OSEM_timeServerIP", "TimeServer IP", base.DEC)
OSEM_timeServerPort = ProtoField.uint16("ISO_proto.OSEM_timeServerPort", "TimeServer Port", base.DEC)
OSEM_deviceID = ProtoField.uint32("ISO_proto.OSEM_deviceID", "Device ID", base.DEC)
OSEM_subDeviceID = ProtoField.uint32("ISO_proto.OSEM_subDeviceID", "Sub Device ID", base.DEC)
OSEM_systemControlCenterID = ProtoField.uint32("ISO_proto.OSEM_systemControlCenterID", "Control Center ID", base.DEC)
GPSWeek = ProtoField.uint16("ISO_proto.GPSWeek", "GPSWeek", base.uint16)
--GPSSecondOfWeek is shared in HEAB and other messages
GPSSecondOfWeek = ProtoField.uint32("ISO_proto.GPSSecondOfWeek","GPSSecondOfWeek", base.uint32 )
GPSQuarterMillisOfWeek = ProtoField.uint32("ISO_proto.GPSQuarterMillisOfWeek","GPSQuarterMillisOfWeek", base.uint32 )
OSEM_maxWayDeviation = ProtoField.uint16("ISO_proto.OSEM_maxWayDeviation","OSEM_maxWayDeviation", base.uint16)
OSEM_maxLateralDeviation = ProtoField.uint16("ISO_proto.OSEM_maxLateralDeviation","OSEM_maxLateralDeviation", base.uint16)
OSEM_minPosAccuracy = ProtoField.uint16("ISO_proto.OSEM_minPosAccuracy","OSEM_minPosAccuracy", base.uint16, {[0]="noAccuracyRequired",[65535] = "unavailable"})

--OSTM (Object State Change Request Message)
OSTM_StateChangeRequest = ProtoField.uint32("ISO_proto.StateChangeRequest","StateChangeRequest", base.DEC, {[2]="arm", [3]="disarm",[6] = "remoteDriving"})

--STRT (Start Message)
StartTime = ProtoField.uint32("ISO_proto.StartTime", "StartTime", base.uint32)
GPSWeekValueID = ProtoField.uint16("ISO_proto.GPSWeekValueID", "GPSWeekValueID", base.uint16)
GPSWeekContentLength =ProtoField.uint16("ISO_proto.GPSWeekContentLength", "GPSWeekContentLength", base.uint16)
--GPSWeek is shared in OSEM and other messages

--HEAB (Heartbeat Message)
--GPSSecondOfWeek is found in OSEM message
HEAB_ControlCenterStatus = ProtoField.uint8 ("HEAB_ControlCenterStatus","ControlCenterStatus", base.uint8,{ [0] = "init", [1] = "ready", [2] = "abort", [3] = "running", [4] = "testDone", [5] = "normalStop" })

-- MONR (Monitor Message)
--GPSSecondOfWeek is found in OSEM message
--- XYZ, heading and long lat acc etc shared with traj
MONR_driveDirection = ProtoField.uint8 ("MONR_driveDirection","driveDirection", base.uint8, { [0]= "forward", [1] = "backward", [2] = "driveDirection unavailable"})
MONR_objectState = ProtoField.uint8 ("MONR_objectState","objectState", base.uint8, { [0] = "off",[1] = "init", [2] = "armed", [3] = "disarmed", [4] = "running", [5] = "postrun", [6] = "remoteControlled" })
MONR_readyToArm = ProtoField.uint8 ("MONR_readyToArm","readyToArm", base.uint8, {[0]="notReady", [2] = "readyToARM", [3] = "unavailable"})
MONR_ObjectErrorStatus = ProtoField.uint8 ("MONR_ObjectErrorStatus","ObjectErrorStatus", base.uint8)
MONR_ObjectErrorCode = ProtoField.uint16 ("MONR_ObjectErrorCode","ObjectErrorCode", base.uint16)

--SOWM (GPS Second Of Week Roll Over Message )
--DateISO8601 is shared with other messages
--GPSWeek is shared in OSEM and other messages

--INFO message not yet defined. See ISO 22133-1

--TRCM (Trigger Configuration Message)
triggerID = ProtoField.uint16("ISO_proto.TriggerID","triggerID", base.uint16 )
triggerType = ProtoField.uint16("ISO_proto.triggerType","triggerType", base.uint16 )
triggerTypeParam1 = ProtoField.uint32("ISO_proto.triggerTypeParam","triggerTypeParameter1", base.uint32 )
triggerTypeParam2 = ProtoField.uint32("ISO_proto.triggerTypeParam","triggerTypeParameter2", base.uint32 )
triggerTypeParam3 = ProtoField.uint32("ISO_proto.triggerTypeParam","triggerTypeParameter3", base.uint32 )

--ACCM (Action Configuration Message)
actionID = ProtoField.uint16("ISO_proto.actionID","actionID", base.uint16 )
actionType = ProtoField.uint16("ISO_proto.actionType","actionType", base.uint16 )
actionTypeParam1 = ProtoField.uint32("ISO_proto.actionTypeParam","actionTypeParameter1", base.uint32 )
actionTypeParam2 = ProtoField.uint32("ISO_proto.actionTypeParam","actionTypeParameter2", base.uint32 )
actionTypeParam3 = ProtoField.uint32("ISO_proto.actionTypeParam","actionTypeParameter3", base.uint32 )

--TREO (Trigger Event Occured Message)
  --- triggerID is shared with TRCM message. timestamp/GPSSecondOfWeek is shared with other messages.

-- EXAC (Execute Action Message)
  --- actionID and timestamp is shared with other messages.

-- CATA (Cancel Trigger&Action Message)
  --- actionID and triggerID is shared with other messages.

--Footer
crc = ProtoField.uint16("ISO_proto.crc", "crc", base.uint16)

ISO_proto.fields = { --- HEADER
                      sync_word,
                      message_length,
                      ack_req_prot_ver,
                      transmitter_id,
                      receiver_id,
                      message_counter,
                      message_id,
                      --- message
                      value_id,
                      content_length,
                      data,

                      --- TRAJ
                      trajectory_id,
                      trajectory_name,
                      trajectory_version,
                      trajectory_relativeTime,
                      trajectory_name_value_id,
                      trajectory_name_content_length,
                      trajectory_info_value_id,
                      trajectory_info_content_length,
                      trajectory_info,
                      trajectory_point_value_id,
                      trajectory_point_content_length,
                      relative_time,
                      x_position,
                      y_position,
                      z_position,
                      lateral_speed,
                      longitudinal_speed,
                      lateral_acceleration,
                      longitudinal_acceleration,
                      curvature,
                      xpos,
                      ypos,
                      zpos,
                      yaw,
                      pitch,
                      roll,
                      longSpeed,
                      latSpeed,
                      longAcc,
                      latAcc,
                      trajectory_curvature,

                      -- STRT
                      StartTime,
                      GPSWeekValueID,
                      GPSWeekContentLength,

                      --- OSEM
                      OSEM_deviceID,
                      OSEM_subDeviceID,
                      OSEM_systemControlCenterID,
                      OSEM_latitude,
                      OSEM_longitude,
                      OSEM_altitude,
                      OSEM_rotation,
                      OSEM_coordinateSystem,
                      OSEM_leapSeconds,
                      OSEM_maxYawDeviation,
                      OSEM_maxPositionError,
                      OSEM_heabTimeout,
                      OSEM_testMode,
                      OSEM_monrRate,
                      OSEM_monr2Rate,
                      OSEM_heabRate,
                      OSEM_maxMessageLength,
                      OSEM_timeServerIP,
                      OSEM_timeServerPort,
                      DateISO8601,
                      GPSWeek,
                      GPSSecondOfWeek,
                      OSEM_maxWayDeviation,
                      OSEM_maxLateralDeviation,
                      OSEM_minPosAccuracy,

                      --- OSTM (Object State Change Request Message)
                      OSTM_StateChangeRequest,


                      --- HEAB (Hearbeat Message)
                      HEAB_ControlCenterStatus,

                      --- MONR (Monitor Message)
                      GPSQuarterMillisOfWeek,
                      MONR_driveDirection,
                      MONR_objectState,
                      MONR_readyToArm,
                      MONR_ObjectErrorStatus,
                      MONR_ObjectErrorCode,

                      --- MONR2
                      --- SOWM
                      --- INFO

                      --- TRCM (Trigger Configuration Message)
                      triggerID,
                      triggerType,
                      triggerTypeParam1,
                      triggerTypeParam2,
                      triggerTypeParam3,

                      --- ACCM (Action Configuration Message)
                      actionID,
                      actionType,
                      actionTypeParam1,
                      actionTypeParam2,
                      actionTypeParam3,

                      --- TREO (Trigger Event Occured Message)
                      -- included from other messages

                      --- EXAC (Execute Action Message)
                      -- included from other messages

                      --- CATA (Cancel Trigger&Action Message)
                      -- included from other messages

      		      --- Footer
      		      crc
 }

-- create a function to dissect it
function ISO_proto.dissector(buffer,pinfo,tree)
  length = buffer():len()

  if length == 0 then return end

  --- DISSECT ISO22133
  if buffer(0,2):bytes() == ByteArray.new("7f7e") then
    pinfo.cols.protocol = "ISO 22133-1 Protocol"

    local subtree = tree:add(ISO_proto, buffer(), "Message Header")
    subtree:add_le(message_length, buffer(2,4))
    subtree:add_le(ack_req_prot_ver, buffer(6,1))
    subtree:add_le(transmitter_id, buffer(7,4))
    subtree:add_le(receiver_id, buffer(11,4))
    subtree:add_le(message_counter, buffer(15,1))
    subtree:add_le(message_id, buffer(16,2))

    local subtree = tree:add(ISO_proto, buffer(), "Message Content")
    if buffer(18,2):bytes() == ByteArray.new("2000") then --osem is special..
      subtree:add_le(value_id, 0020)
      subtree:add_le(content_length, buffer():len()-4-20)
      subtree:add_le(data, buffer(22, buffer():len()-20-4))
    else
      subtree:add_le(value_id, buffer(18,2))
      subtree:add_le(content_length, buffer(20,2))
      subtree:add_le(data, buffer(22, buffer():len()-20-4))
    end

    if buffer(18,2):bytes() == ByteArray.new("0101") then
        if buffer:len() < buffer(2,4):le_uint() then
          pinfo.desegment_len = DESEGMENT_ONE_MORE_SEGMENT
        else
          pinfo.cols.protocol = "TRAJ"
          local subtree = tree:add(ISO_proto, buffer(), "TRAJ")
          subtree:add_le(trajectory_id, buffer(22,2))
          subtree:add_le(trajectory_name_value_id, buffer(24,2))
          subtree:add_le(trajectory_name_content_length, buffer(26,2))
          subtree:add_le(trajectory_name, buffer(28,64)) --, ENC_STRING)
          subtree:add_le(trajectory_info_value_id, buffer(92,2))
          subtree:add_le(trajectory_info_content_length, buffer(94,2))
          subtree:add_le(trajectory_info, buffer(96,1))
          local start_byte = 97
          local traj_point_size = 34
          local num_points = math.floor((buffer:len()-18-2)/traj_point_size)-2
          for i = 0,num_points-1,1
          do
            --local subtree = tree:add(ISO_proto, buffer(), "TRAJ Point")
            subtree:add_le(trajectory_point_value_id, buffer(start_byte + traj_point_size * i,2))
            subtree:add_le(trajectory_point_content_length, buffer((start_byte + traj_point_size*i)+2,2))
            subtree:add_le(relative_time, buffer((start_byte+ traj_point_size*i)+4,4))
            subtree:add_le(x_position, buffer((start_byte+traj_point_size*i)+8,4))
            subtree:add_le(y_position, buffer((start_byte+traj_point_size*i)+12,4))
            subtree:add_le(z_position, buffer((start_byte+traj_point_size*i)+16,4))
            subtree:add_le(yaw, buffer((start_byte+traj_point_size*i)+20,2))
            subtree:add_le(longitudinal_speed, buffer((start_byte+traj_point_size*i)+22,2))
            subtree:add_le(lateral_speed, buffer((start_byte+traj_point_size*i)+24,2))
            subtree:add_le(longitudinal_acceleration, buffer((start_byte+traj_point_size*i)+26,2))
            subtree:add_le(lateral_acceleration, buffer((start_byte+traj_point_size*i)+28,2))
            subtree:add_le(curvature, buffer((start_byte+traj_point_size*i)+30,4))
          end
        end
    end

    if buffer(18,2):bytes() == ByteArray.new("2000") then
      pinfo.cols.protocol = "OSEM"
      local subtree = tree:add(ISO_proto, buffer(), "OSEM Data")
      subtree:add_le(OSEM_deviceID, buffer(22,4)) -- 4 bytes for device ID
      subtree:add_le(OSEM_subDeviceID, buffer(26,4)) -- 4 bytes for sub-device ID
      subtree:add_le(OSEM_systemControlCenterID, buffer(30,4)) -- 4 bytes for system control center ID

      -- skip 4 bytes
      subtree:add_le(OSEM_latitude, buffer(36,8)) -- 8 bytes for latitude
      subtree:add_le(OSEM_longitude, buffer(44,8))
      subtree:add_le(OSEM_altitude, buffer(52,4))
      subtree:add_le(OSEM_rotation, buffer(56,2)) -- 2 bytes for rotation
      subtree:add_le(OSEM_coordinateSystem, buffer(58,1)) -- 1 byte for coordinate system

      --skip 4 bytes
      subtree:add_le(DateISO8601, buffer(62, 4))
      subtree:add_le(GPSWeek, buffer(66, 2))
      subtree:add_le(GPSQuarterMillisOfWeek, buffer(68, 4))
      subtree:add_le(OSEM_leapSeconds, buffer(72, 1))

      --skip 4 bytes
      subtree:add_le(OSEM_maxWayDeviation, buffer(76,2))
      subtree:add_le(OSEM_maxLateralDeviation, buffer(78,2))
      subtree:add_le(OSEM_maxYawDeviation, buffer(80,2))
      subtree:add_le(OSEM_maxPositionError, buffer(82,2))
      subtree:add_le(OSEM_heabTimeout, buffer(84,2))
      subtree:add_le(OSEM_testMode, buffer(86,1))
      subtree:add_le(OSEM_monrRate, buffer(87,1))
      subtree:add_le(OSEM_monr2Rate, buffer(88,1))
      subtree:add_le(OSEM_heabRate, buffer(89,1))
      subtree:add_le(OSEM_maxMessageLength, buffer(90,4))

      --skip 4 bytes
      --subtree:add_le(OSEM_timeServerIP, buffer(94,4))
      --subtree:add_le(OSEM_timeServerPort, buffer(98,2))
    end

    if buffer(18,2):bytes() == ByteArray.new("6400") then
      pinfo.cols.protocol = "OSTM"
      local subtree = tree:add(ISO_proto, buffer(), "OSTM Data")
        subtree:add_le(OSTM_StateChangeRequest, buffer(22,1)) -- state
    end

    if buffer(18,2):bytes() == ByteArray.new("0200") then
      pinfo.cols.protocol = "START"
      local subtree = tree:add(ISO_proto, buffer(), "START Data")
      	  subtree:add_le(StartTime, buffer(22,4))
          subtree:add_le(GPSWeekValueID, buffer(26,2))
          subtree:add_le(GPSWeekContentLength, buffer(28,2))
          subtree:add_le(GPSWeek, buffer(30,2))

    end

    if buffer(18,2):bytes() == ByteArray.new("9000") then
      pinfo.cols.protocol = "HEAB" --Struct
      local subtree = tree:add(ISO_proto, buffer(), "HEAB Data")
        subtree:add_le(GPSQuarterMillisOfWeek, buffer(22,4))
        subtree:add_le(HEAB_ControlCenterStatus, buffer(26,1))
    end

    if buffer(18,2):bytes() == ByteArray.new("8000") then
      pinfo.cols.protocol = "MONR"
      local subtree = tree:add(ISO_proto, buffer(), "MONR Data")
      subtree:add_le(GPSQuarterMillisOfWeek, buffer(22,4))
      subtree:add_le(xpos, buffer(26,4))
      subtree:add_le(ypos, buffer(30,4))
      subtree:add_le(zpos, buffer(34,4))

      subtree:add_le(yaw, buffer(38,2))
      subtree:add_le(pitch, buffer(40,2))
      subtree:add_le(roll, buffer(42,2))

      subtree:add_le(longSpeed, buffer(44,2))
      subtree:add_le(latSpeed, buffer(46,2))
      subtree:add_le(longAcc, buffer(48,2))
      subtree:add_le(latAcc, buffer(50,2))

      subtree:add_le(MONR_driveDirection, buffer(52,1))
      subtree:add_le(MONR_objectState, buffer(53,1))
      subtree:add_le(MONR_readyToArm, buffer(54,1))
      subtree:add_le(MONR_ObjectErrorStatus, buffer(55,1))
      subtree:add_le(MONR_ObjectErrorCode, buffer(56,2))
    end

    if buffer(18,2):bytes() == ByteArray.new("07") then
      pinfo.cols.protocol = "MONR2"
    end

    if buffer(18,2):bytes() == ByteArray.new("11") then --is it DEC or HEX??
      pinfo.cols.protocol = "TRCM"
      local subtree = tree:add(ISO_proto, buffer(), "TRCM Data")
        subtree:add_le(triggerID, buffer(15,2))
        subtree:add_le(triggerType, buffer(21,2))
        subtree:add_le(triggerTypeParam1, buffer(27,4))
        subtree:add_le(triggerTypeParam2, buffer(35,4))
        subtree:add_le(triggerTypeParam3, buffer(43,4))
    end

    if buffer(18,2):bytes() == ByteArray.new("12") then --is it DEC or HEX??
      pinfo.cols.protocol = "ACCM"
      local subtree = tree:add(ISO_proto, buffer(), "ACCM Data")
        subtree:add_le(actionID, buffer(15,2))
        subtree:add_le(actionType, buffer(21,2))
        subtree:add_le(actionTypeParam1, buffer(27,4))
        subtree:add_le(actionTypeParam2, buffer(35,4))
        subtree:add_le(actionTypeParam3, buffer(43,4))
    end
    if buffer(18,2):bytes() == ByteArray.new("13") then --is it DEC or HEX?? (TREO = 0x0013)
      pinfo.cols.protocol = "TREO"
      local subtree = tree:add(ISO_proto, buffer(), "TREO Data")
        subtree:add_le(triggerID, buffer(15,2))
        subtree:add_le(GPSSecondOfWeek, buffer(21,4)) --TriggerTimeStamp
    end
    if buffer(18,2):bytes() == ByteArray.new("14") then --is it DEC or HEX??
      pinfo.cols.protocol = "EXAC"
      local subtree = tree:add(ISO_proto, buffer(), "EXAC Data")
        subtree:add_le(actionID, buffer(15,2))
        subtree:add_le(GPSSecondOfWeek, buffer(21,4)) --ExecuteTime
    end
    if buffer(18,2):bytes() == ByteArray.new("15") then --is it DEC or HEX??
      pinfo.cols.protocol = "CATA"
      local subtree = tree:add(ISO_proto, buffer(), "CATA Data")
        subtree:add_le(triggerID, buffer(15,2))
        subtree:add_le(actionID, buffer(21,2))
    end

    local subtree = tree:add(ISO_proto, buffer(), "Message Footer")
    subtree:add_le(crc, buffer(buffer:len()-2,2))

  end

end


-- load the udp.port and tcp.port tables
udp_table = DissectorTable.get("udp.port")
tcp_table = DissectorTable.get("tcp.port")

-- register our protocol with the udp and tcp dissector tables on the specified ports
udp_table:add(53240, ISO_proto)
tcp_table:add(53240, ISO_proto)
tcp_table:add(53241, ISO_proto)
