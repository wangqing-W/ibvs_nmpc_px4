// Auto-generated. Do not edit!

// (in-package custom_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ImgRecog {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.detected_cnt = null;
      this.detected_num = null;
      this.point = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('detected_cnt')) {
        this.detected_cnt = initObj.detected_cnt
      }
      else {
        this.detected_cnt = 0;
      }
      if (initObj.hasOwnProperty('detected_num')) {
        this.detected_num = initObj.detected_num
      }
      else {
        this.detected_num = [];
      }
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ImgRecog
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [detected_cnt]
    bufferOffset = _serializer.uint8(obj.detected_cnt, buffer, bufferOffset);
    // Serialize message field [detected_num]
    bufferOffset = _arraySerializer.uint8(obj.detected_num, buffer, bufferOffset, null);
    // Serialize message field [point]
    // Serialize the length for message field [point]
    bufferOffset = _serializer.uint32(obj.point.length, buffer, bufferOffset);
    obj.point.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ImgRecog
    let len;
    let data = new ImgRecog(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [detected_cnt]
    data.detected_cnt = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [detected_num]
    data.detected_num = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [point]
    // Deserialize array length for message field [point]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.point = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.point[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.detected_num.length;
    length += 24 * object.point.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msgs/ImgRecog';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3cc7bc92017f3829a0d9445ce06b3944';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    uint8 detected_cnt
    uint8[] detected_num
    
    
    geometry_msgs/Point[] point
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ImgRecog(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.detected_cnt !== undefined) {
      resolved.detected_cnt = msg.detected_cnt;
    }
    else {
      resolved.detected_cnt = 0
    }

    if (msg.detected_num !== undefined) {
      resolved.detected_num = msg.detected_num;
    }
    else {
      resolved.detected_num = []
    }

    if (msg.point !== undefined) {
      resolved.point = new Array(msg.point.length);
      for (let i = 0; i < resolved.point.length; ++i) {
        resolved.point[i] = geometry_msgs.msg.Point.Resolve(msg.point[i]);
      }
    }
    else {
      resolved.point = []
    }

    return resolved;
    }
};

module.exports = ImgRecog;
