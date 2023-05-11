// Auto-generated. Do not edit!

// (in-package ibvs_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class xyzyawVel {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.Vx = null;
      this.Vy = null;
      this.Vz = null;
      this.Vyaw = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('Vx')) {
        this.Vx = initObj.Vx
      }
      else {
        this.Vx = 0.0;
      }
      if (initObj.hasOwnProperty('Vy')) {
        this.Vy = initObj.Vy
      }
      else {
        this.Vy = 0.0;
      }
      if (initObj.hasOwnProperty('Vz')) {
        this.Vz = initObj.Vz
      }
      else {
        this.Vz = 0.0;
      }
      if (initObj.hasOwnProperty('Vyaw')) {
        this.Vyaw = initObj.Vyaw
      }
      else {
        this.Vyaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type xyzyawVel
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [Vx]
    bufferOffset = _serializer.float32(obj.Vx, buffer, bufferOffset);
    // Serialize message field [Vy]
    bufferOffset = _serializer.float32(obj.Vy, buffer, bufferOffset);
    // Serialize message field [Vz]
    bufferOffset = _serializer.float32(obj.Vz, buffer, bufferOffset);
    // Serialize message field [Vyaw]
    bufferOffset = _serializer.float32(obj.Vyaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type xyzyawVel
    let len;
    let data = new xyzyawVel(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [Vx]
    data.Vx = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Vy]
    data.Vy = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Vz]
    data.Vz = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Vyaw]
    data.Vyaw = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ibvs_pkg/xyzyawVel';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cf984bc80b8e8684cced7bacc6b18f87';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float32 Vx
    float32 Vy
    float32 Vz
    float32 Vyaw
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new xyzyawVel(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.Vx !== undefined) {
      resolved.Vx = msg.Vx;
    }
    else {
      resolved.Vx = 0.0
    }

    if (msg.Vy !== undefined) {
      resolved.Vy = msg.Vy;
    }
    else {
      resolved.Vy = 0.0
    }

    if (msg.Vz !== undefined) {
      resolved.Vz = msg.Vz;
    }
    else {
      resolved.Vz = 0.0
    }

    if (msg.Vyaw !== undefined) {
      resolved.Vyaw = msg.Vyaw;
    }
    else {
      resolved.Vyaw = 0.0
    }

    return resolved;
    }
};

module.exports = xyzyawVel;
