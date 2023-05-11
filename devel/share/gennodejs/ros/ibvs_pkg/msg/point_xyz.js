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

class point_xyz {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.p1_x = null;
      this.p1_y = null;
      this.p1_z = null;
      this.p2_x = null;
      this.p2_y = null;
      this.p2_z = null;
      this.p3_x = null;
      this.p3_y = null;
      this.p3_z = null;
      this.p4_x = null;
      this.p4_y = null;
      this.p4_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('p1_x')) {
        this.p1_x = initObj.p1_x
      }
      else {
        this.p1_x = 0.0;
      }
      if (initObj.hasOwnProperty('p1_y')) {
        this.p1_y = initObj.p1_y
      }
      else {
        this.p1_y = 0.0;
      }
      if (initObj.hasOwnProperty('p1_z')) {
        this.p1_z = initObj.p1_z
      }
      else {
        this.p1_z = 0.0;
      }
      if (initObj.hasOwnProperty('p2_x')) {
        this.p2_x = initObj.p2_x
      }
      else {
        this.p2_x = 0.0;
      }
      if (initObj.hasOwnProperty('p2_y')) {
        this.p2_y = initObj.p2_y
      }
      else {
        this.p2_y = 0.0;
      }
      if (initObj.hasOwnProperty('p2_z')) {
        this.p2_z = initObj.p2_z
      }
      else {
        this.p2_z = 0.0;
      }
      if (initObj.hasOwnProperty('p3_x')) {
        this.p3_x = initObj.p3_x
      }
      else {
        this.p3_x = 0.0;
      }
      if (initObj.hasOwnProperty('p3_y')) {
        this.p3_y = initObj.p3_y
      }
      else {
        this.p3_y = 0.0;
      }
      if (initObj.hasOwnProperty('p3_z')) {
        this.p3_z = initObj.p3_z
      }
      else {
        this.p3_z = 0.0;
      }
      if (initObj.hasOwnProperty('p4_x')) {
        this.p4_x = initObj.p4_x
      }
      else {
        this.p4_x = 0.0;
      }
      if (initObj.hasOwnProperty('p4_y')) {
        this.p4_y = initObj.p4_y
      }
      else {
        this.p4_y = 0.0;
      }
      if (initObj.hasOwnProperty('p4_z')) {
        this.p4_z = initObj.p4_z
      }
      else {
        this.p4_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type point_xyz
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [p1_x]
    bufferOffset = _serializer.float32(obj.p1_x, buffer, bufferOffset);
    // Serialize message field [p1_y]
    bufferOffset = _serializer.float32(obj.p1_y, buffer, bufferOffset);
    // Serialize message field [p1_z]
    bufferOffset = _serializer.float32(obj.p1_z, buffer, bufferOffset);
    // Serialize message field [p2_x]
    bufferOffset = _serializer.float32(obj.p2_x, buffer, bufferOffset);
    // Serialize message field [p2_y]
    bufferOffset = _serializer.float32(obj.p2_y, buffer, bufferOffset);
    // Serialize message field [p2_z]
    bufferOffset = _serializer.float32(obj.p2_z, buffer, bufferOffset);
    // Serialize message field [p3_x]
    bufferOffset = _serializer.float32(obj.p3_x, buffer, bufferOffset);
    // Serialize message field [p3_y]
    bufferOffset = _serializer.float32(obj.p3_y, buffer, bufferOffset);
    // Serialize message field [p3_z]
    bufferOffset = _serializer.float32(obj.p3_z, buffer, bufferOffset);
    // Serialize message field [p4_x]
    bufferOffset = _serializer.float32(obj.p4_x, buffer, bufferOffset);
    // Serialize message field [p4_y]
    bufferOffset = _serializer.float32(obj.p4_y, buffer, bufferOffset);
    // Serialize message field [p4_z]
    bufferOffset = _serializer.float32(obj.p4_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type point_xyz
    let len;
    let data = new point_xyz(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [p1_x]
    data.p1_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p1_y]
    data.p1_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p1_z]
    data.p1_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p2_x]
    data.p2_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p2_y]
    data.p2_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p2_z]
    data.p2_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p3_x]
    data.p3_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p3_y]
    data.p3_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p3_z]
    data.p3_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p4_x]
    data.p4_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p4_y]
    data.p4_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [p4_z]
    data.p4_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ibvs_pkg/point_xyz';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3fec7e78a909523b59777fdf937fe045';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    float32 p1_x
    float32 p1_y
    float32 p1_z
    float32 p2_x
    float32 p2_y
    float32 p2_z
    float32 p3_x
    float32 p3_y
    float32 p3_z
    float32 p4_x
    float32 p4_y
    float32 p4_z
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
    const resolved = new point_xyz(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.p1_x !== undefined) {
      resolved.p1_x = msg.p1_x;
    }
    else {
      resolved.p1_x = 0.0
    }

    if (msg.p1_y !== undefined) {
      resolved.p1_y = msg.p1_y;
    }
    else {
      resolved.p1_y = 0.0
    }

    if (msg.p1_z !== undefined) {
      resolved.p1_z = msg.p1_z;
    }
    else {
      resolved.p1_z = 0.0
    }

    if (msg.p2_x !== undefined) {
      resolved.p2_x = msg.p2_x;
    }
    else {
      resolved.p2_x = 0.0
    }

    if (msg.p2_y !== undefined) {
      resolved.p2_y = msg.p2_y;
    }
    else {
      resolved.p2_y = 0.0
    }

    if (msg.p2_z !== undefined) {
      resolved.p2_z = msg.p2_z;
    }
    else {
      resolved.p2_z = 0.0
    }

    if (msg.p3_x !== undefined) {
      resolved.p3_x = msg.p3_x;
    }
    else {
      resolved.p3_x = 0.0
    }

    if (msg.p3_y !== undefined) {
      resolved.p3_y = msg.p3_y;
    }
    else {
      resolved.p3_y = 0.0
    }

    if (msg.p3_z !== undefined) {
      resolved.p3_z = msg.p3_z;
    }
    else {
      resolved.p3_z = 0.0
    }

    if (msg.p4_x !== undefined) {
      resolved.p4_x = msg.p4_x;
    }
    else {
      resolved.p4_x = 0.0
    }

    if (msg.p4_y !== undefined) {
      resolved.p4_y = msg.p4_y;
    }
    else {
      resolved.p4_y = 0.0
    }

    if (msg.p4_z !== undefined) {
      resolved.p4_z = msg.p4_z;
    }
    else {
      resolved.p4_z = 0.0
    }

    return resolved;
    }
};

module.exports = point_xyz;
