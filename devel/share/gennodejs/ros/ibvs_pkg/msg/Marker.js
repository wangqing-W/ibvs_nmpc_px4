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

class Marker {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.marker_desired_x = null;
      this.marker_current_x = null;
      this.marker_current_y = null;
      this.marker_desired_y = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('marker_desired_x')) {
        this.marker_desired_x = initObj.marker_desired_x
      }
      else {
        this.marker_desired_x = [];
      }
      if (initObj.hasOwnProperty('marker_current_x')) {
        this.marker_current_x = initObj.marker_current_x
      }
      else {
        this.marker_current_x = [];
      }
      if (initObj.hasOwnProperty('marker_current_y')) {
        this.marker_current_y = initObj.marker_current_y
      }
      else {
        this.marker_current_y = [];
      }
      if (initObj.hasOwnProperty('marker_desired_y')) {
        this.marker_desired_y = initObj.marker_desired_y
      }
      else {
        this.marker_desired_y = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Marker
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [marker_desired_x]
    bufferOffset = _arraySerializer.int32(obj.marker_desired_x, buffer, bufferOffset, null);
    // Serialize message field [marker_current_x]
    bufferOffset = _arraySerializer.int32(obj.marker_current_x, buffer, bufferOffset, null);
    // Serialize message field [marker_current_y]
    bufferOffset = _arraySerializer.int32(obj.marker_current_y, buffer, bufferOffset, null);
    // Serialize message field [marker_desired_y]
    bufferOffset = _arraySerializer.int32(obj.marker_desired_y, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Marker
    let len;
    let data = new Marker(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [marker_desired_x]
    data.marker_desired_x = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [marker_current_x]
    data.marker_current_x = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [marker_current_y]
    data.marker_current_y = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [marker_desired_y]
    data.marker_desired_y = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.marker_desired_x.length;
    length += 4 * object.marker_current_x.length;
    length += 4 * object.marker_current_y.length;
    length += 4 * object.marker_desired_y.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ibvs_pkg/Marker';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ac084a991c16309650259dd1961a407b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    int32[] marker_desired_x
    int32[] marker_current_x
    int32[] marker_current_y
    int32[] marker_desired_y
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
    const resolved = new Marker(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.marker_desired_x !== undefined) {
      resolved.marker_desired_x = msg.marker_desired_x;
    }
    else {
      resolved.marker_desired_x = []
    }

    if (msg.marker_current_x !== undefined) {
      resolved.marker_current_x = msg.marker_current_x;
    }
    else {
      resolved.marker_current_x = []
    }

    if (msg.marker_current_y !== undefined) {
      resolved.marker_current_y = msg.marker_current_y;
    }
    else {
      resolved.marker_current_y = []
    }

    if (msg.marker_desired_y !== undefined) {
      resolved.marker_desired_y = msg.marker_desired_y;
    }
    else {
      resolved.marker_desired_y = []
    }

    return resolved;
    }
};

module.exports = Marker;
