// Auto-generated. Do not edit!

// (in-package dynamixel_sdk_examples.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SyncSetPosition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id0 = null;
      this.id1 = null;
      this.id2 = null;
      this.position0 = null;
      this.position1 = null;
      this.position2 = null;
    }
    else {
      if (initObj.hasOwnProperty('id0')) {
        this.id0 = initObj.id0
      }
      else {
        this.id0 = 0;
      }
      if (initObj.hasOwnProperty('id1')) {
        this.id1 = initObj.id1
      }
      else {
        this.id1 = 0;
      }
      if (initObj.hasOwnProperty('id2')) {
        this.id2 = initObj.id2
      }
      else {
        this.id2 = 0;
      }
      if (initObj.hasOwnProperty('position0')) {
        this.position0 = initObj.position0
      }
      else {
        this.position0 = 0;
      }
      if (initObj.hasOwnProperty('position1')) {
        this.position1 = initObj.position1
      }
      else {
        this.position1 = 0;
      }
      if (initObj.hasOwnProperty('position2')) {
        this.position2 = initObj.position2
      }
      else {
        this.position2 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SyncSetPosition
    // Serialize message field [id0]
    bufferOffset = _serializer.uint8(obj.id0, buffer, bufferOffset);
    // Serialize message field [id1]
    bufferOffset = _serializer.uint8(obj.id1, buffer, bufferOffset);
    // Serialize message field [id2]
    bufferOffset = _serializer.uint8(obj.id2, buffer, bufferOffset);
    // Serialize message field [position0]
    bufferOffset = _serializer.int32(obj.position0, buffer, bufferOffset);
    // Serialize message field [position1]
    bufferOffset = _serializer.int32(obj.position1, buffer, bufferOffset);
    // Serialize message field [position2]
    bufferOffset = _serializer.int32(obj.position2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SyncSetPosition
    let len;
    let data = new SyncSetPosition(null);
    // Deserialize message field [id0]
    data.id0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [id1]
    data.id1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [id2]
    data.id2 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [position0]
    data.position0 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [position1]
    data.position1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [position2]
    data.position2 = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 15;
  }

  static datatype() {
    // Returns string type for a message object
    return 'dynamixel_sdk_examples/SyncSetPosition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1f8428b176ccf88bc7f2c826e9b347e6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 id0
    uint8 id1
    uint8 id2
    int32 position0
    int32 position1
    int32 position2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SyncSetPosition(null);
    if (msg.id0 !== undefined) {
      resolved.id0 = msg.id0;
    }
    else {
      resolved.id0 = 0
    }

    if (msg.id1 !== undefined) {
      resolved.id1 = msg.id1;
    }
    else {
      resolved.id1 = 0
    }

    if (msg.id2 !== undefined) {
      resolved.id2 = msg.id2;
    }
    else {
      resolved.id2 = 0
    }

    if (msg.position0 !== undefined) {
      resolved.position0 = msg.position0;
    }
    else {
      resolved.position0 = 0
    }

    if (msg.position1 !== undefined) {
      resolved.position1 = msg.position1;
    }
    else {
      resolved.position1 = 0
    }

    if (msg.position2 !== undefined) {
      resolved.position2 = msg.position2;
    }
    else {
      resolved.position2 = 0
    }

    return resolved;
    }
};

module.exports = SyncSetPosition;
