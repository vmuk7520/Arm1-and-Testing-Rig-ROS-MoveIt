// Auto-generated. Do not edit!

// (in-package dynamixel_sdk_examples.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SyncGetPositionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id0 = null;
      this.id1 = null;
      this.id2 = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SyncGetPositionRequest
    // Serialize message field [id0]
    bufferOffset = _serializer.uint8(obj.id0, buffer, bufferOffset);
    // Serialize message field [id1]
    bufferOffset = _serializer.uint8(obj.id1, buffer, bufferOffset);
    // Serialize message field [id2]
    bufferOffset = _serializer.uint8(obj.id2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SyncGetPositionRequest
    let len;
    let data = new SyncGetPositionRequest(null);
    // Deserialize message field [id0]
    data.id0 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [id1]
    data.id1 = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [id2]
    data.id2 = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamixel_sdk_examples/SyncGetPositionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c9fe514d37199be341f5e7c69881370e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 id0
    uint8 id1
    uint8 id2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SyncGetPositionRequest(null);
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

    return resolved;
    }
};

class SyncGetPositionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position0 = null;
      this.position1 = null;
      this.position2 = null;
    }
    else {
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
    // Serializes a message object of type SyncGetPositionResponse
    // Serialize message field [position0]
    bufferOffset = _serializer.int32(obj.position0, buffer, bufferOffset);
    // Serialize message field [position1]
    bufferOffset = _serializer.int32(obj.position1, buffer, bufferOffset);
    // Serialize message field [position2]
    bufferOffset = _serializer.int32(obj.position2, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SyncGetPositionResponse
    let len;
    let data = new SyncGetPositionResponse(null);
    // Deserialize message field [position0]
    data.position0 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [position1]
    data.position1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [position2]
    data.position2 = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'dynamixel_sdk_examples/SyncGetPositionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8fcbfbdfa6f4c7fab0ea23939f15dae9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new SyncGetPositionResponse(null);
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

module.exports = {
  Request: SyncGetPositionRequest,
  Response: SyncGetPositionResponse,
  md5sum() { return 'ee2edb71184f0d84eae62d21dcdef2b3'; },
  datatype() { return 'dynamixel_sdk_examples/SyncGetPosition'; }
};
