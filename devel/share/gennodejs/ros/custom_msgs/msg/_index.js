
"use strict";

let FlatTarget = require('./FlatTarget.js');
let CircleRecog = require('./CircleRecog.js');
let depthRecog = require('./depthRecog.js');
let ImgRecog = require('./ImgRecog.js');
let ControlCommand = require('./ControlCommand.js');
let PurposeWp = require('./PurposeWp.js');
let ParkingRecog = require('./ParkingRecog.js');

module.exports = {
  FlatTarget: FlatTarget,
  CircleRecog: CircleRecog,
  depthRecog: depthRecog,
  ImgRecog: ImgRecog,
  ControlCommand: ControlCommand,
  PurposeWp: PurposeWp,
  ParkingRecog: ParkingRecog,
};
