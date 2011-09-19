/*
 * See Box2D.js
 */

goog.provide('Box2D.Common.b2Settings');

Box2D.Common.b2Settings = {};

/**
 * @param {number} friction1
 * @param {number} friction2
 */
Box2D.Common.b2Settings.b2MixFriction = function (friction1, friction2) {
  if (friction1 === undefined) friction1 = 0;
  if (friction2 === undefined) friction2 = 0;
  return Math.sqrt(friction1 * friction2);
};

Box2D.Common.b2Settings.b2MixRestitution = function (restitution1, restitution2) {
  if (restitution1 === undefined) restitution1 = 0;
  if (restitution2 === undefined) restitution2 = 0;
  return restitution1 > restitution2 ? restitution1 : restitution2;
};

Box2D.Common.b2Settings.b2Assert = function (a) {
  if (!a) {
     throw "Assertion Failed";
  }
};

Box2D.Common.b2Settings.VERSION = "2.1alpha";
Box2D.Common.b2Settings.USHRT_MAX = 0x0000ffff;
Box2D.Common.b2Settings.b2_maxManifoldPoints = 2;
Box2D.Common.b2Settings.b2_aabbExtension = 0.1;
Box2D.Common.b2Settings.b2_aabbMultiplier = 2.0;
Box2D.Common.b2Settings.b2_polygonRadius = 2.0 * Box2D.Common.b2Settings.b2_linearSlop;
Box2D.Common.b2Settings.b2_linearSlop = 0.005;
Box2D.Common.b2Settings.b2_angularSlop = 2.0 / 180.0 * Math.PI;
Box2D.Common.b2Settings.b2_toiSlop = 8.0 * Box2D.Common.b2Settings.b2_linearSlop;
Box2D.Common.b2Settings.b2_maxTOIContactsPerIsland = 32;
Box2D.Common.b2Settings.b2_maxTOIJointsPerIsland = 32;
Box2D.Common.b2Settings.b2_velocityThreshold = 1.0;
Box2D.Common.b2Settings.b2_maxLinearCorrection = 0.2;
Box2D.Common.b2Settings.b2_maxAngularCorrection = 8.0 / 180.0 * Math.PI;
Box2D.Common.b2Settings.b2_maxTranslation = 2.0;
Box2D.Common.b2Settings.b2_maxTranslationSquared = Box2D.Common.b2Settings.b2_maxTranslation * Box2D.Common.b2Settings.b2_maxTranslation;
Box2D.Common.b2Settings.b2_maxRotation = 0.5 * Math.PI;
Box2D.Common.b2Settings.b2_maxRotationSquared = Box2D.Common.b2Settings.b2_maxRotation * Box2D.Common.b2Settings.b2_maxRotation;
Box2D.Common.b2Settings.b2_contactBaumgarte = 0.2;
Box2D.Common.b2Settings.b2_timeToSleep = 0.5;
Box2D.Common.b2Settings.b2_linearSleepTolerance = 0.01;
Box2D.Common.b2Settings.b2_angularSleepTolerance = 2.0 / 180.0 * Math.PI;
