/*
 * See Box2D.js
 */
goog.provide('Box2D.Dynamics.b2ContactManager');

goog.require('Box2D.Collision.b2ContactPoint');
goog.require('Box2D.Collision.b2DynamicTreeBroadPhase');
goog.require('Box2D.Dynamics.b2ContactFilter');
goog.require('Box2D.Dynamics.b2ContactListener');
goog.require('Box2D.Dynamics.Contacts.b2ContactFactory');

/**
 * @param {!Box2D.Dynamics.b2World} world
 * @constructor
 */
Box2D.Dynamics.b2ContactManager = function(world) {
    this.m_world = world;
    this.m_contactCount = 0;
    this.m_contactFilter = Box2D.Dynamics.b2ContactFilter.b2_defaultFilter;
    this.m_contactListener = Box2D.Dynamics.b2ContactListener.b2_defaultListener;
    this.m_contactFactory = new Box2D.Dynamics.Contacts.b2ContactFactory();
    this.m_broadPhase = new Box2D.Collision.b2DynamicTreeBroadPhase();
};

Box2D.Dynamics.b2ContactManager.prototype.AddPair = function (fixtureA, fixtureB) {
  var bodyA = fixtureA.GetBody();
  var bodyB = fixtureB.GetBody();
  if (bodyA == bodyB) {
      return;
  }
  if (!bodyB.ShouldCollide(bodyA)) {
     return;
  }
  if (!this.m_contactFilter.ShouldCollide(fixtureA, fixtureB)) {
     return;
  }
  var edge = bodyB.GetContactList();
  while (edge) {
     if (edge.other == bodyA) {
        var fA = edge.contact.GetFixtureA();
        var fB = edge.contact.GetFixtureB();
        if (fA == fixtureA && fB == fixtureB) {
            return;
        }
        if (fA == fixtureB && fB == fixtureA) {
            return;
        }
     }
     edge = edge.next;
  }
  var c = this.m_contactFactory.Create(fixtureA, fixtureB);
  fixtureA = c.GetFixtureA();
  fixtureB = c.GetFixtureB();
  bodyA = fixtureA.m_body;
  bodyB = fixtureB.m_body;
  c.m_prev = null;
  c.m_next = this.m_world.m_contactList;
  if (this.m_world.m_contactList != null) {
     this.m_world.m_contactList.m_prev = c;
  }
  this.m_world.m_contactList = c;
  c.m_nodeA.contact = c;
  c.m_nodeA.other = bodyB;
  c.m_nodeA.prev = null;
  c.m_nodeA.next = bodyA.m_contactList;
  if (bodyA.m_contactList != null) {
     bodyA.m_contactList.prev = c.m_nodeA;
  }
  bodyA.m_contactList = c.m_nodeA;
  c.m_nodeB.contact = c;
  c.m_nodeB.other = bodyA;
  c.m_nodeB.prev = null;
  c.m_nodeB.next = bodyB.m_contactList;
  if (bodyB.m_contactList != null) {
     bodyB.m_contactList.prev = c.m_nodeB;
  }
  bodyB.m_contactList = c.m_nodeB;
  this.m_world.m_contactCount++;
  return;
};

Box2D.Dynamics.b2ContactManager.prototype.FindNewContacts = function () {
  this.m_broadPhase.UpdatePairs(Box2D.generateCallback(this, this.AddPair));
};

Box2D.Dynamics.b2ContactManager.prototype.Destroy = function (c) {
  var fixtureA = c.GetFixtureA();
  var fixtureB = c.GetFixtureB();
  var bodyA = fixtureA.GetBody();
  var bodyB = fixtureB.GetBody();
  if (c.IsTouching()) {
     this.m_contactListener.EndContact(c);
  }
  if (c.m_prev) {
     c.m_prev.m_next = c.m_next;
  }
  if (c.m_next) {
     c.m_next.m_prev = c.m_prev;
  }
  if (c == this.m_world.m_contactList) {
     this.m_world.m_contactList = c.m_next;
  }
  if (c.m_nodeA.prev) {
     c.m_nodeA.prev.next = c.m_nodeA.next;
  }
  if (c.m_nodeA.next) {
     c.m_nodeA.next.prev = c.m_nodeA.prev;
  }
  if (c.m_nodeA == bodyA.m_contactList) {
     bodyA.m_contactList = c.m_nodeA.next;
  }
  if (c.m_nodeB.prev) {
     c.m_nodeB.prev.next = c.m_nodeB.next;
  }
  if (c.m_nodeB.next) {
     c.m_nodeB.next.prev = c.m_nodeB.prev;
  }
  if (c.m_nodeB == bodyB.m_contactList) {
     bodyB.m_contactList = c.m_nodeB.next;
  }
  this.m_contactFactory.Destroy(c);
  this.m_contactCount--;
};

Box2D.Dynamics.b2ContactManager.prototype.Collide = function () {
  var c = this.m_world.m_contactList;
  while (c) {
     var fixtureA = c.GetFixtureA();
     var fixtureB = c.GetFixtureB();
     var bodyA = fixtureA.GetBody();
     var bodyB = fixtureB.GetBody();
     if (bodyA.IsAwake() == false && bodyB.IsAwake() == false) {
        c = c.GetNext();
        continue;
     }
     if (c.IsFiltering()) {
        if (bodyB.ShouldCollide(bodyA) == false) {
           var cNuke = c;
           c = cNuke.GetNext();
           this.Destroy(cNuke);
           continue;
        }
        if (this.m_contactFilter.ShouldCollide(fixtureA, fixtureB) == false) {
           cNuke = c;
           c = cNuke.GetNext();
           this.Destroy(cNuke);
           continue;
        }
        c.ClearFiltering();
     }
     var proxyA = fixtureA.m_proxy;
     var proxyB = fixtureB.m_proxy;
     var overlap = this.m_broadPhase.TestOverlap(proxyA, proxyB);
     if (overlap == false) {
        cNuke = c;
        c = cNuke.GetNext();
        this.Destroy(cNuke);
        continue;
     }
     c.Update(this.m_contactListener);
     c = c.GetNext();
  }
};

Box2D.Dynamics.b2ContactManager.s_evalCP = new Box2D.Collision.b2ContactPoint();
