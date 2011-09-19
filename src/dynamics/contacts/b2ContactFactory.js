/*
 * Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
/*
 * Original Box2D created by Erin Catto
 * http://www.gphysics.com
 * http://box2d.org/
 * 
 * Box2D was converted to Flash by Boris the Brave, Matt Bush, and John Nesky as Box2DFlash
 * http://www.box2dflash.org/
 * 
 * Box2DFlash was converted from Flash to Javascript by Uli Hecht as box2Dweb
 * http://code.google.com/p/box2dweb/
 * 
 * box2Dweb was modified to utilize Google Closure, as well as other bug fixes, optimizations, and tweaks by Illandril
 * https://github.com/illandril/box2dweb-closure
 */
 
goog.provide('Box2D.Dynamics.Contacts.b2ContactFactory');

goog.require('Box2D.Dynamics.Contacts.b2ContactRegister');
goog.require('Box2D.Dynamics.Contacts.b2CircleContact');
goog.require('Box2D.Collision.Shapes.b2CircleShape');
goog.require('Box2D.Collision.Shapes.b2PolygonShape');
goog.require('Box2D.Collision.Shapes.b2EdgeShape');
goog.require('Box2D.Dynamics.Contacts.b2PolyAndCircleContact');
goog.require('Box2D.Dynamics.Contacts.b2PolygonContact');
goog.require('Box2D.Dynamics.Contacts.b2EdgeAndCircleContact');
goog.require('Box2D.Dynamics.Contacts.b2PolyAndEdgeContact');

/**
 * @constructor
 */
Box2D.Dynamics.Contacts.b2ContactFactory = function() {
    this.m_registers = [];
    this.AddType(Box2D.Dynamics.Contacts.b2CircleContact.Create, Box2D.Collision.Shapes.b2CircleShape.NAME, Box2D.Collision.Shapes.b2CircleShape.NAME);
    this.AddType(Box2D.Dynamics.Contacts.b2PolyAndCircleContact.Create, Box2D.Collision.Shapes.b2PolygonShape.NAME, Box2D.Collision.Shapes.b2CircleShape.NAME);
    this.AddType(Box2D.Dynamics.Contacts.b2PolygonContact.Create, Box2D.Collision.Shapes.b2PolygonShape.NAME, Box2D.Collision.Shapes.b2PolygonShape.NAME);
    this.AddType(Box2D.Dynamics.Contacts.b2EdgeAndCircleContact.Create, Box2D.Collision.Shapes.b2EdgeShape.NAME, Box2D.Collision.Shapes.b2CircleShape.NAME);
    this.AddType(Box2D.Dynamics.Contacts.b2PolyAndEdgeContact.Create, Box2D.Collision.Shapes.b2PolygonShape.NAME, Box2D.Collision.Shapes.b2EdgeShape.NAME);
};

Box2D.Dynamics.Contacts.b2ContactFactory.prototype.AddType = function(createFcn, type1, type2) {
    this.m_registers[type1] = this.m_registers[type1] || [];
    this.m_registers[type1][type2] = new Box2D.Dynamics.Contacts.b2ContactRegister();
    this.m_registers[type1][type2].createFcn = createFcn;
    this.m_registers[type1][type2].primary = true;
    if (type1 != type2) {
        this.m_registers[type2] = this.m_registers[type2] || [];
        this.m_registers[type2][type1] = new Box2D.Dynamics.Contacts.b2ContactRegister();
        this.m_registers[type2][type1].createFcn = createFcn;
        this.m_registers[type2][type1].primary = false;
    }
};

Box2D.Dynamics.Contacts.b2ContactFactory.prototype.Create = function(fixtureA, fixtureB) {
    var type1 = fixtureA.GetShape().GetTypeName();
    var type2 = fixtureB.GetShape().GetTypeName();
    var reg = this.m_registers[type1][type2];
    var c;
    if (reg.pool) {
        c = reg.pool;
        reg.pool = c.m_next;
        reg.poolCount--;
        c.Reset(fixtureA, fixtureB);
        return c;
    }
    var createFcn = reg.createFcn;
    if (createFcn != null) {
        if (reg.primary) {
            c = createFcn();
            c.Reset(fixtureA, fixtureB);
            return c;
        } else {
            c = createFcn();
            c.Reset(fixtureB, fixtureA);
            return c;
        }
    } else {
        return null;
    }
};

Box2D.Dynamics.Contacts.b2ContactFactory.prototype.Destroy = function(contact) {
    if (contact.m_manifold.m_pointCount > 0) {
        contact.m_fixtureA.m_body.SetAwake(true);
        contact.m_fixtureB.m_body.SetAwake(true);
    }
    var type1 = contact.m_fixtureA.GetShape().GetTypeName();
    var type2 = contact.m_fixtureB.GetShape().GetTypeName();
    var reg = this.m_registers[type1][type2];
    if (true) {
        reg.poolCount++;
        contact.m_next = reg.pool;
        reg.pool = contact;
    }
};
