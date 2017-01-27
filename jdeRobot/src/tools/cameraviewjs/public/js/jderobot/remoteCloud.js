// **********************************************************************
//
// Copyright (c) 2003-2016 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.6.3
//
// <auto-generated>
//
// Generated from file `remoteCloud.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

(function(module, require, exports)
{
    var Ice = require("ice").Ice;
    var __M = Ice.__M;
    var jderobot = require("jderobot/pointcloud").jderobot;
    var Slice = Ice.Slice;

    jderobot.remoteCloud = Slice.defineObject(
        undefined,
        Ice.Object,
        [
            jderobot.pointCloud
        ], 2,
        [
            "::Ice::Object",
            "::jderobot::pointCloud",
            "::jderobot::remoteCloud"
        ],
        -1, undefined, undefined, false);

    jderobot.remoteCloudPrx = Slice.defineProxy(Ice.ObjectPrx, jderobot.remoteCloud.ice_staticId, [
        jderobot.pointCloudPrx]);

    Slice.defineOperations(jderobot.remoteCloud, jderobot.remoteCloudPrx,
    {
        "initConfiguration": [, , , , , [3], , , , , ],
        "read": [, , , , , [7], [[3]], , , , ],
        "write": [, , , , , [3], [[7], [3]], , , , ],
        "setConfiguration": [, , , , , [3], [[3]], , , , ]
    });
    exports.jderobot = jderobot;
}
(typeof(global) !== "undefined" && typeof(global.process) !== "undefined" ? module : undefined,
 typeof(global) !== "undefined" && typeof(global.process) !== "undefined" ? require : this.Ice.__require,
 typeof(global) !== "undefined" && typeof(global.process) !== "undefined" ? exports : this));
