window.rsdNamespace = window.rsdNamespace || { };

rsdNamespace.connection;

rsdNamespace.IGNORE = "0";
rsdNamespace.MESSAGE = "1";
rsdNamespace.WARNING = "2";
rsdNamespace.ERROR   = "3";

rsdNamespace.BOX = "1";
rsdNamespace.CAMERA = "2";
rsdNamespace.ZONE_TWO = "3";
rsdNamespace.ZONE_ONE = "4";
rsdNamespace.RC_1 = "5";
rsdNamespace.RC_2 = "6";
rsdNamespace.RC_3 = "7";

rsdNamespace.SAFE = '1';
rsdNamespace.PROXIMITY_ALERT = '2';
rsdNamespace.COLLIDING = '3';

rsdNamespace.currentNodeMessage = 0;
rsdNamespace.performActionMessage = 1;

// rsdNamespace.ON = "1";
// rsdNamespace.OFF = "2";
// rsdNamespace.TIPPER= "1";
// rsdNamespace.LINE_FOLLOWING = "2";
// rsdNamespace.GPS = "3";
// rsdNamespace.COLLECTING_BRICKS = "4";
// rsdNamespace.INSIDE_BOX = "5";
// rsdNamespace.CHARGING = "6";

rsdNamespace.StartListening = function() {

    rsdNamespace.connection = new WebSocket( 'ws://' + rsdNamespace.commAddr + ':' + rsdNamespace.commPort );

    // When the connection is open, send some data to the server
    rsdNamespace.connection.onopen = function () {

        // Request information on the Mobile Platform's location in every second
        rsdNamespace.statusRequestScheduler = setInterval( function () {

            messageOut = {
                "messageType":"status_request",
                "data": {
                    "resume": rsdNamespace.availability
                }
            }

            rsdNamespace.connection.send( JSON.stringify( messageOut ) );

            //rsdNamespace.resumeSignal = false;

        }, 1000);

        // Send information on the remote's status
        rsdNamespace.remoteUpdateScheduler = setInterval( function () {

            messageOut = {
                "messageType":"remote_update",
                "data":{
                    "left": rsdNamespace.activeButtonLeft,
                    "right": rsdNamespace.activeButtonRight,
                    "ena": rsdNamespace.actuationEnable,
                    "linV": rsdNamespace.linearVelocity,
                    "angV": rsdNamespace.angularVelocity
                }
            }

            rsdNamespace.connection.send( JSON.stringify( messageOut ) );

        }, 50); // 200

    };

    // Log messages from the server
    rsdNamespace.connection.onmessage = function ( e ) 
    {
        var messageIn = $.parseJSON( e.data );
        console.log( 'Server: ' + e.data );

        switch( messageIn.messageType ) 
        {
            case 'status_response':
                var log = messageIn.data.log;
                if( log ) 
                {
                    var messages = String( log ).split(",");
                    console.log( 'Yonas: ' + messages.length );

                    if( messages.length > 2 ) 
		    {
                        for( i = 0; i < messages.length; i += 3 ) 
			{
                            // If message contains displayable content... (LOG)
                            var message_code = messages[i][0];
                            if( message_code != rsdNamespace.IGNORE ) 
			    {
                                // If there was an error,
                                /*if( message_code == rsdNamespace.ERROR ) 
				{
                                    // The SetAvailabilitySwitch method will take care of the
                                    // notification of the MES server
                                    rsdNamespace.SetAvailabilitySwitch( false );
                                }*/

                                rsdNamespace.UpdateLog( message_code, messages[i + 1], messages[i + 2] );
                            }

                            // If message contains location related information... (POSITION)
                            // var location_code = messages[i][2];
                            var location_code = messages[i][1];
                            if( location_code != rsdNamespace.IGNORE ) 
			    {
                                var location = parseInt( location_code, 10 );

                                if( rsdNamespace.zoneSelected != location ) 
				{
                                    rsdNamespace.ToggleHighlighting( rsdNamespace.zoneSelected );
                                    rsdNamespace.ToggleHighlighting( location );
                                }
                            }

                            // If message contains activity information...
                            var activity_code = messages[i].substr(2, 2);
                            if( activity_code != rsdNamespace.IGNORE ) 
			    {
                                rsdNamespace.IndicateStatus( activity_code );
                            }
                        }
                    }
                }

                break;
        }
    };

    // Log errors
    rsdNamespace.connection.onerror = function ( error ) {

        $('#availability').css( 'background-color', 'red' );
        console.log('WebSocket Error ' + error );

    };

    // Terminate schedulers upon exiting
    rsdNamespace.connection.onclose = function() {

        if( rsdNamespace.statusRequestScheduler ) 
	{
            clearInterval( rsdNamespace.statusRequestScheduler );
            clearInterval( rsdNamespace.remoteUpdateScheduler );
        }

    };

};

rsdNamespace.stopListening = function() 
{
    if( rsdNamespace.connection ) 
	rsdNamespace.connection.close();
};

rsdNamespace.sendCurrentNodeMsg = function( target ) 
{
    console.log( 'Current node: ' + document.getElementById("setCurrentNodeID").value);

    value_ = document.getElementById('setCurrentNodeID').value
    messageOut = {
        "messageType":"text_msg",
        "data": {
            "target": target,
            "msg": value_
        }
    }

    value_.value = "";
    rsdNamespace.connection.send( JSON.stringify( messageOut ) );
}

rsdNamespace.sendPerformActionMsg = function( target ) 
{
    console.log( 'Perform action: ' + document.getElementById("performActionID").value);

    value_ = document.getElementById('performActionID').value
    messageOut = {
        "messageType":"text_msg",
        "data": {
            "target": target,
            "msg": value_
        }
    }

    value_.value = "";
    rsdNamespace.connection.send( JSON.stringify( messageOut ) );
}
