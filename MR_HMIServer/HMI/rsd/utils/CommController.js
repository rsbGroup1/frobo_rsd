window.rsdNamespace = window.rsdNamespace || { };

rsdNamespace.connection;

rsdNamespace.StartListening = function() {

    // $('#main_title').text( port );

    //create a new WebSocket object.
    // var ws = new WebSocket( "ws://localhost:" + port );
    // ws.onopen = function( evt ) {
    //
    //     ws.send('client> Hello!');
    //     console.log( 'WS started listening on port: ' + port );
    //
    // }; //on open event
    // ws.onclose = function( evt ) { console.log( 'WS has been closed.' ); }; //on close event
    // ws.onmessage = function( evt ) { console.log( evt.data ); }; //on message event
    // ws.onerror = function( evt ) { console.log( evt.data ); }; //on error event

    rsdNamespace.connection = new WebSocket( 'ws://' + rsdNamespace.commAddr + ':' + rsdNamespace.commPort );

    // When the connection is open, send some data to the server
    rsdNamespace.connection.onopen = function () {

        // Request information on the Mobile Platform's location in every second
        rsdNamespace.locationRequestScheduler = setInterval( function () {

            messageOut = {
                "messageType":"location_request",
                "data": {}
            }

            rsdNamespace.connection.send( JSON.stringify( messageOut ) );

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

        }, 200);

    };

    // Log messages from the server
    rsdNamespace.connection.onmessage = function ( e ) {

        var messageIn = $.parseJSON( e.data );
        console.log( 'Server: ' + e.data );

        switch( messageIn.messageType ) {
            case 'location_response':
                if( rsdNamespace.zoneSelected != messageIn.data ) {

                    rsdNamespace.ToggleHighlighting( rsdNamespace.zoneSelected );
                    rsdNamespace.ToggleHighlighting( messageIn.data );

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

        if( rsdNamespace.locationRequestScheduler ) {

            clearInterval( rsdNamespace.locationRequestScheduler );
            clearInterval( rsdNamespace.remoteUpdateScheduler );

        }

    };

};

rsdNamespace.stopListening = function() {

    if( rsdNamespace.connection ) rsdNamespace.connection.close();

};
