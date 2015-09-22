window.rsdNamespace = window.rsdNamespace || { };

rsdNamespace.connection;

rsdNamespace.StartListening = function( port ) {

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

    rsdNamespace.connection = new WebSocket( 'ws://localhost:' + port );

    // When the connection is open, send some data to the server
    rsdNamespace.connection.onopen = function () {

        // rsdNamespace.connection.send( 'Ping' ); // Send the message 'Ping' to the server

        // Request information on the Mobile Platform's location in every second
        setInterval( function () {

            messageOut = {
                "messageType":"location_request",
                "data": {}
            }

            rsdNamespace.connection.send( JSON.stringify( messageOut ) );

        }, 1000);

        // Send information on the remote's status
        setInterval( function () {

            messageOut = {
                "messageType":"remote_update",
                "data":{
                    "left": rsdNamespace.activeButtonLeft,
                    "right": rsdNamespace.activeButtonRight,
                    "ena": rsdNamespace.actuationEnable
                }
            }

            rsdNamespace.connection.send( JSON.stringify( messageOut ) );

        }, 50);

    };

    // Log errors
    rsdNamespace.connection.onerror = function ( error ) {

        $('#availability').css( 'background-color', 'red' );
        console.log('WebSocket Error ' + error );

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

};
