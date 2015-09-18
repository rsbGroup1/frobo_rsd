window.rsdNamespace = window.rsdNamespace || { };

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

    var connection = new WebSocket( 'ws://localhost:' + port );

    // When the connection is open, send some data to the server
    connection.onopen = function () {

        connection.send('Pingx'); // Send the message 'Ping' to the server

        setInterval(function () {

            connection.send('location_request');

        }, 1000);

    };

    // Log errors
    connection.onerror = function (error) {

        console.log('WebSocket Error ' + error );

    };

    // Log messages from the server
    connection.onmessage = function (e) {

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
