window.rsdNamespace = window.rsdNamespace || { };

// Color mapping
rsdNamespace.colorUnselectedTrack = 'f2f2f2';
rsdNamespace.colorUnselectedCell = 'e2b46c';
rsdNamespace.colorUnselectedText = '000000';
rsdNamespace.colorSelected = '9200ff';
rsdNamespace.colorSelectedText = 'ffffff';
rsdNamespace.colorError = 'ff0000';

// Path mapping
rsdNamespace.robotCellOne = 'robotCellOne';
rsdNamespace.robotCellTwo = 'robotCellTwo';
rsdNamespace.robotCellThree = 'robotCellThree';
rsdNamespace.trackZoneOne = 'trackZoneOne';
rsdNamespace.trackZoneTwo = 'trackZoneTwo';
rsdNamespace.cameraZone = 'cameraZone';
rsdNamespace.box = 'box';
rsdNamespace.zoneSelected = '0';

// Connection parameters
rsdNamespace.locationRequestScheduler;
rsdNamespace.remoteUpdateScheduler;
rsdNamespace.commAddr = 'localhost';
rsdNamespace.commPort = 8888;
rsdNamespace.commState = 3;

// Control filters
rsdNamespace.activeButtonLeft = 'n';
rsdNamespace.activeButtonRight = 'n';
rsdNamespace.actuationEnable = false;

// Safety
rsdNamespace.watchdog;

rsdNamespace.ToggleHighlighting = function( selector ) {

    var id;

    //console.log( 'selector:' + selector );

    switch( selector ) {

        case '0':
            id = rsdNamespace.robotCellThree;
            break;
        case '1':
            id = rsdNamespace.robotCellTwo;
            break;
        case '2':
            id = rsdNamespace.robotCellOne;
            break;
        case '3':
            id = rsdNamespace.trackZoneOne;
            break;
        case '4':
            id = rsdNamespace.trackZoneTwo;
            break;
        case '5':
            id = rsdNamespace.cameraZone;
            break;
        case '6':
            id = rsdNamespace.box;
            break;
        default:
            id = rsdNamespace.robotCellOne;
            break;

    }

    //console.log( "id: " + id );

    var style = $( '#' + id ).attr( 'style' );
    var textStyle = $( '#' + id + 'Label' ).attr( 'style' );

    if( style.search( this.colorUnselectedTrack ) > 0 ) {

        style = style.replace( this.colorUnselectedTrack, this.colorSelected );
        textStyle = textStyle.replace( this.colorUnselectedText, this.colorSelectedText );

    } else if( style.search( this.colorUnselectedCell ) > 0 ) {

        style = style.replace( this.colorUnselectedCell, this.colorSelected );
        textStyle = textStyle.replace( this.colorUnselectedText, this.colorSelectedText );

    } else {

    if( id.search( 'Cell' ) != -1 || id.search( 'box' ) != -1 ) {

        style = style.replace( this.colorSelected, this.colorUnselectedCell );

    } else {

        style = style.replace( this.colorSelected, this.colorUnselectedTrack );

    }

        textStyle = textStyle.replace( this.colorSelectedText, this.colorUnselectedText );

    }

    $( '#' + id ).attr( 'style', style );
    $( '#' + id + 'Label' ).attr( 'style', textStyle );

    rsdNamespace.zoneSelected = selector;

};

rsdNamespace.WindowResizeHandler = function( event ) {

    $('#remoteBottomContent').css( 'left', '50%' ).css( 'left', '-=200px' );

};

rsdNamespace.RegisterTouchSurfaces = function() {

    // Hostname handler
    $('#hostname_data').on( "keypress", function( e ) {

        if( e.which == 13 || e.which == 32 ) {

            var tmp = $('#hostname_data').text().split( ':' );
            if( tmp.length == 2 ) {

                rsdNamespace.commAddr = tmp[0]
                rsdNamespace.commPort = tmp[1]

                if( rsdNamespace.connection ) rsdNamespace.connection.close();
                rsdNamespace.StartListening();

            }

            return false;

        }

    });

    // Left Controller

    // Button Left

    $('#button_l').on( "mousedown touchstart", function() {
        $('#remoteLeftLayer1').attr('style', 'display:none');
        $('#remoteLeftLayerL').attr('style', 'display:inline');
        rsdNamespace.activeButtonLeft = 'l';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteLeftLayer1').attr('style', 'display:inline');
        $('#remoteLeftLayerL').attr('style', 'display:none');
        rsdNamespace.activeButtonLeft = 'n';

    });

    // Button Right
    $('#button_r').on( "mousedown touchstart", function() {

        $('#remoteLeftLayer1').attr('style', 'display:none');
        $('#remoteLeftLayerR').attr('style', 'display:inline');
        rsdNamespace.activeButtonLeft = 'r';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteLeftLayer1').attr('style', 'display:inline');
        $('#remoteLeftLayerR').attr('style', 'display:none');
        rsdNamespace.activeButtonLeft = 'n';

    });

    // Button Up
    $('#button_u').on( "mousedown touchstart", function() {

        $('#remoteLeftLayer1').attr('style', 'display:none');
        $('#remoteLeftLayerU').attr('style', 'display:inline');
      rsdNamespace.activeButtonLeft = 'u';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteLeftLayer1').attr('style', 'display:inline');
        $('#remoteLeftLayerU').attr('style', 'display:none');
        rsdNamespace.activeButtonLeft = 'n';

    });

    // Button Down
    $('#button_d').on( "mousedown touchstart", function() {

        $('#remoteLeftLayer1').attr('style', 'display:none');
        $('#remoteLeftLayerD').attr('style', 'display:inline');
        rsdNamespace.activeButtonLeft = 'd';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteLeftLayer1').attr('style', 'display:inline');
        $('#remoteLeftLayerD').attr('style', 'display:none');
        rsdNamespace.activeButtonLeft = 'n';

    });


    // Right Controller

    // Button X
    $('#button_x').on( "mousedown touchstart", function() {

        $('#remoteRightLayer1').attr('style', 'display:none');
        $('#remoteRightLayerX').attr('style', 'display:inline');
        rsdNamespace.activeButtonRight = 'x';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteRightLayer1').attr('style', 'display:inline');
        $('#remoteRightLayerX').attr('style', 'display:none');
        rsdNamespace.activeButtonRight = 'n';

    });

    // Button Y
    $('#button_y').on( "mousedown touchstart", function() {

        $('#remoteRightLayer1').attr('style', 'display:none');
        $('#remoteRightLayerY').attr('style', 'display:inline');
        rsdNamespace.activeButtonRight = 'y';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteRightLayer1').attr('style', 'display:inline');
        $('#remoteRightLayerY').attr('style', 'display:none');
        rsdNamespace.activeButtonRight = 'n';

    });
    // Button A
    $('#button_a').on( "mousedown touchstart", function() {

        $('#remoteRightLayer1').attr('style', 'display:none');
        $('#remoteRightLayerA').attr('style', 'display:inline');
        rsdNamespace.activeButtonRight = 'a';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteRightLayer1').attr('style', 'display:inline');
        $('#remoteRightLayerA').attr('style', 'display:none');
        rsdNamespace.activeButtonRight = 'n';

    });

    // Button B

    $('#button_b').on( "mousedown touchstart", function() {

        $('#remoteRightLayer1').attr('style', 'display:none');
        $('#remoteRightLayerB').attr('style', 'display:inline');
        rsdNamespace.activeButtonRight = 'b';

    }).on( "mouseup mouseout touchend", function() {

        $('#remoteRightLayer1').attr('style', 'display:inline');
        $('#remoteRightLayerB').attr('style', 'display:none');
        rsdNamespace.activeButtonRight = 'n';

    });

    // Home button
    $('#button_home').on( "click touchstart", function() {

        rsdNamespace.actuationEnable = false;

        rsdNamespace.ReleaseDashboard();

    });

    $('#button_home').on( "mouseover", function() {

        $('#HomeButtonInactive').attr('style', 'display:none');
        $('#HomeButtonActive').attr('style', 'display:inline');

    }).on( "mouseup mouseout touchend", function() {

        $('#HomeButtonInactive').attr('style', 'display:inline');
        $('#HomeButtonActive').attr('style', 'display:none');

    });

};

rsdNamespace.CallDashboard = function( event ) {

    rsdNamespace.actuationEnable = true

    // Displaying dashboard
    $('#remote').css( 'z-index', '10' );
    $('.touchSurface').css( 'z-index', '11' );
    $('#remote').animate( {opacity: '1'}, 50 );


    event.stopPropagation();

};

rsdNamespace.ReleaseDashboard = function( event ) {

    // Fade out dashboard
    $('#remote').animate( {opacity: '0'}, 50 );

    setTimeout( function() {

        $('#remote').css( 'z-index', '0' );
        $('.touchSurface').css( 'z-index', '0' );

        // $('#remoteLeftLayer1').attr('style', 'display:inline');
        // $('#remoteRightLayer1').attr('style', 'display:inline');

  }, 50);

    // $('#remoteLeftLayer1')


    event.stopPropagation();

};

rsdNamespace.SetWatchdog = function( event ) {

    rsdNamespace.watchdog = setInterval( function () {

        if( rsdNamespace.connection && ( rsdNamespace.commState != rsdNamespace.connection.readyState ) ) {

            rsdNamespace.commState = rsdNamespace.connection.readyState;

            console.log( rsdNamespace.commState );

            if( rsdNamespace.commState == 2 || rsdNamespace.commState == 3 ) {

                var tmp1 = $('#upperGlobe').attr( 'style' );
                var tmp2 = $('#lowerGlobe').attr( 'style' );
                tmp1 = tmp1.replace( 'green', 'red' );
                tmp2 = tmp2.replace( 'green', 'red' );
                $('#upperGlobe').attr( 'style', tmp1 );
                $('#lowerGlobe').attr( 'style', tmp2 );

                $('#availability').css( 'background-color', 'red' );

            } else {

                var tmp1 = $('#upperGlobe').attr( 'style' );
                var tmp2 = $('#lowerGlobe').attr( 'style' );
                tmp1 = tmp1.replace( 'red', 'green' );
                tmp2 = tmp2.replace( 'red', 'green' );
                $('#upperGlobe').attr( 'style', tmp1 );
                $('#lowerGlobe').attr( 'style', tmp2 );

                $('#availability').css( 'background-color', 'green' );

            }

        }

    }, 500);

}
