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

rsdNamespace.ToggleHighlighting = function( selector ) {

    var id;

    console.log( 'selector:' + selector );

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

    console.log( "id: " + id );

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

  $('#remoteBottomContent').css( 'left', '50%' ).css( 'left', '-=43px' );

};

rsdNamespace.RegisterTouchSurfaces = function() {

  // Left Controller

  // Button Left
  $('#button_l').on( "click", function() {



  });

  $('#button_l').on( "mouseover", function() {

    $('#remoteLeftLayer1').attr('style', 'display:none');
    $('#remoteLeftLayerL').attr('style', 'display:inline');

  });

  $('#button_l').on( "mouseout", function() {

    $('#remoteLeftLayer1').attr('style', 'display:inline');
    $('#remoteLeftLayerL').attr('style', 'display:none');

  });

  // Button Right
  $('#button_r').on( "click", function() {



  });

  $('#button_r').on( "mouseover", function() {

    $('#remoteLeftLayer1').attr('style', 'display:none');
    $('#remoteLeftLayerR').attr('style', 'display:inline');

  });

  $('#button_r').on( "mouseout", function() {

    $('#remoteLeftLayer1').attr('style', 'display:inline');
    $('#remoteLeftLayerR').attr('style', 'display:none');

  });
  // Button Up
  $('#button_u').on( "click", function() {



  });

  $('#button_u').on( "mouseover", function() {

    $('#remoteLeftLayer1').attr('style', 'display:none');
    $('#remoteLeftLayerU').attr('style', 'display:inline');

  });

  $('#button_u').on( "mouseout", function() {

    $('#remoteLeftLayer1').attr('style', 'display:inline');
    $('#remoteLeftLayerU').attr('style', 'display:none');

  });

  // Button Down
  $('#button_d').on( "click", function() {



  });

  $('#button_d').on( "mouseover", function() {

    $('#remoteLeftLayer1').attr('style', 'display:none');
    $('#remoteLeftLayerD').attr('style', 'display:inline');

  });

  $('#button_d').on( "mouseout", function() {

    $('#remoteLeftLayer1').attr('style', 'display:inline');
    $('#remoteLeftLayerD').attr('style', 'display:none');

  });


  // Right Controller

  // Button X
  $('#button_x').on( "click", function() {



  });

  $('#button_x').on( "mouseover", function() {

    $('#remoteRightLayer1').attr('style', 'display:none');
    $('#remoteRightLayerX').attr('style', 'display:inline');

  });

  $('#button_x').on( "mouseout", function() {

    $('#remoteRightLayer1').attr('style', 'display:inline');
    $('#remoteRightLayerX').attr('style', 'display:none');

  });

  // Button Y
  $('#button_y').on( "click", function() {



  });

  $('#button_y').on( "mouseover", function() {

    $('#remoteRightLayer1').attr('style', 'display:none');
    $('#remoteRightLayerY').attr('style', 'display:inline');

  });

  $('#button_y').on( "mouseout", function() {

    $('#remoteRightLayer1').attr('style', 'display:inline');
    $('#remoteRightLayerY').attr('style', 'display:none');

  });
  // Button A
  $('#button_a').on( "click", function() {



  });

  $('#button_a').on( "mouseover", function() {

    $('#remoteRightLayer1').attr('style', 'display:none');
    $('#remoteRightLayerA').attr('style', 'display:inline');

  });

  $('#button_a').on( "mouseout", function() {

    $('#remoteRightLayer1').attr('style', 'display:inline');
    $('#remoteRightLayerA').attr('style', 'display:none');

  });

  // Button B
  $('#button_b').on( "click", function() {



  });

  $('#button_b').on( "mouseover", function() {

    $('#remoteRightLayer1').attr('style', 'display:none');
    $('#remoteRightLayerB').attr('style', 'display:inline');

  });

  $('#button_b').on( "mouseout", function() {

    $('#remoteRightLayer1').attr('style', 'display:inline');
    $('#remoteRightLayerB').attr('style', 'display:none');

  });

  // Home button
  $('#button_home').on( "click", function() {

    rsdNamespace.ReleaseDashboard();

  });

  $('#button_home').on( "mouseover", function() {

    $('#HomeButtonInactive').attr('style', 'display:none');
    $('#HomeButtonActive').attr('style', 'display:inline');

  });

  $('#button_home').on( "mouseout", function() {

    $('#HomeButtonInactive').attr('style', 'display:inline');
    $('#HomeButtonActive').attr('style', 'display:none');

  });

};

rsdNamespace.CallDashboard = function( event ) {

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
