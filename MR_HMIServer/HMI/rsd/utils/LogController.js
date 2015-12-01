window.rsdNamespace = window.rsdNamespace || { };

rsdNamespace.logErrorFlag = false;
rsdNamespace.logWarningFlag = false;

rsdNamespace.UpdateLog = function( type, timestamp, message ) {

    var highlighting = "";
    var type_to_display = "Info";

    if( type == 3 ) {

        type_to_display = "Error"
        highlighting = " error_message";

    } else if( type == 2 ) {

        type_to_display = "Warning"
        highlighting = " warning_message";

    }

    var entry = '<div class="log_entry' + highlighting + '">' +
        '<span class="log_title"><h5 class="log_title">' + type_to_display + '</h5> - ' + timestamp + '</span>' +
        '<p class="log_message">' + message + '</p>' +
    '</div>';

    $("#log_screen").prepend( entry );

    if( rsdNamespace.logErrorFlag ) rsdNamespace.FilterMessages( '.log_entry', 'error_message' );
    else if( rsdNamespace.logWarningFlag ) rsdNamespace.FilterMessages( '.log_entry', 'warning_message' );

    rsdNamespace.RemoveOldEntries( ".log_entry", 50 );

};

rsdNamespace.RemoveOldEntries = function( selector, threshold ) {

    $(selector).each( function( index ) {

        if( index > ( threshold - 1 ) ) $(this).remove();

    });

};

rsdNamespace.FilterMessages = function( selector, filter ) {

    if( filter.search( "error" ) ) {

        rsdNamespace.logWarningFlag = false;
        rsdNamespace.logErrorFlag = true;

    }

    if( filter.search( "warning" ) ) {

        rsdNamespace.logErrorFlag = false;
        rsdNamespace.logWarningFlag = true;

    }

    $(selector).each( function( index ) {

        if( !$(this).hasClass( filter ) ) $(this).css( "display", "none" );
        else  $(this).css( "display", "inline-block" );

    });

};

rsdNamespace.RemoveFilters = function( selector ) {

    rsdNamespace.logErrorFlag = false;
    rsdNamespace.logWarningFlag = false;

    $(selector).each( function( index ) {

        $(this).css( "display", "inline-block" );

    });

};

rsdNamespace.IndicateStatus = function( code ) 
{
    selector = "#indicator_" + code[0];
    //console.log(selector);
    //console.log(String(code));
    $(selector).toggleClass( "statusActiveNormal" );

    if( code[1] != rsdNamespace.IGNORE ) 
    {

        if( code[1] == rsdNamespace.SAFE ) {

            $('#indicator_7').removeClass( "statusActiveWarning statusActiveError" );

        } else if( code[1] == rsdNamespace.PROXIMITY_ALERT ) {

            $('#indicator_7').removeClass( "statusActiveError" ).addClass( "statusActiveWarning" );

        } else if( code[1] == rsdNamespace.COLLIDING ) {

            // If there was an error,react:
            //     The SetAvailabilitySwitch method will take care of the
            //     notification of the MES server
            //rsdNamespace.SetAvailabilitySwitch( false );

            $('#indicator_7').removeClass( "statusActiveWarning" ).addClass( "statusActiveError" );
        }

    }

};
