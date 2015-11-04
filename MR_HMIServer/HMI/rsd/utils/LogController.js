window.rsdNamespace = window.rsdNamespace || { };

rsdNamespace.logErrorFlag = false;
rsdNamespace.logWarningFlag = false;

rsdNamespace.UpdateLog = function( type, timestamp, message ) {

    var highlighting = "";

    if( type === "Error" ) highlighting = " error_message";
    else if( type === "Warning" ) highlighting = " warning_message";

    var entry = '<div class="log_entry' + highlighting + '">' +
        '<span class="log_title"><h5 class="log_title">' + type + '</h5> - ' + timestamp + '</span>' +
        '<p class="log_message">' + message + '</p>' +
    '</div>';

    $("#log_screen").prepend( entry );

    if( rsdNamespace.logErrorFlag ) rsdNamespace.FilterMessages( '.log_entry', 'error_message' );
    else if( rsdNamespace.logWarningFlag ) rsdNamespace.FilterMessages( '.log_entry', 'warning_message' );

    rsdNamespace.RemoveOldEntries( ".log_entry", 30 );

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

rsdNamespace.IndicateStatus = function( code ) {

    selector = "#indicator_" + code[0];

    if( code[1] == 1 ) {

        rsdNamespace.ActivateIndicator( selector );

    } else {

        rsdNamespace.DeactivateIndicator( selector );

    }

};

rsdNamespace.ActivateIndicator = function( selector ) {

    if( !$(selector).hasClass( "statusActive" ) ) {

        $(selector).addClass( "statusActive" );

    }

};

rsdNamespace.DeactivateIndicator= function( selector ) {

    alert(selector);

    if( $(selector).hasClass( "statusActive" ) ) {

        $(selector).removeClass( "statusActive" );

    }

};
