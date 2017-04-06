// *****************************************************************************
// Module..: SDK -- Software development kit for Leddar products.
//
/// \file    Main.c
///
/// \brief   Simple console program demonstrating the use of LeddarC functions.
///
// Platform: Win32, Linux
//
// Copyright (c) 2013-2015 LeddarTech Inc. All rights reserved.
// Information contained herein is or may be confidential and proprietary to
// LeddarTech inc. Prior to using any part of the software development kit
// accompanying this notice, you must accept and agree to be bound to the
// terms of the LeddarTech Inc. license agreement accompanying this file.
// *****************************************************************************

// *****************************************************************************
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// ! WARNING !
// ! To keep the example simple, scanf is used to input values and printf to
// ! output. If you use non-ASCII characters for your paths, this may not work
// ! correctly on all operating systems (for example Windows prefers to use
// ! wchar_t for this). So in a real application you may have to use other
// ! functions for I/O and conversion function to/from UTF-8.
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// *****************************************************************************

#include <stdio.h>
#include <ctype.h>
#include <string.h>

#include "LeddarC.h"
#include "LeddarProperties.h"

#define ARRAY_LEN( a )  (sizeof(a)/sizeof(a[0]))

// Global variable to avoid passing to each function.
static LeddarHandle gHandle=NULL;

// *****************************************************************************
// Function: CheckError
//
/// \brief   Check a return code and if it is not success, display an error
///          message corresponding to the code.
///
/// \param   aCode  The result code to verify.
// *****************************************************************************

static void
CheckError( int aCode )
{
    if ( aCode != LD_SUCCESS )
    {        
        printf( "LeddarC error (%d): ", aCode );

        switch ( aCode )
        {
            case LD_ACCESS_DENIED : printf("LD_ACCESS_DENIED\n" ); break;
            case LD_TIMEOUT : printf("LD_TIMEOUT\n" ); break;
            case LD_START_OF_FILE : printf("LD_START_OF_FILE\n" ); break;
            case LD_END_OF_FILE : printf("LD_END_OF_FILE\n" ); break;
            case LD_NO_RECORD : printf("LD_NO_RECORD\n" ); break;
            case LD_ALREADY_STARTED : printf("LD_ALREADY_STARTED\n" ); break;
            case LD_NO_DATA_TRANSFER : printf("LD_NO_DATA_TRANSFER\n" ); break;
            case LD_NOT_CONNECTED : printf("LD_NOT_CONNECTED\n" ); break;
            case LD_INVALID_ARGUMENT : printf("LD_INVALID_ARGUMENT\n" ); break;
            case LD_ERROR : printf("LD_ERROR\n" ); break;
            case LD_NOT_ENOUGH_SPACE : printf("LD_NOT_ENOUGH_SPACE\n" ); break;
        }
    }
}

// *****************************************************************************
// Function: WaitKey
//
/// \brief   Wait for a key to be pressed on the keyboard, pinging the sensor
///          to keep the connection alive while waiting.
///
/// \return  The character corresponding to the key pressed (converted to
///          uppercase for letters).
// *****************************************************************************

static char
WaitKey( void )
{
    // LeddarGetKey is blocking so we need to wait for a key to be pressed
    // before calling it.
    while( !LeddarKeyPressed() )
    {
        LeddarSleep( 0.5 );
    }

    return toupper( LeddarGetKey() );
}

// *****************************************************************************
// Function: DataCallback
//
/// \brief   This is the function that is called when a new set of data is
///          available. Here we simply display the first 12 detections.
///
/// \param   aHandle  This is the user data parameter that was passed to
///                   LeddarAddCallback. Here by design we know its the handle.
/// \param   aLevels  A bitmask of the data levels received in that frame.
///
/// \return  Non zero to be called again (responding 0 would remove this
///          function from the callback list).
// *****************************************************************************

static void
DataCallback( void *aHandle )
{
    LdDetection lDetections[50];
    unsigned int i, j, lCount = LeddarGetDetectionCount( aHandle );

    if ( lCount > ARRAY_LEN( lDetections ) )
    {
        lCount = ARRAY_LEN( lDetections );
    }

    LeddarGetDetections( aHandle, lDetections, ARRAY_LEN( lDetections ) );

    // When replaying a record, display the current index
    if ( LeddarGetRecordSize( gHandle ) != 0 )
    {
        printf( "%6d ", LeddarGetCurrentRecordIndex( gHandle ) );
    }

    for( i=0, j=0; (i<lCount) && (j<12); ++i )
    {
        printf( "%5.2f ", lDetections[i].mDistance );
        ++j;
    }
    puts( "" );

}

// *****************************************************************************
// Function: ReadLiveData
//
/// \brief   Start data transfer until a key is pressed and stop it (data is
///          displayed by the callback).
// *****************************************************************************

static void
ReadLiveData( void )
{
    puts( "\nPress a key to start reading data and press a key again to stop." );
    WaitKey();

    CheckError( LeddarStartDataTransfer( gHandle, LDDL_DETECTIONS ) );
    LeddarSetCallback( gHandle, DataCallback, gHandle );

    WaitKey();

    LeddarStopDataTransfer( gHandle );
    LeddarRemoveCallback( gHandle, DataCallback, gHandle );
}

// *****************************************************************************
// Function: ReplayData
//
/// \brief   Navigation through a record file to display the data (data is
///          displayed by the callback).
// *****************************************************************************

static void
ReplayData( void )
{
    puts( "\nP to go forward, O to go backward, H to return to beginning, Q to quit" );

    CheckError( LeddarStartDataTransfer( gHandle, LDDL_DETECTIONS ) );
    LeddarSetCallback( gHandle, DataCallback, gHandle );

    for(;;)
    {
        char lChoice = WaitKey();

        switch( lChoice )
        {
            case 'H':
                LeddarMoveRecordTo( gHandle, 0 );
                break;
            case 'O':
                CheckError( LeddarStepBackward( gHandle ) );
                break;
            case 'P':
                CheckError( LeddarStepForward( gHandle ) );
                break;
            case 'Q':
            case  27: // Escape
                LeddarStopDataTransfer( gHandle );
                LeddarRemoveCallback( gHandle, DataCallback, gHandle );
                return;
        }
    }
}

// *****************************************************************************
// Function: ReadConfiguration
//
/// \brief   Display some parameters of the current configuration.
// *****************************************************************************

static void
ReadConfiguration( void )
{
    double lValue;
    char   lValueStr[64];

    puts( "\nCurrent Configuration:\n" );
    CheckError( LeddarGetTextProperty( gHandle, PID_NAME, 0, lValueStr ) );
    printf( "  Device Name     : %s\n", lValueStr );
    CheckError( LeddarGetProperty( gHandle, PID_BASE_POINT_COUNT, 0, &lValue ) );
    printf( "  Base Point Count: %.0f\n", lValue );
    CheckError( LeddarGetProperty( gHandle, PID_LED_INTENSITY, 0, &lValue ) );
    printf( "  Led Intensity   : %.0f\n", lValue );
    CheckError( LeddarGetProperty( gHandle, PID_THRESHOLD_OFFSET, 0, &lValue ) );
    printf( "  Threshold offset: %.2f\n", lValue );
    
    puts( "\nPress a key to continue." );
    WaitKey();
}


// *****************************************************************************
// Function: ListSensors
//
/// \brief   List the address of all sensors available.
// *****************************************************************************

static void
ListSensors( char* aConnectyionType, char* aAddresses, unsigned int aSize )
{
    char         lConnectionType[256];
    unsigned int lIndex = 0;

    if ( aConnectyionType == NULL )
    {
        printf("\nEnter the connection type (USB or SERIAL): ");
        scanf("%255s", lConnectionType );
    }
    else
    {
        strcpy( lConnectionType, aConnectyionType );
    }

    strcpy( aAddresses, aConnectyionType );

    CheckError( LeddarListSensors( aAddresses, &aSize ) );

    printf( "\nFound %d sensors of type %s\n", aSize, lConnectionType );

    int lConnectionFoundIndex = 0;
    while( strlen( aAddresses+lIndex ) > 0 )
    {
        if ( lConnectionFoundIndex % 2 == 0 )
        {
            if ( strcmp( aConnectyionType, "USB" ) != 0 )
            {
                printf( "%d : %s", lConnectionFoundIndex/2, aAddresses + lIndex );
            }
            else
            {
                printf( "%d : ", lConnectionFoundIndex/2 );
            }
        }
        else
        {
            if ( strcmp( aConnectyionType, "USB" ) != 0 )
            {
                printf( "\t[ %s ]\n", aAddresses + lIndex );
            }
            else
            {
                printf( "[ %s ]\n", aAddresses + lIndex );
            }
        }

        lConnectionFoundIndex++;
        lIndex += strlen( aAddresses+lIndex ) + 1;
    }
}

// *****************************************************************************
// Function: ConfigurationMenu
//
/// \brief   Menu allowing the change of configuration parameters.
// *****************************************************************************

static void
ConfigurationMenu( void )
{
    while( LeddarGetConnected( gHandle ) == LD_SUCCESS )
    {
        char         lChoice;
        unsigned int lId = 0;
        unsigned int lType = 1;

        puts( "\nConfiguration Change Menu" );
        puts( "  1. Change Oversampling Exponent" );
        puts( "  2. Change Accumulation Exponent" );
        puts( "  3. Change Base Point Count" );
        puts( "  4. Change Led Intensity" );
        puts( "  5. Change Threshold Offset" );
        puts( "  6. Change Name" );
        puts( "  7. Write" );
        puts( "  8. Restore" );
        puts( "  9. Quit" );

        lChoice = WaitKey();

        switch( lChoice )
        {
            case '1':
                lId = PID_OVERSAMPLING_EXPONENT;
                break;
            case '2':
                lId = PID_ACCUMULATION_EXPONENT;
                break;
            case '3':
                lId = PID_BASE_POINT_COUNT;
                break;
            case '4':
                lId = PID_LED_INTENSITY;
                break;
            case '5':
                lId = PID_THRESHOLD_OFFSET;
                break;
            case '6':
                lId = PID_NAME;
                lType = 2;
                break;
            case '7':
                CheckError( LeddarWriteConfiguration( gHandle ) );
                break;
            case '8':
                CheckError( LeddarRestoreConfiguration( gHandle ) );
                break;
            case '9':
            case  27: // Escape
                if ( LeddarGetConfigurationModified( gHandle ) != LD_SUCCESS )
                {
                    return;
                }

                puts( "\n** Configuration modified, please Write or Restore before quitting **" );
                break;
        }

        if ( lId != 0 )
        {
            printf( "\nEnter new value: " );

            switch( lType )
            {
                case 1:
                {
                    double lValue;

                    scanf( "%lf", &lValue );
                    CheckError( LeddarSetProperty( gHandle, lId, 0, lValue ) );
                }
                    break;
                case 2:
                {
                    char lValue[64];

                    scanf( "%63s", lValue );
                    CheckError( LeddarSetTextProperty( gHandle, lId, 0, lValue ) );
                }
                    break;
            }
        }
    }
}

// *****************************************************************************
// Function: FindAddressByIndex
//
/// \brief   Get the address found by the index displayed in the function ListSensors
//
//  \param  aIndex of the sensor to find
//  \param  aAddresses List of addresses.
//
//  \return Address of the sensor, NULL if there is no address found for this index.
//
// *****************************************************************************

static char* 
FindAddressByIndex( unsigned int aIndex, char* aAddresses )
{
    unsigned int lCurrentIndex = 0;
    unsigned int lConnectionFoundIndex = 0;

    while( strlen( aAddresses+lCurrentIndex ) > 0 )
    {
        if ( ( lConnectionFoundIndex / 2 ) == aIndex )
        {
            return aAddresses + lCurrentIndex;
        }

        lConnectionFoundIndex++;
        lCurrentIndex += strlen( aAddresses+lCurrentIndex ) + 1;
    }

    return NULL;
}

// *****************************************************************************
// Function: ConnectMenu
//
/// \brief   Main menu when a live connection is made.
///
/// \param   aTrySingleUsb  If true we will try to connect to a single USB
///                         sensor by sending an empty string as the address.
///                         This works only if there is 1 and only 1 USB sensor
///                         plugged to the PC.
// *****************************************************************************

static void
ConnectMenu( int aTrySingleUsb )
{
    char lAddresses[256];
    char* lAddress = NULL;
    char lConnectionType[10];

    if ( aTrySingleUsb )
    {
        strcpy( lConnectionType, "USB" );
        ListSensors( lConnectionType, lAddresses, 255 );
        lAddress = FindAddressByIndex( 0, lAddresses );
        if ( lAddress == NULL )
        {
            return;
        }
    }
    else
    {
        // Ask for connection type and try to connect before displaying menu.
        printf( "\nEnter connection type (USB or SERIAL): " );
        scanf( "%24s", lConnectionType );    

        ListSensors( lConnectionType, lAddresses, 255 );

        unsigned int lSensorNumber = 0;
        printf("\nConnect to sensor number: ");
        scanf("%u", &lSensorNumber );
        
        lAddress = FindAddressByIndex( lSensorNumber, lAddresses );
        if ( lAddress == NULL )
        {
            printf("\nError, sensor index not valid.");
            return;
        }
    }

    if ( LeddarConnect( gHandle, lConnectionType, lAddress ) == LD_SUCCESS )
    {
        while( LeddarGetConnected( gHandle ) == LD_SUCCESS )
        {
            char lChoice;

            puts( "\n\nConnected Menu" );
            puts( "  1. Read Data" );
            puts( "  2. Read Configuration" );
            puts( "  3. Change Configuration" );
            if ( LeddarGetRecording( gHandle ) == LD_SUCCESS )
            {
                puts( "  4. Stop Recording" );
            }
            else
            {
                puts( "  4. Start Recording" );
            }
            puts( "  5. Disconnect" );

            lChoice = WaitKey();

            switch( lChoice )
            {
                case '1':
                    ReadLiveData();
                    break;
                case '2':
                    ReadConfiguration();
                    break;
                case '3':
                    ConfigurationMenu();
                    break;
                case '4':
                    if ( LeddarGetRecording( gHandle ) == LD_SUCCESS )
                    {
                        LeddarStopRecording( gHandle );
                    }
                    else
                    {
                        LeddarChar lFilename[255];
                        CheckError( LeddarStartRecording( gHandle, lFilename ) );
                    }
                    break;
                case '5':
                case  27:
                    LeddarDisconnect( gHandle );
                    return;
            }
        }
    }
    else
    {
        puts( "\nConnection failed!" );
    }    
}

// *****************************************************************************
// Function: ReplayMenu
//
/// \brief   Main menu when a replay a record file.
// *****************************************************************************

static void
ReplayMenu( void )
{
    LeddarChar lName[256];

    // Ask for file name and try to load record before display menu.
    printf( "\nEnter file name: " );
    scanf( "%255s", lName );

    if ( LeddarLoadRecord( gHandle, lName ) == LD_SUCCESS )
    {
        puts( "\nPlease wait while the record is loading..." );

        // For a big file, especially if it is on a network drive, it may
        // take a while before the replay is 100% ready. Note that you
        // can still use the replay but it will not report the complete
        // size until it is finished loading.
        while( LeddarGetRecordLoading( gHandle ) )
        {
            LeddarSleep( 0.5 );
        }

        printf( "Finished loading record of %d frames.\n", 
                LeddarGetRecordSize( gHandle ) );

        for(;;)
        {
            char lChoice;

            puts( "\nReplay Menu" );
            puts( "  1. Read Data" );
            puts( "  2. Read Configuration" );
            puts( "  3. Close" );

            lChoice = WaitKey();

            switch( lChoice )
            {
                case '1':
                    ReplayData();
                    break;
                case '2':
                    ReadConfiguration();
                    break;
                case '3':
                case  27:
                    LeddarDisconnect( gHandle );
                    return;
            }
        }
    }
    else
    {
        puts( "\nFailed to load file!" );
    }
}

// *****************************************************************************
// Function: ConfigureRecordingMenu
//
/// \brief   Menu allowing for configuration of data recording.
// *****************************************************************************

static void
ConfigureRecordingMenu( void )
{
    for(;;)
    {
        int    lChoice;
        LeddarChar lPath[256];

        puts( "\nConfigure Recording Menu" );
        LeddarGetRecordingDirectory( lPath, ARRAY_LEN(lPath) );
        printf( "  1. Change directory (%s)\n", lPath );
        printf( "  2. Change max file size (%dMB)\n", LeddarGetMaxRecordFileSize() );
        puts( "  3. Quit" );

        lChoice = toupper( LeddarGetKey() );

        switch( lChoice )
        {
            case '1':
                printf( "\nEnter recording directory: " );
                //LeddarScanf( LTS( "%255s" ), lPath );
                scanf( "%255s", lPath );
                LeddarConfigureRecording( lPath, 0, 0 );
                break;
            case '2':
                printf( "\nEnter max file size in MB: " );
                scanf( "%d", &lChoice );
                LeddarConfigureRecording( NULL, lChoice, 0 );
                break;
            case '3':
            case  27:
                return;
        }
    }
}

// *****************************************************************************
// Function: MainMenu
//
/// \brief   Display and responds to the main menu.
// *****************************************************************************

static void
MainMenu( void )
{

    for(;;)
    {
        int lChoice;

        puts( "\n\nMain Menu" );
        puts( "  1. Connect" );
        puts( "  2. Connect to single USB sensor" );
        puts( "  3. List Sensors" );
        puts( "  4. Replay Record" );
        puts( "  5. Configure Recording" );
        puts( "  6. Quit" );

        lChoice = toupper( LeddarGetKey() );

        switch( lChoice )
        {
            case '1':
                ConnectMenu( 0 );
                break;
            case '2':
                ConnectMenu( 1 );
                break;
            case '3':
            {
                char lAddresses[256];
                ListSensors( "USB", lAddresses, 255 );
                ListSensors( "SERIAL", lAddresses, 255 );
                break;
            }
            case '4':
                ReplayMenu();
                break;
            case '5':
                ConfigureRecordingMenu();
                break;
            case '6':
            case 'Q':
            case  27:
                puts( "\n*** Goodbye! ***" );
                return;
        }
    }
}

// *****************************************************************************
// Function: main
//
/// \brief   Standard C entry point!
// *****************************************************************************

int
main( int argc, char *argv[] )
{
    puts( "*************************************************" );
    puts( "* Welcome to the LeddarC Demonstration Program! *" );
    puts( "*************************************************" );

    gHandle = LeddarCreateWithConsole( argc, argv );

    MainMenu();

    LeddarDestroy( gHandle );

    return 0;
}

// End of file Main.c
