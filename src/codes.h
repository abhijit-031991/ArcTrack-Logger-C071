////// Calibration codes //////

#define CALIBRATION_BEGIN (byte)1       // Sent from device to App to indicate start of calibration mode
#define CALIBRATION_END (byte)2         // Sent from device to App to indicate end of calibration mode
#define FINISH_CALIBRATION (byte)3      // Sent from App to device to indicate calibration process is complete

#define GPS_CALIBRATION (byte)10        // Sent from device to App to indicate start of GPS calibration
#define GPS_CALIBRATION_END (byte)11    // Sent from device to App to indicate end of GPS calibration
#define GPS_ERROR (byte)12              // Sent from device to App if GPS error detected
#define GPS_SUCCESS (byte)13            // Sent from device to App if GPS is functioning properly      
#define GPS_CALIBRATING (byte)14        // Sent from device to App to indicate GPS is attempting to acquire fix

#define GSM_TEST (byte)15               // Sent from device to App to indicate start of GSM test
#define GSM_TEST_END (byte)16           // Sent from device to App to indicate end of GSM test
#define GSM_TEST_ERROR (byte)17         // Sent from device to App if GSM test failed     
#define GSM_TEST_SUCCESS (byte)18       // Sent from device to App if GSM test succeeded

#define FLASH_DIAGNOSTICS (byte)21      // Sent from device to App to indicate start of flash diagnostics
#define FLASH_DIAGNOSTICS_END (byte)22  // Sent from device to App to indicate end of flash diagnostics
#define FLASH_ERROR (byte)23            // Sent from device to App if flash error detected
#define FLASH_SUCCESS (byte)24          // Sent from device to App if flash is functioning properly

#define ABORT_CALIBRATION (byte)30      // Sent from App to device to abort calibration process
#define CALIBRATION_ABORTED (byte)31    // Sent from device to App if calibration is

////// Radio Ping Codes(30-39) //////
#define SIMPLE_PING (byte)30            // Sent from device to App along with ID. App should respond with SIMPLE_PING_ACK
#define SIMPLE_PING_ACK (byte)31        // Sent from App to device to acknowledge SIMPLE_PING. If recieved, device should send the long Ping with info.

////// Settings Codes (40-49) //////
#define REQUEST_SETTINGS (byte)40       // Sent from App to device to request current settings
#define SETTINGS_UPDATED (byte)41       // Sent from device to App to confirm settings have been updated
#define SETTINGS_UPDATE_ERROR (byte)42  // Sent from device to App if settings update failed
#define SETTINGS_RESET (byte)43         // Sent from App to device to reset settings to default

////// Data Download Codes(50-55)  //////
#define DATA_DOWNLOAD_NEW (byte)50      // Sent from App to device to request new data since last download
#define DATA_DOWNLOAD_ALL (byte)51      // Sent from App to device to request all stored data
#define DATA_DOWNLOAD_END (byte)52      // Sent from device to App to indicate end of data transmission
#define DATA_DOWNLOAD_ERROR (byte)53    // Sent from device to App if data transmission failed
#define DATA_DOWNLOAD_BEGIN (byte)54    // Sent from device to App to indicate start of data transmission

////// Memory Codes (60-69) //////
#define MEMORY_FULL (byte)61            // Sent from device to App if memory is full
#define MEMORY_ERROR (byte)62           // Sent from device to App if memory read/write error
#define MEMORY_CLEAR (byte)63           // Sent from App to device to clear memory
#define MEMORY_CLEARED (byte)64         // Sent from device to App to confirm memory has been cleared
#define MEMORY_CLEAR_ERROR (byte)65     // Sent from device to App if memory clear failed
#define MEMORY_STATUS (byte)66          // Sent from App to device to request memory status
#define MEMORY_GOOD (byte)67            // Sent from device to App if memory is functioning properly

////// Accelerometer Codes(70-75) //////
#define ACCELEROMETER_OK (byte)71       // Sent from device to App if accelerometer is functioning properly
#define ACCELEROMETER_ERROR (byte)72    // Sent from device to App if accelerometer error detected

////// Capacitance Sensor Codes (80-85)   //////
#define CAPACITANCE_OK (byte)81       // Sent from device to App if capacitance sensor is functioning properly
#define CAPACITANCE_ERROR (byte)82  // Sent from device to App if capacitance sensor error detected

