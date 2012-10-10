/* User preferences file - Modify this header to use the correct options for your 
 * installation. 
 * Be sure to set the passwords/etc, as the defaul is "1234"
 */ 

/* Hardware options
 *
 */
#define MCU328         // Set this if using any boards other than the "Mega"
#define HWV3STD        // Use this option if using Open access v3 Standard board

#define MCPIOXP        // this if using the v3 hardware with the MCP23017 i2c IO chip
//#define AT24EEPROM     // Set this if you have the At24C i2c EEPROM chip installed
#define LCDBOARD       // Set this if using an LCD. Default is 16x2 with controller.



/* Static user List - Implemented as an array for testing and access override 
*/                               
#define LCDBOARD                        // Uncomment to use LCD board
                                        // Uses the "cLCD" library that extends arduino LCD class
                                        // Has issues - must use a non-standard pinout, disables other MCP IO pins
                                        // Library is from the TC4 Coffee Roaster project
                                        // Download here: http://code.google.com/p/tc4-shield/
                                        
#define STDLOG				// Change the logging header file to change logging output.
                 
#define DEBUG 2                         // Set to 2 for display of raw tag numbers in log files, 1 for only denied, 0 for never.               
#define VERSION 1.34
#define UBAUDRATE 57600                 // Set the baud rate for the USB serial port

#define josh   0xDA28AA                  // Name and badge number in HEX. We are not using checksums or site ID, just the whole
#define rhys   0x25AEBC6                  // output string from the reader.
#define eric   0xD621AC
#define tim    0x14F0C85
const long  superUserList[] = { josh, rhys, eric, tim };  // Super user table (cannot be changed by software)

#define PRIVPASSWORD 0x1234             // Console "priveleged mode" password

#define DOORDELAY 500                  // How long to open door lock once access is granted. (2500 = 2.5s)
#define SENSORTHRESHOLD 100             // Analog sensor change that will trigger an alarm (0..255)
#define KEYPADTIMEOUT 5000              // Timeout for pin pad entry. Users on keypads can enter commands after reader swipe.


