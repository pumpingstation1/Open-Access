/* Set up some strings that will live in flash instead of memory. This saves our precious 2k of
 * RAM for something else.
*/
const prog_uchar rebootMessage[]          PROGMEM  = {"Access Control System rebooted."};
const prog_uchar doorChimeMessage[]       PROGMEM  = {"Front Door opened."};
const prog_uchar doorslockedMessage[]     PROGMEM  = {"All Doors relocked"};
const prog_uchar alarmtrainMessage[]      PROGMEM  = {"Alarm Training performed."};
const prog_uchar privsdeniedMessage[]     PROGMEM  = {"Access Denied. Priveleged mode is not enabled."};
const prog_uchar privsenabledMessage[]    PROGMEM  = {"Priveleged mode enabled."};
const prog_uchar privsdisabledMessage[]   PROGMEM  = {"Priveleged mode disabled."};
const prog_uchar privsAttemptsMessage[]   PROGMEM  = {"Too many failed attempts. Try again later."};

const prog_uchar consolehelpMessage1[]    PROGMEM  = {"Valid commands are:"};
const prog_uchar consolehelpMessage2[]    PROGMEM  = {"(d)ate, (s)show user, (m)odify user <num>  <usermask> <tagnumber>"};
const prog_uchar consolehelpMessage3[]    PROGMEM  = {"(a)ll user dump,(r)emove_user <num>,(o)open door <num>"};
const prog_uchar consolehelpMessage4[]    PROGMEM  = {"(u)nlock all doors,(l)lock all doors"};
const prog_uchar consolehelpMessage5[]    PROGMEM  = {"(1)disarm_alarm, (2)arm_alarm,(3)train_alarm (9)show_status"};
const prog_uchar consolehelpMessage6[]    PROGMEM  = {"(e)nable <password> - enable or disable priveleged mode"};                                       
const prog_uchar consoledefaultMessage[]  PROGMEM  = {"Invalid command. Press '?' for help."};

const prog_uchar statusMessage1[]         PROGMEM  = {"Alarm armed state (1=armed):"};
const prog_uchar statusMessage2[]         PROGMEM  = {"Alarm siren state (1=activated):"};
const prog_uchar statusMessage3[]         PROGMEM  = {"Server/Locker door open state (0=closed):"};
const prog_uchar statusMessage4[]         PROGMEM  = {"Front door open state (0=closed):"};
const prog_uchar statusMessage5[]         PROGMEM  = {"Door 1 unlocked state(1=locked):"};                   
const prog_uchar statusMessage6[]         PROGMEM  = {"Door 2 unlocked state(1=locked):"};
const prog_uchar statusMessage7[]         PROGMEM  = {"Front KEYED door open state (0=closed):"};
const prog_uchar statusMessage8[]         PROGMEM  = {"Back door open state (0 = closed):"};

typedef struct {
	byte l1;
	byte l2;
	byte l3;
	byte l4;
        uint8_t detail;
} logStruct;

void PROGMEMprintln(const prog_uchar str[])    // Function to retrieve logging strings from program memory
{                                              // Prints newline after each string  
  char c;
  if(str) {
  while((c = pgm_read_byte(str++))){
    Serial.write(c);
    
  }
  Serial.println();
  } else {
    Serial.println("ERROR IN EEPROM MEMORY PRINTING");
  }
}

void PROGMEMprint(const prog_uchar str[])    // Function to retrieve logging strings from program memory
{                                            // Does not print newlines
  char c;
  if(str) { 
  while((c = pgm_read_byte(str++))){
    Serial.write(c);
  }
  } else {
    Serial.println("ERROR IN EEPROM MEMORY PRINTING");
  }
}

void logError(logStruct logcode) {
	Serial.println("LOGGING ERROR. logStruct: ");
	Serial.println(logcode.l1);
	Serial.println(logcode.l2);
	Serial.println(logcode.l3);
	Serial.println(logcode.l4);
	Serial.println(logcode.detail);
}

void logAccess(logStruct logcode, long tag) {
/*  Converts numeric log code to heiarchial log format.
 *  ACCESS Events
 *  logStruct use. 
 *  l1 specific reader
 *  l2 reader event
 * 	l3 further explaination of event
 *  tag is user's keyfob ID/keypad command/user location
 */
	Serial.print("<access> READER"); 						
	Serial.print(logcode.l1);							
	Serial.print(" ");

	switch(logcode.l2) {							// Reader Events
		case 1:
			Serial.print("Tag Presented: ");
			Serial.println(tag,HEX);
			break;
		case 2:
			Serial.print("Access Granted to Tag: ");
			Serial.println(tag,HEX);
			break;
		case 3:
		    Serial.print("Access Denied to Tag: ");
		    Serial.println(tag,HEX);
			break;
		case 4:
		    Serial.print("Keypad Entry: ");
		    Serial.println(tag,HEX);
			break;
		case 5:
		    Serial.print("User Locked-out: ");
		    Serial.println(tag,HEX);
			break;
		case 6 :
			Serial.println("Superuser access: ");
			Serial.println(tag,HEX);
		default:
			logError(logcode);
			break;
	}
}

void logAlarm(logStruct logcode) {
/*  Converts numeric log code to heiarchial log format.
 *  ALARM Events
 *  logStruct use. 
 *  l1 specific zone
 *  l2 alarm event
 * 	l3 further alarm event details.
 */
	Serial.print("<alarm> ZONE"); 						
	Serial.print(logcode.l1);							
	Serial.print(" ");

	switch(logcode.l2) {							// Alarm Events
		case 1:
			Serial.println("Sensor Activated.");
			break;
		case 2:
			Serial.println("ALARM TRIGGERED.");
			break;
		case 3:
		    Serial.print("Alarm State Changed to: ");
		    Serial.println(logcode.l3,DEC);
			break;
		case 4:
		    Serial.print("Alarm Armed State Changed to: ");
		    Serial.println(logcode.l3,DEC);
			break;
		default:
			logError(logcode);
			break;
	}
}

void logDoor(logStruct logcode) {
/*  Converts numeric log code to heiarchial log format.
 *  DOOR Events
 *  logStruct use. 
 *  l1 specific door number
 *  l2 action of door
 * 	l3 further explaination of action
 */
	Serial.print("<door> DOOR"); 						
        Serial.print(logcode.l1);		
	Serial.print(" ");

	switch(logcode.l2) {							
		case 1:
			Serial.print("Locked");
			switch(logcode.l3) {					
				case 0:
					Serial.println(" Default Function Call");
					break;
				case 1:
					break;
				case 2:							// Failsafe Locking Mechanism
													// uses .detail for failsafe hours/minutes
					Serial.print(" Bedtime Locking Mechanism set for: ");
					Serial.print(logcode.detail);
					Serial.println(" hours.");
					break;
                                case 3:
                                        Serial.println(" All Doors relocked");
                                        break;
				default:
					logError(logcode);
					break;
			break;
			}				
			break;
		case 2:
			Serial.println("Unlocked.");
			break;
		default:
			logError(logcode);
			break;
	}
}

void logSystem(logStruct logcode) {
/*  Converts numeric log code to heiarchial log format.
 *  System Events
 *  logStruct use. 
 *  l1 event
 */
	Serial.print("<system> "); 						
					

	switch(logcode.l1) {							// Reader Events
		case 1:
			PROGMEMprintln(rebootMessage);
			break;
		case 2:
			PROGMEMprintln(doorChimeMessage);
			break;
		case 3:
		        Serial.println("User Database Erased.");
			break;
		case 4:
		        Serial.println("Invalid User modify attempt.");
			break;
		case 5:
		        Serial.println("Invalid User delete attempt.");
			break;
		case 6:
		        Serial.print("User ");
		        Serial.print(logcode.detail);
		        Serial.println(" successfully modified.");
			break;
		case 7:
		        Serial.print("User ");
		        Serial.print(logcode.detail);
		        Serial.println(" successfully deleted.");
			break;
		case 8:
		        Serial.print("User ");
		        Serial.print(logcode.detail);
		        Serial.println(" authenticated.");
			break;
		case 9:
			Serial.print("Tag ");
			Serial.print(logcode.detail,HEX);
			Serial.println(" not found in user database.");
                        break;
		default:
			logError(logcode);
			break;
	}
}
