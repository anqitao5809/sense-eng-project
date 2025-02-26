#include <SoftwareSerial.h>
#include <DFRobot_DF1101S.h>

const int BUFFER_SIZE = 14; // RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const int DATA_SIZE = 10; // 10byte data (2byte version + 8byte tag)
const int DATA_VERSION_SIZE = 2; // 2byte version (actual meaning of these two bytes may vary)
const int DATA_TAG_SIZE = 8; // 8byte tag
const int CHECKSUM_SIZE = 2; // 2byte checksum

SoftwareSerial ssrfid = SoftwareSerial(4,5); 
SoftwareSerial df1101sSerial(0,1); //RX, TX
uint8_t buffer[BUFFER_SIZE]; // used to store an incoming data frame 
int buffer_index = 0;

DFRobot_DF1101S df1101s;

const int rfid_debounce_time_ms = 3000;
const int buttonPin = 3;  // the number of the pushbutton pin
int buttonState; //declear button state
unsigned long last_audio_time;

void setup() {
  // put your setup code here, to run once:
    // initialize the pushbutton pin as an input:
    Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
   ssrfid.begin(9600);
   df1101sSerial.begin(115200);
   ssrfid.listen(); 
  while(!df1101s.begin(df1101sSerial)){
    Serial.println("Init failed, please check the wire connection!");
    delay(1000);
  }
   Serial.println("INIT DONE");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("looping");
  buttonState = digitalRead(buttonPin);
  unsigned id_num = readNow();
  if (buttonState == LOW) { //WRITE MODE: RECORD AUDIO
    Serial.print("button pressed \n");
    df1101s.switchFunction(df1101s.RECORD);

    //trigger write mode
  }
  else { //READ MODE: PLAY AUDIO
    //read mode always
    //Serial.print("button not pressed \n");
    int will_sound = LOW;
     if (id_num!=0 && (millis()-last_audio_time)> rfid_debounce_time_ms ) {  //if reading valid rfid
      tone(9,262,250);
      last_audio_time = millis();
     }

    if (id_num!=0) {
     Serial.print(id_num);
     Serial.print("\n");
    }

  }
}


unsigned readNow() {
    if (ssrfid.available() > 0){
    bool call_extract_tag = false;
    
    int ssvalue = ssrfid.read(); // read 
    if (ssvalue == -1) { // no data was read
      return;
    }

    if (ssvalue == 2) { // RDM630/RDM6300 found a tag => tag incoming 
      buffer_index = 0;
    } else if (ssvalue == 3) { // tag has been fully transmitted       
      call_extract_tag = true; // extract tag at the end of the function call
    }

    if (buffer_index >= BUFFER_SIZE) { // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
      Serial.println("Error: Buffer overflow detected! ");
      return;
    }
    
    buffer[buffer_index++] = ssvalue; // everything is alright => copy current value to buffer

    if (call_extract_tag == true) {
      if (buffer_index == BUFFER_SIZE) {
        unsigned tag = extract_tag();
        return tag;
      } else { // something is wrong... start again looking for preamble (value: 2)
        buffer_index = 0;
        return;
      }
    }    
  }
  else {
    return 0; // return 0 when none exist
  }    
}

unsigned extract_tag() {
    uint8_t msg_head = buffer[0];
    uint8_t *msg_data = buffer + 1; // 10 byte => data contains 2byte version + 8byte tag
    uint8_t *msg_data_version = msg_data;
    uint8_t *msg_data_tag = msg_data + 2;
    uint8_t *msg_checksum = buffer + 11; // 2 byte
    uint8_t msg_tail = buffer[13];

    // print message that was sent from RDM630/RDM6300
    // Serial.println("--------");

    // Serial.print("Message-Head: ");
    // Serial.println(msg_head);

    // Serial.println("Message-Data (HEX): ");
    // for (int i = 0; i < DATA_VERSION_SIZE; ++i) {
    //   Serial.print(char(msg_data_version[i]));
    // }
    // Serial.println(" (version)");
    
    // for (int i = 0; i < DATA_TAG_SIZE; ++i) {
    //   Serial.print(char(msg_data_tag[i]));
    // }
    // Serial.println(" (tag)");

    // Serial.print("Message-Checksum (HEX): ");
    // for (int i = 0; i < CHECKSUM_SIZE; ++i) {
    //   Serial.print(char(msg_checksum[i]));
    // }
    // Serial.println("");

    // Serial.print("Message-Tail: ");
    // Serial.println(msg_tail);

    // Serial.println("--");

    long tag = hexstr_to_value(msg_data_tag, DATA_TAG_SIZE);
    // Serial.print("Extracted Tag: ");
    // Serial.println(tag);

    // long checksum = 0;
    // for (int i = 0; i < DATA_SIZE; i+= CHECKSUM_SIZE) {
    //   long val = hexstr_to_value(msg_data + i, CHECKSUM_SIZE);
    //   checksum ^= val;
    // }
    // Serial.print("Extracted Checksum (HEX): ");
    // Serial.print(checksum, HEX);
    // if (checksum == hexstr_to_value(msg_checksum, CHECKSUM_SIZE)) { // compare calculated checksum to retrieved checksum
    //   Serial.print(" (OK)"); // calculated checksum corresponds to transmitted checksum!
    // } else {
    //   Serial.print(" (NOT OK)"); // checksums do not match
    // }


    return tag;
}


long hexstr_to_value(char *str, unsigned int length) { // converts a hexadecimal value (encoded as ASCII string) to a numeric value
  char* copy = malloc((sizeof(char) * length) + 1); 
  memcpy(copy, str, sizeof(char) * length);
  copy[length] = '\0'; 
  // the variable "copy" is a copy of the parameter "str". "copy" has an additional '\0' element to make sure that "str" is null-terminated.
  long value = strtol(copy, NULL, 16);  // strtol converts a null-terminated string to a long value
  free(copy); // clean up 
  return value;
}