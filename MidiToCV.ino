//
// Generic MIDI to CV converter 
// Uses DAC module via SPI to convert MIDI events received from
//


// Note the script needs to include SPI:
#include <SPI.h>
#include <DAC.h>
#include <SoftwareSerial.h>

#define RX_PIN 2
#define TX_PIN 3
SoftwareSerial ss =  SoftwareSerial( RX_PIN, TX_PIN );

#define GATE_PIN 5

#define LOWEST_NOTE  0x24

#define NOTE_OFF   0x80 
#define NOTE_ON    0x90
#define PITCH_BEND 0xE0
#define CONTROL_CHANGE  0xB0
#define PROG_CHANGE 0xC0
#define MOD_WHEEL 0x01
#define GLIDE_CONTROLLER 0x52


#define MODE_AB 2      // Note to A + B
#define MODE_APBE 0    // Note to A, Mod wheel to B
#define MODE_APBV 1    // Note to A, Velocity to B
#define MODE_TUNEA440 3
#define MODE_TUNEA220 4
#define MODE_TUNEA880 5

#define MAX_MODE 5  // The maximum number of modes


#define PITCH_WHEEL_CENTERED 0x2000
#define PITCH_WHEEL_MAX 0x3FFF
#define MOD_WHEEL_MAX 0xFF

//#define LCD_DEBUGGING 1

// 20x4 Serial Controlled LCD:
class SerialLCD {
public:
  SerialLCD(); 
  void displayScreen( char* theText );
  void displayLine(int lineNum, char *theText);
  void displayChar(int lineNum, int charNum, char theChar);
  void clear();
  void backlight(int Percentage);

private:
  int d_pin;

};



SerialLCD lcd;



const float SEMITONE_INC = 1.0/12.0;  // semitone increment

float note_voltage = 0.0;

// defines for MIDI Shield components only
#define KNOB1  0
#define KNOB2  1

#define BUTTON1  17
#define BUTTON2  18
#define GATE_LED_PIN 16

#define STAT1  7
#define STAT2  6

#define OFF 1
#define ON 2
#define WAIT 3

#define A440 69  // MIDI Note 69 is 440
#define A220 57  // MIDI Note 57 is 220
#define A880 81  // MIDI Note 81 is 880

// You may need to change these depending on how he board is wired up
#define LDAC_PIN 8
#define SHDN_PIN 9
#define SS_PIN  10


int total_bytes =0; // The total number of MIDI bytes received
int disp_line = 1;
byte note;
byte velocity;
int pot;
byte channel;
byte stat;
byte current_channel = 1;
byte current_note = -1;
char buff[21];
byte midi_data[3];
int num_bytes_expected = 0;
int cnt=0;
int button1_press = 0;
int button2_press = 0;
int out_mode = 0;  // current output mode
int pitch_bend_value = 0;
int glide_value = 0; // The current glide value, 0 turns it off. Otherwise it's the time in ms to glide the note
bool doing_glide = false; // A flag to indicate we are gliding to a new value
unsigned long glide_target; // The time in millis() we expect to have eached the note target:
float glide_target_voltage = 0.0; // The voltage we want to reach
float glide_start_voltage = 0.0;  // The voltage we started at






DAC dac( LDAC_PIN, SHDN_PIN, SS_PIN );


void setup() {
  pinMode(STAT1,OUTPUT);   
  pinMode(STAT2,OUTPUT);



  pinMode(BUTTON1,INPUT);
  pinMode(BUTTON2,INPUT);

  pinMode(GATE_PIN, OUTPUT);

  digitalWrite(BUTTON1,HIGH);
  digitalWrite(BUTTON2,HIGH);

  for(int i = 0;i < 10;i++) // flash MIDI Sheild LED's on startup
  {
    digitalWrite(STAT1,HIGH);  
    digitalWrite(STAT2,LOW);
    delay(30);
    digitalWrite(STAT1,LOW);  
    digitalWrite(STAT2,HIGH);
    delay(30);
  }
  digitalWrite(STAT1,HIGH);   
  digitalWrite(STAT2,HIGH);


  //start serial with midi baudrate 31250
  Serial.begin(31250);     

  pinMode( RX_PIN, INPUT );
  pinMode( TX_PIN, OUTPUT);

  pinMode( GATE_LED_PIN, OUTPUT );

  lcd.clear();
  lcd.backlight(20);
  digitalWrite( GATE_LED_PIN, HIGH );
  lcd.displayLine( 1, "MIDI2CV Conv.   ");
  lcd.displayLine( 2, "Version 1.0     ");
  delay( 5000 );
  digitalWrite( GATE_LED_PIN, LOW );
  update_lcd();




#ifdef LCD_DEBUGGING
  lcd.clear();
#endif

}

//////////////////////////////////////////////////////////////////////



void loop() {

  update_glide_voltage();


  if( button( BUTTON1 ) ) {
    delay(100);
    if( button( BUTTON1 ) ) {
      current_channel += 1;
      if( current_channel > 12 ) {
        current_channel = 1;
      }
      update_lcd();
      while( button( BUTTON1 ) ) {
      }
    }
  }

  if( button( BUTTON2 ) ) {
    delay(100);
    if( button( BUTTON2 ) ) {
      out_mode += 1;
      if( out_mode > MAX_MODE) {
        out_mode = 0;
      }
      update_lcd();
      while( button( BUTTON2 ) ) {
      }
      if( out_mode == MODE_TUNEA440 ) {
        note_on( A440, 128 );
      } 
      else if( out_mode == MODE_TUNEA220 ) {
        note_on( A220, 128 );
      } 
      else if( out_mode == MODE_TUNEA880 ) {
        note_on( A880, 128 );
      } 
      else {
        note_off( A440 );
        current_note = -1;
      }


    }
  }



  //*************** MIDI IN ***************//
  if (Serial.available() > 0) {

    // read the incoming byte:
    midi_data[cnt] = Serial.read();
    total_bytes += 1;


#ifdef LCD_DEBUGGING
    snprintf(buff, 16, "%2d: %2x H%2x L%2x", total_bytes, midi_data[cnt], midi_data[cnt]&0xF0, midi_data[cnt]&0x0F );

    lcd.displayLine( disp_line, buff );
    disp_line += 1;
    if( disp_line > 2)
      disp_line = 1;
#endif

    cnt += 1;

    if( cnt == 1 ) {     


      stat = byte( midi_data[0] & 0xF0);
      channel = byte( 1 + midi_data[0] & 0x0F) ;


      if( stat == NOTE_ON ) {
        num_bytes_expected = 3;
      }
      else if( stat == NOTE_OFF ) {
        num_bytes_expected = 3; 
      }
      else if( stat == PITCH_BEND ) {
        num_bytes_expected = 3;
      }
      else if( stat == PROG_CHANGE ) {
        num_bytes_expected = 2;
      }
      else {
        num_bytes_expected = 3;
      }
    }

    // Once we have the reqired amount of data and on the right channel then action the note:
    if( cnt == num_bytes_expected ) {
      cnt = 0;


      if( channel == current_channel) {

        if( stat == NOTE_ON && midi_data[2] != 0 ) {

          note_on(midi_data[1], midi_data[2]);
          current_note = midi_data[1];
        }

        if( stat == NOTE_OFF || (stat == NOTE_ON && midi_data[2] == 0 )) {
          if( current_note == midi_data[1] ) {
            note_off( midi_data[1]); 
            current_note = -1;
          }
        }
        if( stat == PITCH_BEND ) {
          handle_pitch_bend();

        }
        if( stat == CONTROL_CHANGE ) {
          if( midi_data[1] == MOD_WHEEL ) {
            handle_mod_wheel();
          }
          if( midi_data[1] == GLIDE_CONTROLLER ) {
            handle_glide_control(); 
          }
        }

      }
    }

  }
}

// Handle a note on event. Set the voltage and turn the GATE on:
void note_on( byte note, byte velocity)
{
  float vel_voltage;
  digitalWrite(GATE_PIN, LOW ); // Turn the gate off
  digitalWrite( GATE_LED_PIN, LOW );
  glide_start_voltage = note_voltage;

  note_voltage = (float)(note - (byte)LOWEST_NOTE ) * SEMITONE_INC;  
  if( note_voltage < 0.0 ) {
    note_voltage = 0.0; 
  }
  if( note_voltage > 5.0 ) {
    note_voltage = 5.0; 
  }
  digitalWrite(STAT2, LOW);
  digitalWrite(GATE_PIN, HIGH );
  digitalWrite( GATE_LED_PIN, HIGH );

  if( glide_value > 0 ) {
    doing_glide = true;
    glide_target_voltage = note_voltage;
    glide_target = millis() + glide_value;
  } 
  else {
    // Just set the note value:

    dac.setVoltage( note_voltage, CHANNEL_A );
    if( out_mode == MODE_AB ) {
      dac.setVoltage( note_voltage, CHANNEL_B );
    }
  }


  if( out_mode == MODE_APBV ) {  // Send velocity as voltage on Channel B

    vel_voltage = float(velocity)/128.0 * 5.0;
    if( vel_voltage > 5.0 ) {
      vel_voltage= 5.0;
    }
    if( vel_voltage < 0.0 ) {
      vel_voltage = 0; 
    }
    dac.setVoltage( vel_voltage, CHANNEL_B );
  }
}


// Handle a note-off event. Simply turn the gate off:
void note_off( byte note ) {
  digitalWrite( GATE_PIN, LOW ); 
  digitalWrite( GATE_LED_PIN, LOW );
  digitalWrite( STAT2, HIGH );

}

char button(char button_num)
{
  return (!(digitalRead(button_num)));
}

void update_lcd( void ) {

  snprintf(buff, 16, "Chn:%2d Gld:%4d", int(current_channel), int(glide_value) );
  lcd.displayLine( 1, buff );

  if( out_mode == MODE_AB ) {
    lcd.displayLine( 2, "Mode:A+B" );
  } 
  else if (out_mode == MODE_APBE ) {
    lcd.displayLine( 2, "Mode:A=Pth,B=Exp");
  } 
  else if (out_mode == MODE_APBV ) {
    lcd.displayLine( 2, "Mode:A=Pth,B=Vel");
  } 
  else if (out_mode == MODE_TUNEA440 ) {
    lcd.displayLine( 2, "Mode:Tune 440Hz ");
  }
  else if (out_mode == MODE_TUNEA220 ) {
    lcd.displayLine( 2, "Mode:Tune 220Hz ");
  }
  else if (out_mode == MODE_TUNEA880 ) {
    lcd.displayLine( 2, "Mode:Tune 880Hz ");
  }
}

void handle_pitch_bend( void ) {
  pitch_bend_value = midi_data[2];
  pitch_bend_value = pitch_bend_value << 7;
  pitch_bend_value = pitch_bend_value | midi_data[1];
  float voltage_offset = float(pitch_bend_value-PITCH_WHEEL_CENTERED) /PITCH_WHEEL_MAX * SEMITONE_INC * 4;
  float voltage = note_voltage + voltage_offset;
  if( voltage < 0.0 )
    voltage = 0.0;
  if( voltage > 5.0 )
    voltage = 5.0;
  dac.setVoltage( voltage, CHANNEL_A );
  if( out_mode == MODE_AB ) {
    dac.setVoltage( voltage, CHANNEL_B );
  }
  // snprintf( buff, 16, "PB: %4d %f ",pitch_bend_value, voltage_offset );
  // lcd.displayLine( 2, buff );
}

void handle_mod_wheel( void ) {

  float voltage = (float(midi_data[2])/MOD_WHEEL_MAX)*10.0;

  if( out_mode == MODE_APBE ) {
    // snprintf( buff, 16, "MW: %d",int(voltage*100) );
    // lcd.displayLine( 2, buff );
    if( voltage < 0.0 )
      voltage = 0.0;
    if( voltage > 5.0 )
      voltage = 5.0;

    dac.setVoltage( voltage, CHANNEL_B );
  }

}

void handle_glide_control( void ) {
  int glide = int((float(midi_data[2])/127)*1000);
  glide_value = glide;
  update_lcd();
  doing_glide= false; // turn off the current glide
}

void update_glide_voltage( void ) {

  if( doing_glide == false )
    return;

  if( millis() > glide_target ) {
    doing_glide = false;
    return; 
  }

  // We are still doing glide so calculate the current note voltage:
  unsigned long time_remaining = glide_target - millis();

  float fraction = float(time_remaining)/float(glide_value);

  note_voltage = glide_target_voltage - (glide_target_voltage-glide_start_voltage)*fraction;
  dac.setVoltage( note_voltage, CHANNEL_A );
  if( out_mode == MODE_AB ) {
    dac.setVoltage( note_voltage, CHANNEL_B );
  }
}










































