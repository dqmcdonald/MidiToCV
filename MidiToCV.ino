//
// Generic MIDI to CV converter 
// Uses DAC module via SPI to convert MIDI events received from
//
// Relies of MIDI library from https://github.com/FortySevenEffects/arduino_midi_library/


// Note the script needs to include SPI:
#include <SPI.h>
#include <DAC.h>
#include <SoftwareSerial.h>
#include <MIDI.h>
#include "noteList.h"



#define RX_PIN 2
#define TX_PIN 3
SoftwareSerial ss =  SoftwareSerial( RX_PIN, TX_PIN );

#define GATE_PIN 5

#define DEFAULT_LOWEST_NOTE  0x24

#define NOTE_OFF   0x80 
#define NOTE_ON    0x90
#define PITCH_BEND 0xE0
#define CONTROL_CHANGE  0xB0
#define PROG_CHANGE 0xC0
#define MOD_WHEEL 0x01
#define GLIDE_CONTROLLER 0x52


#define MODE_OUTPUT      0
#define MODE_CHANNEL     1
#define MODE_GLIDE       2
#define MODE_LOWEST_NOTE 3
#define MODE_LEGATO      4

#define MAX_MODE 4


#define MODE_AB 2      // Note to A + B
#define MODE_APBE 0    // Note to A, Mod wheel to B
#define MODE_APBV 1    // Note to A, Velocity to B
#define MODE_TUNEA440 3
#define MODE_TUNEA220 4
#define MODE_TUNEA880 5

#define MAX_OUTPUT_MODE 5  // The maximum number of modes


#define PITCH_WHEEL_CENTERED MIDI_PITCHBEND_MAX/2

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

const int LOWEST_NOTES[] = { 
  12, 24, 36, 48, 60 };

const float SEMITONE_INC = 1.0/12.0;  // semitone increment

float note_voltage = 0.0;

// defines for MIDI Shield components only
#define KNOB1  0
#define KNOB2  1

#define BUTTON1  17   // Button 1 is the "Mode" button
#define BUTTON2  18   // Button 2 is the "Value" button
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

int lowest_note_index = 1;
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
static const unsigned int MAX_NUM_NOTES = 16; // Maximum number of MIDI notes
MidiNoteList<MAX_NUM_NOTES> midi_notes;

int current_mode = MODE_OUTPUT;  // The current input and display mode

bool legato_mode = false;


MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);



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
  lcd.displayLine( 2, "Version 1.1     ");
  delay( 5000 );
  digitalWrite( GATE_LED_PIN, LOW );
  update_lcd();

  MIDI.setHandleNoteOn(handle_note_on);  
  MIDI.setHandleNoteOff(handle_note_off);
  MIDI.setHandlePitchBend(handle_pitch_bend);
  MIDI.setHandleControlChange(handle_control_change);
  // Initiate MIDI communications, listen to current channel
  MIDI.begin(current_channel);




#ifdef LCD_DEBUGGING
  lcd.clear();
#endif

}

//////////////////////////////////////////////////////////////////////



void loop() {

  update_glide_voltage();

  handle_button_one();
  handle_button_two();





  //*************** MIDI IN ***************//
  // Call MIDI.read the fastest you can for real-time performance.
  MIDI.read();

}

void handle_notes_changed(bool is_first_note = false)
{
  if (midi_notes.empty())
  // No notes - turn the gate off:
  {
    set_gate(false);
  }
  else
  {
    // Possible playing modes:
    // Mono Low:  use midi_notes.getLow
    // Mono High: use midi_notes.getHigh
    // Mono Last: use midi_notes.getLast
    byte note = 0;
    byte vel = 0;
    if (midi_notes.getLast(note,vel))
    {
      set_voltage( note, vel );
      if (is_first_note)
      {
        set_gate(true);
      }
      else
      {
        if( !legato_mode ) 
          pulse_gate(); // Re trigger envelopes. Remove for legato effect.
      }
    }
  }
}



// Handle a note on event. Set the voltage and turn the GATE on:
void handle_note_on( byte channel, byte note, byte velocity)
{

  const bool first_note = midi_notes.empty();
  midi_notes.add(MidiNote(note, velocity));
  handle_notes_changed(first_note);
}


void set_voltage( byte note, byte velocity ) {


  float vel_voltage;
  glide_start_voltage = note_voltage;

  note_voltage = (float)(note - (byte)LOWEST_NOTES[lowest_note_index] ) 
    * SEMITONE_INC;  
  if( note_voltage < 0.0 ) {
    note_voltage = 0.0; 
  }
  if( note_voltage > 5.0 ) {
    note_voltage = 5.0; 
  }

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


// Handle a note-off event.
void handle_note_off(byte channel, byte pitch, byte velocity ) {
  
    midi_notes.remove(pitch);
    handle_notes_changed();
}

char button(char button_num)
{
  return (!(digitalRead(button_num)));
}

void handle_button_one ( void ) {
  // The mode button - simply update the main mode and redisplay in the LCD
  //
  if( button( BUTTON1 ) ) {
    delay(100);
    if( button( BUTTON1 ) ) {
      current_mode += 1;
      if( current_mode > MAX_MODE ) {
        current_mode = 0;
      }
      update_lcd();
      while( button( BUTTON1 ) ) {

      }

    }
  }

}

void handle_button_two ( void ) {
  // The value button - what we do here depends on the value of the main mode:



  if( button( BUTTON2 ) ) {
    delay(100);
    if( button( BUTTON2 ) ) {


      if( current_mode == MODE_OUTPUT) {
        out_mode += 1;
        if( out_mode > MAX_OUTPUT_MODE) {
          out_mode = 0;
        }


        if( out_mode == MODE_TUNEA440 ) {
          handle_note_on( current_channel, A440, 128 );
        } 
        else if( out_mode == MODE_TUNEA220 ) {
          handle_note_on( current_channel, A220, 128 );
        } 
        else if( out_mode == MODE_TUNEA880 ) {
          handle_note_on( current_channel, A880, 128 );
        } 
        else {
          handle_note_off( current_channel, A440, 0 );
          current_note = -1;
        }

      }
    }

    if( current_mode == MODE_CHANNEL ) {
      current_channel += 1;
      if( current_channel > 12 ) {
        current_channel = 0;
      }
      MIDI.setInputChannel( current_channel );

    }

    if( current_mode == MODE_GLIDE) {
      glide_value += 5;
      if( glide_value > 127 ) {
        glide_value = 0;
      }
    }

    if( current_mode == MODE_LOWEST_NOTE ) {
      lowest_note_index += 1;
      if( lowest_note_index > 4 ) {
        lowest_note_index = 0;
      }
    }

    if( current_mode == MODE_LEGATO ) {
      legato_mode = ! legato_mode;
    }



  }


  update_lcd();
  while( button( BUTTON2 ) ) {
  }

}

void update_lcd( void ) {


  // In ouput mode
  if( current_mode == MODE_OUTPUT ) {
    lcd.displayLine( 1, "Output Mode:" );
    if( out_mode == MODE_AB ) {
      lcd.displayLine( 2, "Pitch to A + B" );
    } 
    else if (out_mode == MODE_APBE ) {
      lcd.displayLine( 2, "Pitch: A, Exp: B");
    } 
    else if (out_mode == MODE_APBV ) {
      lcd.displayLine( 2, "Pitch: A, Vel: B");
    } 
    else if (out_mode == MODE_TUNEA440 ) {
      lcd.displayLine( 2, "Tune 440Hz ");
    }
    else if (out_mode == MODE_TUNEA220 ) {
      lcd.displayLine( 2, "Tune 220Hz ");
    }
    else if (out_mode == MODE_TUNEA880 ) {
      lcd.displayLine( 2, "Tune 880Hz ");
    }
  }

  if( current_mode == MODE_CHANNEL ) {
    snprintf(buff, 16, "Channel :%2d", int(current_channel) );
    lcd.displayLine( 1, buff );
    lcd.displayLine( 2, "" );
  }

  if( current_mode == MODE_GLIDE ) {
    snprintf(buff, 16, "Glide: %4d", int(glide_value) );
    lcd.displayLine( 1, buff );
    lcd.displayLine( 2, "" );
  }

  if( current_mode == MODE_LOWEST_NOTE ) {
    snprintf(buff, 16, "Low Note: C%1d", int(lowest_note_index) );
    lcd.displayLine( 1, buff );
    lcd.displayLine( 2, "" );
  }

  if( current_mode == MODE_LEGATO ) {
    if( legato_mode )
      snprintf(buff, 16, "Legato On");
    else
      snprintf(buff, 16, "Legato off");
    lcd.displayLine( 1, buff );
    lcd.displayLine( 2, "" );
  }



}

inline void set_gate(bool gate_active)
{
  digitalWrite(GATE_PIN, gate_active ? HIGH : LOW);
  digitalWrite(GATE_LED_PIN, gate_active ? HIGH : LOW);
}

inline void pulse_gate()
{
  set_gate(false);
  delay(1);
  set_gate(true);
}




void handle_pitch_bend( byte channel, int bend  ) {
  float voltage_offset = float(bend-PITCH_WHEEL_CENTERED) /MIDI_PITCHBEND_MAX * SEMITONE_INC * 4;
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

void handle_control_change( byte channel, byte number, byte value  ) {

  if( number == MOD_WHEEL ) {
    handle_mod_wheel(value);
  }
  if( number == GLIDE_CONTROLLER ) {
    handle_glide_control(value); 
  }
}


void handle_mod_wheel( byte value ) {

  float voltage = (float(value)/MOD_WHEEL_MAX)*10.0;

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

void handle_glide_control( byte value ) {
  int glide = int((float(value)/127)*1000);
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





















































