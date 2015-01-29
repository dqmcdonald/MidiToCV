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
#include <avr/eeprom.h>

// Software serial is used for the LCD display
#define RX_PIN 2
#define TX_PIN 3
SoftwareSerial ss =  SoftwareSerial( RX_PIN, TX_PIN );

#define GATE_PIN 5
#define GATE_LED_PIN 16
#define BUTTON1_PIN  18   // Button 1 is the "Mode" button
#define BUTTON2_PIN  17   // Button 2 is the "Value" button
// You may need to change these depending on how he board is wired up
#define LDAC_PIN 8
#define SHDN_PIN 9
#define SS_PIN  10

#define MOD_WHEEL 0x01          // Mod wheel is control change #1
#define GLIDE_CONTROLLER 0x52   // This is one of the rotary encoders on the keyboard

// Main modes - these control the display and function of the buttons
#define MODE_OUTPUT      0
#define MODE_CHANNEL     1
#define MODE_GLIDE       2
#define MODE_LOWEST_NOTE 3
#define MODE_LEGATO      4

#define MAX_MODE 4  // The number of main modes

// Output modes
#define MODE_OUTPUT_AB          2    // Note to A + B
#define MODE_OUTPUT_APBE        0    // Note to A, Mod wheel to B
#define MODE_OUTPUT_APBV        1    // Note to A, Velocity to B
#define MODE_OUTPUT_TUNEA440    3    // Set tuning to 440Hz
#define MODE_OUTPUT_TUNEA220    4    // Set tuning to 220Hz
#define MODE_OUTPUT_TUNEA880    5    // Set tuning to 880Hz

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

// Values available of the lowest notes:
const int LOWEST_NOTES[] = {  
  12, 24, 36, 48, 60 };

const float SEMITONE_INC = 1.0/12.0;  // semitone increment

float note_voltage = 0.0;  // The voltage in place for the 

// Notes used for tuning:
#define A440 69  // MIDI Note 69 is 440
#define A220 57  // MIDI Note 57 is 220
#define A880 81  // MIDI Note 81 is 880



char buff[21];
bool doing_glide = false; // A flag to indicate we are gliding to a new value
unsigned long glide_target; // The time in millis() we expect to have eached the note target:
float glide_target_voltage = 0.0; // The voltage we want to reach
float glide_start_voltage = 0.0;  // The voltage we started at
static const unsigned int MAX_NUM_NOTES = 16; // Maximum number of MIDI notes that can be active at one time
MidiNoteList<MAX_NUM_NOTES> midi_notes;  // A list of Midi Notes

struct settings_t
{
  int current_mode;          // The current main mode
  int output_mode;           // current output mode
  byte current_channel;      // The midi channel
  int glide_value;           // The current glide value
  int lowest_note_index;     // The lowest note index
  bool legato_mode;          // Legato mode
  bool settings_saved;       // Whether these settings have ever been saved
} 
settings;



MIDI_CREATE_INSTANCE(HardwareSerial, Serial, MIDI);



DAC dac( LDAC_PIN, SHDN_PIN, SS_PIN );


void setup() {

 
  pinMode(BUTTON1_PIN,INPUT);
  pinMode(BUTTON2_PIN,INPUT);

  pinMode(GATE_PIN, OUTPUT);

  digitalWrite(BUTTON1_PIN,HIGH);
  digitalWrite(BUTTON2_PIN,HIGH);




  //start serial with midi baudrate 31250
  Serial.begin(31250);     

  pinMode( RX_PIN, INPUT );
  pinMode( TX_PIN, OUTPUT);

  pinMode( GATE_LED_PIN, OUTPUT );

  lcd.clear();
  lcd.backlight(20);
  digitalWrite( GATE_LED_PIN, HIGH );
  lcd.displayLine( 1, "MIDI2CV Conv.   ");
  lcd.displayLine( 2, "Version 1.3     ");
  delay( 5000 );
  digitalWrite( GATE_LED_PIN, LOW );

 initialze_settings();

  update_lcd();

  MIDI.setHandleNoteOn(handle_note_on);  
  MIDI.setHandleNoteOff(handle_note_off);
  MIDI.setHandlePitchBend(handle_pitch_bend);
  MIDI.setHandleControlChange(handle_control_change);
  // Initiate MIDI communications, listen to current channel
  MIDI.begin(settings.current_channel);




#ifdef LCD_DEBUGGING
  lcd.clear();
#endif

}

//////////////////////////////////////////////////////////////////////



void loop() {

  // If we are gliding then update the transition now:
  update_glide_voltage();

  // Check for button presses:
  handle_button_one();
  handle_button_two();

  //*************** MIDI IN ***************//
  // Call MIDI.read the fastest you can for real-time performance.
  MIDI.read();

}

// This function is called whenever a note on or note off happens. It looks at the list of notes and 
// plays the last note received, or turns the gate off if there's none:
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
        if( !settings.legato_mode ) 
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

// Set the DAC voltage corresponding to the note
void set_voltage( byte note, byte velocity ) {


  float vel_voltage;
  glide_start_voltage = note_voltage;

  note_voltage = (float)(note - (byte)LOWEST_NOTES[settings.lowest_note_index] ) 
    * SEMITONE_INC;  
  if( note_voltage < 0.0 ) {
    note_voltage = 0.0; 
  }
  if( note_voltage > 5.0 ) {
    note_voltage = 5.0; 
  }

  if( settings.glide_value > 0 ) {
    doing_glide = true;
    glide_target_voltage = note_voltage;
    glide_target = millis() + settings.glide_value;
  } 
  else {
    // Just set the note value:

    dac.setVoltage( note_voltage, CHANNEL_A );
    if( settings.output_mode == MODE_OUTPUT_AB ) {
      dac.setVoltage( note_voltage, CHANNEL_B );
    }
  }


  if( settings.output_mode == MODE_OUTPUT_APBV ) {  // Send velocity as voltage on Channel B

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

// Utility function that checks for button press:
char button(char button_num)
{
  return (!(digitalRead(button_num)));
}

// Handle the mode button:
void handle_button_one ( void ) {
  // The mode button - simply update the main mode and redisplay in the LCD
  //
  if( button( BUTTON1_PIN ) ) {
    delay(100);
    if( button( BUTTON1_PIN ) ) {
      settings.current_mode += 1;
      if( settings.current_mode > MAX_MODE ) {
        settings.current_mode = 0;
      }
      update_lcd();
      save_settings(); // Write current settings to EEPROM
      while( button( BUTTON1_PIN ) ) {

      }

    }
  }

}

// Handle the change value button:
void handle_button_two ( void ) {
  // The value button - what we do here depends on the value of the main mode:

  if( button( BUTTON2_PIN ) ) {
    delay(100);
    if( button( BUTTON2_PIN ) ) {


      if( settings.current_mode == MODE_OUTPUT) {
        settings.output_mode += 1;
        if( settings.output_mode > MAX_OUTPUT_MODE) {
          settings.output_mode = 0;
        }


        if( settings.output_mode == MODE_OUTPUT_TUNEA440 ) {
          handle_note_on( settings.current_channel, A440, 128 );
        } 
        else if( settings.output_mode == MODE_OUTPUT_TUNEA220 ) {
          handle_note_off( settings.current_channel, A440, 0 );
          handle_note_on( settings.current_channel, A220, 128 );
        } 
        else if( settings.output_mode == MODE_OUTPUT_TUNEA880 ) {
          handle_note_off( settings.current_channel, A220, 0 );
          handle_note_on( settings.current_channel, A880, 128 );
        } 
        else {
          handle_note_off( settings.current_channel, A880, 0 );
        }

      }
    }

    if( settings.current_mode == MODE_CHANNEL ) {
      settings.current_channel += 1;
      if( settings.current_channel > 12 ) {
        settings.current_channel = 0;
      }
      MIDI.setInputChannel( settings.current_channel );

    }

    if( settings.current_mode == MODE_GLIDE) {
      settings.glide_value += 5;
      if( settings.glide_value > 127 ) {
        settings.glide_value = 0;
      }
    }

    if( settings.current_mode == MODE_LOWEST_NOTE ) {
      settings.lowest_note_index += 1;
      if( settings.lowest_note_index > 4 ) {
        settings.lowest_note_index = 0;
      }
    }

    if( settings.current_mode == MODE_LEGATO ) {
      settings.legato_mode = ! settings.legato_mode;
    }

    update_lcd();
    save_settings(); 
    while( button( BUTTON2_PIN ) ) {
    }


  }
}


// Update the display - what is displayed depends on the main mode:
void update_lcd( void ) {
  // In ouput mode
  if( settings.current_mode == MODE_OUTPUT ) {
    lcd.displayLine( 1, "Output Mode:" );
    if( settings.output_mode == MODE_OUTPUT_AB ) {
      lcd.displayLine( 2, "Pitch to A + B" );
    } 
    else if (settings.output_mode == MODE_OUTPUT_APBE ) {
      lcd.displayLine( 2, "Pitch: A, Exp: B");
    } 
    else if (settings.output_mode == MODE_OUTPUT_APBV ) {
      lcd.displayLine( 2, "Pitch: A, Vel: B");
    } 
    else if (settings.output_mode == MODE_OUTPUT_TUNEA440 ) {
      lcd.displayLine( 2, "Tune 440Hz ");
    }
    else if (settings.output_mode == MODE_OUTPUT_TUNEA220 ) {
      lcd.displayLine( 2, "Tune 220Hz ");
    }
    else if (settings.output_mode == MODE_OUTPUT_TUNEA880 ) {
      lcd.displayLine( 2, "Tune 880Hz ");
    }
  }

  if( settings.current_mode == MODE_CHANNEL ) {
    snprintf(buff, 16, "Channel :%2d", int(settings.current_channel) );
    lcd.displayLine( 1, buff );
    lcd.displayLine( 2, "" );
  }

  if( settings.current_mode == MODE_GLIDE ) {
    snprintf(buff, 16, "Glide: %4d", int(settings.glide_value) );
    lcd.displayLine( 1, buff );
    lcd.displayLine( 2, "" );
  }

  if( settings.current_mode == MODE_LOWEST_NOTE ) {
    snprintf(buff, 16, "Low Note: C%1d", int(settings.lowest_note_index) );
    lcd.displayLine( 1, buff );
    lcd.displayLine( 2, "" );
  }

  if( settings.current_mode == MODE_LEGATO ) {
    if( settings.legato_mode )
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

// Calculate the pitch bend value. Max is +/- two full tones.
void handle_pitch_bend( byte channel, int bend  ) {
  float voltage_offset = float(bend-PITCH_WHEEL_CENTERED) /MIDI_PITCHBEND_MAX * SEMITONE_INC * 4;
  float voltage = note_voltage + voltage_offset;
  if( voltage < 0.0 )
    voltage = 0.0;
  if( voltage > 5.0 )
    voltage = 5.0;
  dac.setVoltage( voltage, CHANNEL_A );
  if( settings.output_mode == MODE_OUTPUT_AB ) {
    dac.setVoltage( voltage, CHANNEL_B );
  }

}

// Control change is mod wheel and the rotary controller used for Glide:
void handle_control_change( byte channel, byte number, byte value  ) {

  if( number == MOD_WHEEL ) {
    handle_mod_wheel(value);
  }
  if( number == GLIDE_CONTROLLER ) {
    handle_glide_control(value); 
  }
}

// Calculate the mod wheel offset to the channel B voltage
void handle_mod_wheel( byte value ) {

  float voltage = (float(value)/MOD_WHEEL_MAX)*10.0;

  if( settings.output_mode == MODE_OUTPUT_APBE ) {
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
  settings.glide_value = glide;
  update_lcd();
  doing_glide= false; // turn off the current glide
}

// Apply the glide voltage - this is done linearly between the starting and target voltage:
void update_glide_voltage( void ) {

  if( doing_glide == false )
    return;

  if( millis() > glide_target ) {
    doing_glide = false;
    return; 
  }

  // We are still doing glide so calculate the current note voltage:
  unsigned long time_remaining = glide_target - millis();

  float fraction = float(time_remaining)/float(settings.glide_value);

  note_voltage = glide_target_voltage - (glide_target_voltage-glide_start_voltage)*fraction;
  dac.setVoltage( note_voltage, CHANNEL_A );
  if( settings.output_mode == MODE_OUTPUT_AB ) {
    dac.setVoltage( note_voltage, CHANNEL_B );
  }
}

void initialze_settings(){
  // Read the settings from EEPROM:
  eeprom_read_block((void*)&settings, (void*)0, sizeof(settings));

  // If the Settings had not actually been saved then initialize them now and save them:
  if( !settings.settings_saved ) {
    settings.current_mode = MODE_OUTPUT;
    settings.output_mode = MODE_OUTPUT_APBE;
    settings.glide_value = 0;
    settings.current_channel = 1;
    settings.lowest_note_index = 1;
    settings.legato_mode = false;
    settings.settings_saved = true;
    save_settings();
  }

}

void save_settings() {
  // Write settings to EEProm
  settings.settings_saved = true;
  eeprom_write_block((const void*)&settings, (void*)0, sizeof(settings));

}






















































