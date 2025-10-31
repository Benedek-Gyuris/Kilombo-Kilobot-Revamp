#include <kilolib.h>

/* Optional: set your own guesses if you haven't saved calibration.
   You can comment these two lines out if you already calibrated in KiloGUI. */
static const uint8_t TURN_LEFT_DUTY  = 90;  // typical 60–75 works well
/* kilo_turn_left will be initialized from EEPROM if present. */

void setup(void)
{
    // If you have no calibration saved yet, seed turn-left manually:
    kilo_turn_left = TURN_LEFT_DUTY;

    set_color(RGB(3,0,0));  // red = turning left
    spinup_motors();        // brief full-speed to overcome stiction
    set_motors(kilo_turn_left, 0);
}

void loop(void)
{
    // keep turning; nothing else to do
    // NOTE: Avoid delay() here; KiloGUI Run/Pause stays responsive.
}

int main(void)
{
    kilo_init();
    kilo_start(setup, loop);
    return 0;
}
