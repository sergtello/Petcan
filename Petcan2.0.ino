#include <stdio.h>
#include "soniclib.h"      // Chirp SonicLib sensor API definitions
#include "chirp_board_config.h" // required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"
#include "chirp_bsp.h"      // board support package function definitions
#include "chirp_esp32.h"


void setup() {
  // put your setup code here, to run once:
  chbsp_reset_assert();
}

void loop() {
  // put your main code here, to run repeatedly:

}
