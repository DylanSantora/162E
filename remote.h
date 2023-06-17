#include <IRremote.h>

const int RECV_PIN = 9;
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup_remote(){
  irrecv.enableIRIn();
  irrecv.blink13(true);
}

int get_IR_input() {
    if (irrecv.decode(&results)){
        unsigned long x = (results.value);
        if (x == 16724175) {
          return 1;
        }
        else if (x == 16718055) {
          return 2;
        }
        else if (x == 16743045) {
          return 3;
        }
        else if (x == 16716015) {
          return 4;
        }
        else if (x == 16726215) {
          return 5;
        }
        else if (x == 16734885) {
          return 6;
        }
        else {
          Serial.println(x);
        }
        irrecv.resume();
  }
  else {
    return -1;
  }
}