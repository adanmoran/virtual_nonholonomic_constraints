#include <RX24F.h>

void setup() {
  RX24F.begin(1000000, 2);
  RX24F.setBD(1, 500000);
}

void loop() {
  // put your main code here, to run repeatedly:

}
