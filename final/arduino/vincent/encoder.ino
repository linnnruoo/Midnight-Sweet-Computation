/*
 * Setup and start codes for external interrupts and
 * pullup resistors.
 *
 */
// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT() {
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EIMSK = 0b00000011;  // Activate INT0 and INT1
  EICRA = 0b00001010; // set INT0 and INT1 to trigger interrupt on falling edge
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect) {
  leftISR();
}

ISR(INT1_vect) {
  rightISR();
}
// Implement INT0 and INT1 ISRs above.

// Enable pull up resistors on pins 2 and 3
void enablePullups() {
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011; // Set port 2 and 3 as input
  PORTD |= 0b00001100;    // Set port 2 and 3 as HIGH output  
}

void leftISR() {
  switch (dir) {
    case FORWARD:
      leftForwardTicks++;
      forwardDist = (unsigned long)((float)leftForwardTicks/COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case BACKWARD:
      leftReverseTicks++;
      reverseDist = (unsigned long)((float)leftReverseTicks/COUNTS_PER_REV * WHEEL_CIRC);
      break;
    case LEFT:
      leftReverseTicksTurn++;
      break;
    case RIGHT:
      leftForwardTicksTurn++;
      break;
  }

  //Serial.println(leftForwardTicks);
  
  //leftTicks++;
  //leftRevs = leftTicks / COUNTS_PER_REV;
  //Serial.print("LEFT: ");
  //Serial.println(leftRevs);
}

void rightISR() {
  switch (dir) {
    case FORWARD:
      rightForwardTicks++;
      break;
    case BACKWARD:
      rightReverseTicks++;
      break;
    case LEFT:
      rightForwardTicksTurn++;
      break;
    case RIGHT:
      rightReverseTicksTurn++;
      break;
  }

  //Serial.println(rightForwardTicks);
  //rightTicks++;
  //rightRevs = rightTicks / COUNTS_PER_REV
  //Serial.print("RIGHT: ");
  //Serial.println(rightRevs);
}
