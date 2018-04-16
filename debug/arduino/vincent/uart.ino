/*
 * Setup and start codes for serial communications
 *
 */
#define XMIT_SIZE      128
#define RECV_SIZE      128

// Variables for Serial Communication
static TBuffer _recvBuffer, _xmitBuffer;

// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial() {
  // To replace later with bare-metal.
  //Serial.begin(9600);
  UBRR0L = 103;
  UBRR0H = 0;


  //USART to work at 9600 in 8N1 format
  UCSR0C = 0b00000110; // Set USART to Asynchronous mode, N partity mode, 1 stop bit and 8-bit character size
  UCSR0A = 0;
  
  // Set baud rate to 9600
    
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial() {
  // Empty for now. To be replaced with bare-metal code
  // later on. 
  
  // Start the serial port
  // Enable RXC and UDRIE0
  // Enable USART receiver and transmitter
  // UCSR0B = 0b10111000;

  // Using polling for transmitting
  UCSR0B = 0b10011000; 
}


void setupBuffers()
{
  // Initialize the receive and transmit buffers.
  initBuffer(&_recvBuffer, RECV_SIZE);
  //initBuffer(&_xmitBuffer, XMIT_SIZE);
}

ISR(USART_RX_vect) {
  // Write received data
  unsigned char data = UDR0;
  writeBuffer(&_recvBuffer, data);
}

/*
ISR(USART_UDRE_vect) {
  unsigned char data;
  TBufferResult result = readBuffer(&_xmitBuffer, &data);
  if(result == BUFFER_OK)
    UDR0 = data;
  else 
    if (result == BUFFER_EMPTY)
      UCSR0B &= 0b11011111;
}
*/

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(unsigned char *line) {

  /*
  int count=0;

  while(Serial.available())
    buffer[count++] = Serial.read();

  return count;
  
  */
  int count = 0;
  
  TBufferResult result;
  
  do {
    result = readBuffer(&_recvBuffer, &line[count]);
    if (result == BUFFER_OK)
      count++;
  } while (result == BUFFER_OK);
  
  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const unsigned char *line, int len) {
  //Serial.write(buffer, len);

  /*
  TBufferResult result = BUFFER_OK;
  int i;
  for (int i=1; i<len && result == BUFFER_OK; i++)
    result = writeBuffer(&_xmitBuffer, buffer[i]);
  UDR0 = buffer[0];
  UCSR0B |= 0b00100000;
  */

  // Using polling to write to Pi
  while (len--){
      while ((UCSR0A & 0b00100000) == 0);
      UDR0 = *line;
      line++;
  }
}
