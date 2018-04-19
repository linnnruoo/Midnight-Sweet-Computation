/*
 *
 * Vincent Communication Routines (Write)
 *
 */

void sendResponse(TPacket *packet) {
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;
    
  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

void sendStatus() {
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  
  TPacket statusPacket;
  statusPacket.packetType=PACKET_TYPE_RESPONSE;
  statusPacket.command=RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurn;
  statusPacket.params[5] = rightForwardTicksTurn;
  statusPacket.params[6] = leftReverseTicksTurn;
  statusPacket.params[7] = rightReverseTicksTurn;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;

  /////all the sensors' readings/////
  statusPacket.params[10] = ultraInCm; //ultrasound sensor
  statusPacket.params[11] = leftIRreading;
  statusPacket.params[12] = rightIRreading;

  
  sendResponse(&statusPacket);
}

void sendMessage(const char *message) {
  // Sends text messages back to the Pi. Useful
  // for debugging.
  
  TPacket messagePacket;
  messagePacket.packetType=PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void sendBadPacket() {
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
    
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);  
}

void sendBadChecksum() {
  // Tell the Pi that it sent us a packet with a bad
  // checksum.
    
  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand() {
  // Tell the Pi that we don't understand its
  // command sent to us.
    
  TPacket badCommand;
  badCommand.packetType=PACKET_TYPE_ERROR;
  badCommand.command=RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse() {
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK() {
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

////////////////////////////////////////////////////
void sendDone(){
  TPacket donePacket;
  donePacket.packetType = PACKET_TYPE_RESPONSE;
  donePacket.command = RESP_DONE;
  sendResponse (&donePacket);
}
////////////////////////////////////////////////////

