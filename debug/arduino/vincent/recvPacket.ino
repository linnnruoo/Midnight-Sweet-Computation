/*
 *
 * Vincent Communication Routines (Read)
 *
 */

TResult readPacket(TPacket *packet) {
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".
    
  char buffer[PACKET_SIZE];
  int len;
    
  len = readSerial(buffer);
    
  if(len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}


void handleCommand(TPacket *command) {
  switch(command->command) {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
      sendOK();
      forward((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_REVERSE:
      sendOK();
      reverse((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_LEFT:
      sendOK();
      left((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_TURN_RIGHT:
      sendOK();
      right((float) command->params[0], (float) command->params[1]);
      break;
      
    case COMMAND_STOP:
      stop();
      sendDone(); //edited
      break;
    
    case COMMAND_GET_STATS:
      sendStatus();
      sendDone(); //edited
      break;
    
    case COMMAND_CLEAR_STATS:
      clearCounters();
      sendOK();
      sendDone(); //edited
      break;

    ///////////////////////////////////////////////////////////////////////////////////////
    case COMMAND_MARK:
      sendOK();
      stop();
      startBuzzer();
      sendDone(); //edited
      break;
    //////////////////////////////////////////////////////////////////////////////////////
    
      
    default:
      sendBadCommand();
      sendDone();
    }
}


void handlePacket(TPacket *packet) {
  switch(packet->packetType) {
    case PACKET_TYPE_COMMAND:
      handleCommand(packet);
      break;
      
    case PACKET_TYPE_RESPONSE:
      break;
            
    case PACKET_TYPE_ERROR:
      break;
            
    case PACKET_TYPE_MESSAGE:
      break;
            
    case PACKET_TYPE_HELLO:
      break;
  }
}
