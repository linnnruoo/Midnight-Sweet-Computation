#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"

///////////////////////////////
#include <iostream>
#include <stack>
using namespace std;

stack<TPacket> backtrackstack;
stack<char> movementCommand;

int startBacktrack=0;
//////////////////////////////

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

int exitFlag=0;
sem_t _xmitSema;

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- VINCENT STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);

	////////////////////////////////////////////////////////
    printf("Ultrasonic Sensor:\t\t%d\n", packet->params[10]);
    printf("LeftIR reading:\t\t\t%d\n", packet->params[11]);
    printf("RightIR reading:\t\t%d\n", packet->params[12]);
    ////////////////////////////////////////////////////////

    printf("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printf("Command OK\n");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
			printf("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n");
		break;

		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n");
		break;

		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n");
		break;

		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n");
		break;

		default:
			printf("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Vincent: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	} 
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

void flushInput()
{
	char c;

	while((c = getchar()) != '\n' && c != EOF);
}



void getParams(TPacket *commandPacket)
{
	//int first, second;
	if(startBacktrack==0){
		printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
		printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
		scanf("%d %d", &commandPacket->params[0], &commandPacket->params[1]);
		flushInput();
	}
}

void sendCommand(char command)
{
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;
	
	switch(command)
	{
		case 'f':
		case 'F':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_FORWARD;
			//cout << "what is this in f:  "<<commandPacket.command<<endl; //checking
			sendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_REVERSE;
			//cout << "what is this in b:  "<<commandPacket.command<<endl; //che
			sendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_LEFT;
			sendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			getParams(&commandPacket);
			commandPacket.command = COMMAND_TURN_RIGHT;
			sendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;

		case 'q':
		case 'Q':
			exitFlag=1;
			break;

		//////////////////
		case 'z':
		case 'Z':
			startBacktrack=1;
			break;
		/////////////////

		default:
			printf("Bad command\n");

	}

	///////////////////////////////////////////////
	if(startBacktrack==0 && !(command=='z' || command=='Z' || command=='C' || command=='c' ||
		command=='g' || command=='G' || command=='Q' || command=='q')) {
		
		movementCommand.push(command);
		cout << "this is pushed into movement" << command <<endl; //for checking

		backtrackstack.push(commandPacket);
		cout << "pushed into backtrackstack"<<endl; //for checking

	}
	//////////////////////////////////////////////

}

void sendCommand2(char command)
{
	TPacket temp = backtrackstack.top();
	

	switch(command)
	{
		case 'f':
		case 'F':
			getParams(&temp);
			//cout << "before "<<temp.command<<endl;
			temp.command = COMMAND_REVERSE;
		//	cout << "after "<<temp.command<<endl;
		//	cout << "the param[0] "<< temp.params[0] << endl;
		//	cout << "the param[1] "<< temp.params[1] << endl;
			sendPacket(&temp);
			break;

		case 'b':
		case 'B':
			getParams(&temp);
			temp.command = COMMAND_FORWARD;
			sendPacket(&temp);
			break;

		case 'l':
		case 'L':
			getParams(&temp);
			temp.command = COMMAND_TURN_RIGHT;
			sendPacket(&temp);
			break;

		case 'r':
		case 'R':
			getParams(&temp);
			temp.command = COMMAND_TURN_LEFT;
			sendPacket(&temp);
			break;

		case 's':
		case 'S':
			temp.command = COMMAND_STOP;
			sendPacket(&temp);
			break;

		

		default:
			printf("Bad command\n");

	}

	
}

int main()
{
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);

	while(!exitFlag)
	{
		char ch;
		printf("Command (f=forward, b=reverse, l=turn left, r=turn right, s=stop, c=clear stats, g=get stats, q=exit, z=backtracking)\n");
		scanf("%c", &ch);

		// Purge extraneous characters from input stream
		flushInput();

		sendCommand(ch);
		
		///////////////////////
		
		if(startBacktrack==1) {
			cout <<"backtrack starts"<<endl; //for checking
			
			while(!backtrackstack.empty()){
				
				sendCommand2(movementCommand.top());
				cout << "movementCommand is poped: " <<movementCommand.top() <<endl; //for checking
				movementCommand.pop();
				backtrackstack.pop();

			//	flushInput();
			//	flushInput2();
				sleep(2);
			//	cin.get();
			}

			break;
		}
		///////////////////////


	}

	////////////////////////
/*	if(startBacktrack==1) {
		while(!backtrackstack.empty()){
			flushInput();
			
			sendCommand2(movementCommand.top());
			cout << "movementCommand is poped: " <<movementCommand.top() <<endl; //for checking
			movementCommand.pop();
			backtrackstack.pop();
		}
	}
*/	////////////////////////

	printf("Closing connection to Arduino.\n");
	endSerial();
}
