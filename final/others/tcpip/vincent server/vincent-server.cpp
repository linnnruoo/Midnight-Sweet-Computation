#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <sys/types.h>
#include <errno.h>
#include <string.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
#include "netconstants.h"

#include <iostream>
//#include <stack>
#include <vector>
using namespace std;

//stack<TPacket> backtrackstack;
//stack<char> movementCommand;

//stack<TPacket> backup_packet;
//stack<char> backup_movement;

vector<TPacket> backtrack_vec;
//vector<TPacket>::iterator it_backtrack = backtrack_vec.begin();

vector<char> movement_vec;
vector<char>::reverse_iterator it_movement;// = movement_vec.begin();

int doneFlag = 0;
int startBacktrack=0;
int helloFlag = 0;

//backup connection
int client_counter = 0;

#define SERVER_PORT			5000
#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

// Our network buffer consists of 1 byte of packet type, and 128 bytes of data
#define BUF_LEN				129

// This variable shows whether a network connection is active
// We will also use this variable to prevent the server from serving
// more than one connection, to keep connection management simple.

static volatile int networkActive;

// The listener and connection file descriptors
int listenfd, connfd;

/*

	Vincent Serial Routines to the Arduino

*/

// Prototype for sendNetworkData
void sendNetworkData(const char *, int);

void handleErrorResponse(TPacket *packet)
{
	printf("UART ERROR: %d\n", packet->command);
	char buffer[2];
	buffer[0] = NET_ERROR_PACKET;
	buffer[1] = packet->command;
	sendNetworkData(buffer, sizeof(buffer));
}

void handleMessage(TPacket *packet)
{
	char data[33];
	printf("UART MESSAGE PACKET: %s\n", packet->data);
	data[0] = NET_MESSAGE_PACKET;
	memcpy(&data[1], packet->data, sizeof(packet->data));
	sendNetworkData(data, sizeof(data));
}

void handleStatus(TPacket *packet)
{
	char data[65];
	printf("UART STATUS PACKET\n");
	data[0] = NET_STATUS_PACKET;
	memcpy(&data[1], packet->params, sizeof(packet->params));
	sendNetworkData(data, sizeof(data));
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			//helloFlag = 1;
			char resp[2];
			printf("Command OK\n");
			resp[0] = NET_ERROR_PACKET;
			resp[1] = RESP_OK;
			sendNetworkData(resp, sizeof(resp));
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		/////////////////////////////////////
		case RESP_DONE:
			printf("FINALLY DONE YAY\n");
			doneFlag = 1;

			resp[0] = NET_ERROR_PACKET;
			resp[1] = RESP_DONE;
			sendNetworkData(resp, sizeof(resp));
		break;
		////////////////////////////////////

		default:
			printf("Boo! Arduino is confused. So are we.\n");
	}
}


void handleUARTPacket(TPacket *packet)
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

		/*case PACKET_TYPE_HELLO:
				helloFlag = 1;
			break;
	*/}
}


void uartSendPacket(TPacket *packet) //send to arduino???
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

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

void *uartReceiveThread(void *p) //receive from client?
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
				handleUARTPacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				} // result
		} // len > 0
	} // while
}

/*

	Vincent Network Routines

*/


void sendNetworkData(const char *data, int len) //write to client
{
	// Send only if network is active
	if(networkActive)
	{
		printf("WRITING TO CLIENT\n");
		int c = write(connfd, data, len);

		// Network is still active if we can write more then 0 bytes.
		networkActive = (c > 0);
	}
}

void backtrack_process(){
	printf("BACKTRACKING STARTS\n");

	it_movement = movement_vec.rbegin();
	
	for(auto it_backtrack=backtrack_vec.rbegin(); it_backtrack!=backtrack_vec.rend(); it_backtrack++){
		
		doneFlag = 0;
		
		TPacket temp = (*it_backtrack);
		char ch = (*it_movement);

		switch(ch)
		{
			case 'f':
			case 'F':
				temp.command = COMMAND_REVERSE;
				uartSendPacket(&temp);
				break;

			case 'b':
			case 'B':
				temp.command = COMMAND_FORWARD;
				uartSendPacket(&temp);
				break;

			case 'l':
			case 'L':
				temp.command = COMMAND_TURN_RIGHT;
				uartSendPacket(&temp);
				break;

			case 'r':
			case 'R':
				temp.command = COMMAND_TURN_LEFT;
				uartSendPacket(&temp);
				break;

			case 's':
			case 'S':
				temp.command = COMMAND_STOP;
				uartSendPacket(&temp);
				break;

			case 'm':
			case 'M':
				temp.command = COMMAND_MARK;
				uartSendPacket(&temp); //this packet will be sent to arduino to produce a sound
				break;

			default:
				printf("Bad command during backtracking\n");

		}//switch

			while(doneFlag==0) {};
			
			//it_backtrack++;
			it_movement++;


		
	}//while

}



void handleCommand(const char *buffer)
{	
	doneFlag = 0;
	// The first byte contains the command
	char cmd = buffer[1];
	uint32_t cmdParam[2];

	// Copy over the parameters.
	memcpy(cmdParam, &buffer[2], sizeof(cmdParam));

	TPacket commandPacket;

	commandPacket.params[0] = cmdParam[0];
	commandPacket.params[1] = cmdParam[1];

	printf("COMMAND RECEIVED: %c %d %d\n", cmd, cmdParam[0], cmdParam[1]);
	
	switch(cmd)
	{
		case 'f':
		case 'F':
			commandPacket.command = COMMAND_FORWARD;
			uartSendPacket(&commandPacket);
			break;

		case 'b':
		case 'B':
			commandPacket.command = COMMAND_REVERSE;
			uartSendPacket(&commandPacket);
			break;

		case 'l':
		case 'L':
			commandPacket.command = COMMAND_TURN_LEFT;
			uartSendPacket(&commandPacket);
			break;

		case 'r':
		case 'R':
			commandPacket.command = COMMAND_TURN_RIGHT;
			uartSendPacket(&commandPacket);
			break;

		case 's':
		case 'S':
			commandPacket.command = COMMAND_STOP;
			uartSendPacket(&commandPacket);
			break;

		case 'c':
		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			uartSendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			uartSendPacket(&commandPacket);
			break;

		//////////////////////////
		case 'm':
		case 'M':
			commandPacket.command = COMMAND_MARK;		
			break;

		case 'z':
		case 'Z':
			startBacktrack = 1;
			backtrack_process();
			break;
		//////////////////////////
		
		default:
			printf("BAD COMMAND received at SERVER side\n");

	}

	printf("debug1\n");

	while(doneFlag==0 && startBacktrack==0 && !(cmd=='m' || cmd=='M' || cmd=='z' || cmd=='Z')){};
	
	printf("debug2\n");

	if(startBacktrack==0 && !(cmd=='z' || cmd=='Z' || cmd=='C' || cmd=='c' || cmd=='g' || cmd=='G' || cmd=='Q' || cmd=='q')) {
		
		/****************STACK*********************************/
		/*movementCommand.push(cmd);
		cout << "this is pushed into movement stack " << cmd <<endl; //for checking
		backtrackstack.push(commandPacket);
		cout << "this is pushed into backtrackstack"<<endl; //for checking	
		*/
		/****************BACKUP********************************/
		//backup_movement.push(cmd);
		//backup_packet.push(TPacket);

		/****************VECTOR********************************/
		movement_vec.push_back(cmd);
		backtrack_vec.push_back(commandPacket);

	}

	startBacktrack = 0;

}

void handleNetworkData(const char *buffer, int len) //from client
{
	
	if(buffer[0] == NET_COMMAND_PACKET) {
		//if(client_counter==0) 
		handleCommand(buffer);
	
	}else if(buffer[0] == NET_DECISION_PACKET){
		//client_counter=0;
		
		if(buffer[1]=='1') {  //restart from phase 1
			backtrack_vec.clear();
			movement_vec.clear();
			//handleCommand(buffer);

		}else if(buffer[1]=='2'){ //restart from phase 2
			//it_backtrack = backtrack_vec.rbegin();
			//it_movement = movement_vec.rbegin();
			backtrack_process();

		}else if(buffer[1]=='3'){ //continue
			handleCommand(buffer);
		}


	}

}

void *netRecvThread(void *p)
{
	int len;

	char buffer[BUF_LEN];
	
	while(networkActive)
	{
		len = read(connfd, buffer, sizeof(buffer));

		// As long as we are getting data, network is active
		networkActive=(len > 0);

		if(len > 0) {

			handleNetworkData(buffer, len);
		}
		else
			if(len < 0)
				perror("ERROR READING NETWORK: ");
	}
	pthread_exit(NULL);
}


void startServer(int portNum)
{	
	struct sockaddr_in serv_addr;
	memset(&serv_addr, 0, sizeof(serv_addr));
	listenfd = socket(AF_INET, SOCK_STREAM, 0);

	if(listenfd < 0)
	{
		perror("Cannot create socket:");
		exit(-1);
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	serv_addr.sin_port = htons(portNum);

	if(bind(listenfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0)
	{
		perror("Unable to bind: ");
		exit(-1);
	}

	printf("Listening..\n");
	if(listen(listenfd, 10) < 0)
	{
		perror("Unable to listen to port: ");
		exit(-1);
	}

	int c = sizeof(struct sockaddr_in);

	while(1)
	{
		struct sockaddr_in client;
		connfd = accept(listenfd, (struct sockaddr *) &client, (socklen_t *)&c);

		if(connfd < 0)
		{
			perror("ACCEPT ERROR: ");
			exit(-1);
		}
		char clientAddress[32];

		// Use inet_ntop to extract client's IP address.
		inet_ntop(AF_INET, &client.sin_addr, clientAddress, 32);

		printf("Received connection from %s\n", clientAddress);

		// New connection. Set networkActive to true
		networkActive=1;

		// Now spawn handler thread
		pthread_t netHandler;
		pthread_create(&netHandler, NULL, netRecvThread, NULL);

		// We wait for the netHandler thread to end, stopping us from accepting new
		// connections. This is just to simplify connection management because we
		// only need to think of one connection at a time, instead of having to resolve
		// which connection sent what command and who should get what response.

		void *result;
		pthread_join(netHandler, &result); //check whether client close connection
		
		printf("Client closed connections.\n");
		// Close the connection
		close(connfd);

		client_counter++;
	}
}

// We have a separate thread for the server since it runs infinitely
void *serverThread(void *p)
{
	startServer(SERVER_PORT);
	pthread_exit(NULL);
}

void sendHello()
{
	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	uartSendPacket(&helloPacket);
}

int main()
{
	// Start the uartReceiveThread and serverThread. The network listener thread
	// is started by serverThread

	pthread_t serThread, netThread;

	printf("\nVINCENT REMOTE SUBSYSTEM\n\n");

	printf("Opening Serial Port\n");
	// Open the serial port
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);
	printf("Done. Waiting 3 seconds for Arduino to reboot\n");
	sleep(3);

	printf("DONE. Starting Serial Listener\n");
	pthread_create(&serThread, NULL, uartReceiveThread, NULL);
	printf("Starting Network Listener\n");
	pthread_create(&netThread, NULL, serverThread, NULL);
	printf("DONE. Sending HELLO to Arduino\n");
	
	sendHello();

	//while(helloFlag==0);
	
	printf("DONE.\n");


	void *r;
	pthread_join(serThread, &r);
	pthread_join(netThread, &r);
}
