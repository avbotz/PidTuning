#include <assert.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <thread>
#include <mutex>

#include <plstream.h>

#define IM_HERE printf("%d\n",__LINE__); fflush(stdout); 

struct MBED
{
	struct termios options;
	int serialPort;
	
	char serialBuf[256];
	char messageBuf[10];
	int mBufIndex;
};

MBED mbed;

int openPort();
void readStdin();
int writeSerial(const char* str);
int reset();
void processData();

char input[256];
bool new_input = false;
std::mutex in_mut;

int new_value;
bool new_value_ready = false;

bool redraw_graph = true;


#define HISTORY_VALUES 50
PLFLT history[HISTORY_VALUES];
PLFLT xcoord[HISTORY_VALUES];
PLFLT xzero[2] = {0.0f, HISTORY_VALUES-1};
PLFLT yzero[2] = {100.0f, 100.0f};


plstream* pls;

int main()
{
	mbed.mBufIndex = 0;
	
	pls = new plstream();
	pls->star(1,1);
	pls->col0(1);
	pls->env(0, HISTORY_VALUES-1, 0, 200, 0, 0);
	for (int i = 0; i < HISTORY_VALUES; i++)
	{
		history[i] = 0.0f;
		xcoord[i] = i;
	}
	openPort();
	std::thread t(&readStdin);
	while (true)
	{
		if (new_input)
		{
			int length = strlen(input);
			std::string message = "";
			std::string control = "";
			for (int i = 0; i < length; i++)
			{
				//split up the input into the message and control characters
				if ('0' <= input[i] <= '9' || 'a' <= input[i] <= 'z')
				{
					message += input[i];
				}
				else if ('A' <= input[i] <= 'Z')
				{
					control += input[i];
				}
			}
			if (message != "")
			{
				//send the message
				writeSerial(message.c_str());
			}
			if (control != "")
			{
				//parse the control characters
				for (int i = 0; i < control.length(); i++)
				{
					switch (control[i])
					{
					case 'B': reset(); break;
					}
				}
			}
		}
		
		int charsRead = read(mbed.serialPort, mbed.serialBuf, 256);
	//	printf(mbed.serialBuf);
		if (charsRead)
		{
			for (int i = 0; i < charsRead; i++)
			{
				mbed.messageBuf[mbed.mBufIndex] = mbed.serialBuf[i];
				if (mbed.serialBuf[i] == '\n')
				{
					processData();
					if (new_value_ready)
					{
						//if new value is ready add it to the graph
						for (int i = 0; i < HISTORY_VALUES-1; i++)
						{
							history[i] = history[i+1];
						}
						history[HISTORY_VALUES-1] = new_value;
						new_value_ready = false;
						redraw_graph = true;
					}
					
					memset(mbed.messageBuf, 0, 20);	//Clear the buffer after reading data
					mbed.mBufIndex = 0;
				}
				else
				{
					mbed.mBufIndex = (mbed.mBufIndex + 1) % 20;
				}
			}
		}
		
		if (redraw_graph)
		{
			pls->clear();
//			pls->col0(3);
//			pls->box("bcg", 20, 5, "bcg", 20, 5);
			pls->col0(2);
			pls->line(HISTORY_VALUES, xcoord, history);
			pls->col0(4);
			pls->line(2, xzero, yzero);
			redraw_graph = false;
		}
	}
}

void processData()
{
	int value = 0;
	int  i = 0;
	while (mbed.messageBuf[i] != 0 && i < 10)
	{
		if ('0' <= mbed.messageBuf[i] && mbed.messageBuf[i] <= '9')
		{
			value = 10*value + (mbed.messageBuf[i] - '0');
		}
		i++;
	}
	if (0 <= value && value <= 200)
	{
		new_value = value;
		new_value_ready = true;
	}
	
	return;
}

int openPort()
{
	//Open the first USB-serial converter
	//read-write, prevents file (terminal device) from becoming controlling device of eva, open the connection immediately
	mbed.serialPort = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
	//If opening the port failed
	if (mbed.serialPort == -1)
	{
		perror("Error: unable to open serial port /dev/ttyACM0");
		return -1;
	}
	//get paramaters associated to fd
	tcgetattr(mbed.serialPort, &mbed.options);
	//Baud rate = 115200
	cfsetispeed(&mbed.options, B115200);
	cfsetospeed(&mbed.options, B115200);
	//ignore modem control lines, enable receiver
	mbed.options.c_cflag |= (CLOCAL | CREAD);
	//disable parity, 1 stop bit, remove any previous character size setting
	mbed.options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
	//8 bit characters
	mbed.options.c_cflag |= CS8;
	//non-canonical input (dont wait for newline to send), disable echo, cant use erase character, dont forward beagleboard's signals to mbed
	mbed.options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	//disable implementation-defined output processing
	mbed.options.c_oflag &= ~OPOST;
	//dont wait for multiple characters to read, read every one
	mbed.options.c_cc[VMIN]  = 0;
	//no timeout for read
	mbed.options.c_cc[VTIME]  = 0;
	//set EVA as the receiver of SIGIO and SIGURG signals
	fcntl(mbed.serialPort, F_SETOWN, getpid());
	//read doesnt block
	fcntl(mbed.serialPort, F_SETFL, FNDELAY);
	//set the attributes effective immediately
	return tcsetattr(mbed.serialPort, TCSANOW, &mbed.options);
}

int writeSerial(const char* str)
{
	return write(mbed.serialPort, str, strlen(str));
}

void readStdin()
{
	char in[256];
	while (true)
	{
		gets(in);
		in_mut.lock();
		strcpy(input, in);
		new_input = true;
		in_mut.unlock();
		usleep(10*1000);
	}
}

int reset()
{
	assert(tcsendbreak(mbed.serialPort, 0) == 0);
	usleep(500*1000);
}