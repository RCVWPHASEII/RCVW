//============================================================================
// Name        : HRIStatusPlugin.cpp
// Author      : Battelle Memorial Institute - Matt Cline (cline@battelle.org)
// Version     :
// Copyright   : Battelle 2016
// Description : HRI Status Plugin - sends out a SPAT message every 10ms
//============================================================================

#include <atomic>
#include <thread>
#include <queue>
#include <chrono>
#include <iostream>
#include <mutex>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <wdt_dio.h>

#include "PluginClient.h"
#include <Clock.h>
#include <EventLogMessage.h>
#include <tmx/j2735_messages/SpatMessage.hpp>
#include <tmx/j2735_messages/BasicSafetyMessage.hpp>
#include <tmx/messages/message_document.hpp>
#include <FrequencyThrottle.h>
#include <System.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <aiousb.h>
#ifdef __cplusplus
}
#endif

using namespace std;
using namespace tmx;
using namespace tmx::utils;
using namespace tmx::messages;

namespace HRIStatusPlugin
{

bool wdtPresent() {
	static atomic<bool> _present { false };
	static atomic<bool> _check { true };
	if (_check.exchange(false)) {
		int ret;
		System::ExecCommand("lsmod | grep -q \"^wdt_dio\"", &ret);
		_present = (ret == 0);
	}
	return _present;
}

class HRIStatusPlugin: public PluginClient
{
public:
	HRIStatusPlugin(std::string);
	virtual ~HRIStatusPlugin();
	int Main();
	// Virtual method overrides.
	void OnConfigChanged(const char *key, const char *value);
	void OnStateChange(IvpPluginState state);
	void HandleBSMMessage(tmx::messages::BsmMessage &msg, routeable_message &routeableMsg);
protected:
	void UpdateConfigSettings();


private:
	// SPaT message to send
	message_container_type _spat;

	//Config Values
	uint64_t _frequency = 100;
	uint64_t _monitorFreq = 100;
	std::atomic<double> _serialDataTimeoutMS;
	string _portName = "";

	mutex _stringConfigLock;

	bool _alwaysSend = true;
	unsigned int _railPinNumber = 0;
	std::vector<std::pair<int, std::string>> _laneMapping;

	bool _isReceivingBsms = false;

	std::atomic<bool> _newConfigValues{false};

	uint64_t _lastSendTime = 0;
	std::mutex _dataLock;

	std::atomic<bool> _trainComing;
	bool _previousState = false;

	bool _muteDsrcRadio = false;

	unsigned char *_serialBuffer = (unsigned char*)malloc(2048);
	int _serialDataLength = 0;
	uint64_t _lastSerialDataTime = 0;
	std::atomic<bool> _sendSPAT{true};
	std::atomic<bool> _serialPinState{false};
	std::atomic<bool> _stopThreads{false};

	//Digital I/O Functions
	bool DioSetup();
	bool GetPinState(int pinNumber);
	int GetBufferPosition(int pin);
	void MonitorRailSignal();
	void SerialPortReader();

	//Spat Generation Functions
	void UpdateMovementState(message_document &md);
	void UpdateTimestamp(message_document &md);

	FrequencyThrottle<int> _throttle;

	std::atomic<int> _serialPortFd{-1};
	int SetInterfaceAttribs (int fd, int speed, int parity);
	void SetBlocking (int fd, int should_block);

	uint16_t GetCrc16(uint16_t crc,uint8_t *data,uint16_t length);
	uint32_t GetCrc32(uint32_t crc,uint8_t *data,uint16_t length);

	uint16_t crc16_table[256] = {
	0x0000,0x1189,0x2312,0x329b,0x4624,0x57ad,0x6536,0x74bf,
	0x8c48,0x9dc1,0xaf5a,0xbed3,0xca6c,0xdbe5,0xe97e,0xf8f7,
	0x1081,0x0108,0x3393,0x221a,0x56a5,0x472c,0x75b7,0x643e,
	0x9cc9,0x8d40,0xbfdb,0xae52,0xdaed,0xcb64,0xf9ff,0xe876,
	0x2102,0x308b,0x0210,0x1399,0x6726,0x76af,0x4434,0x55bd,
	0xad4a,0xbcc3,0x8e58,0x9fd1,0xeb6e,0xfae7,0xc87c,0xd9f5,
	0x3183,0x200a,0x1291,0x0318,0x77a7,0x662e,0x54b5,0x453c,
	0xbdcb,0xac42,0x9ed9,0x8f50,0xfbef,0xea66,0xd8fd,0xc974,
	0x4204,0x538d,0x6116,0x709f,0x0420,0x15a9,0x2732,0x36bb,
	0xce4c,0xdfc5,0xed5e,0xfcd7,0x8868,0x99e1,0xab7a,0xbaf3,
	0x5285,0x430c,0x7197,0x601e,0x14a1,0x0528,0x37b3,0x263a,
	0xdecd,0xcf44,0xfddf,0xec56,0x98e9,0x8960,0xbbfb,0xaa72,
	0x6306,0x728f,0x4014,0x519d,0x2522,0x34ab,0x0630,0x17b9,
	0xef4e,0xfec7,0xcc5c,0xddd5,0xa96a,0xb8e3,0x8a78,0x9bf1,
	0x7387,0x620e,0x5095,0x411c,0x35a3,0x242a,0x16b1,0x0738,
	0xffcf,0xee46,0xdcdd,0xcd54,0xb9eb,0xa862,0x9af9,0x8b70,
	0x8408,0x9581,0xa71a,0xb693,0xc22c,0xd3a5,0xe13e,0xf0b7,
	0x0840,0x19c9,0x2b52,0x3adb,0x4e64,0x5fed,0x6d76,0x7cff,
	0x9489,0x8500,0xb79b,0xa612,0xd2ad,0xc324,0xf1bf,0xe036,
	0x18c1,0x0948,0x3bd3,0x2a5a,0x5ee5,0x4f6c,0x7df7,0x6c7e,
	0xa50a,0xb483,0x8618,0x9791,0xe32e,0xf2a7,0xc03c,0xd1b5,
	0x2942,0x38cb,0x0a50,0x1bd9,0x6f66,0x7eef,0x4c74,0x5dfd,
	0xb58b,0xa402,0x9699,0x8710,0xf3af,0xe226,0xd0bd,0xc134,
	0x39c3,0x284a,0x1ad1,0x0b58,0x7fe7,0x6e6e,0x5cf5,0x4d7c,
	0xc60c,0xd785,0xe51e,0xf497,0x8028,0x91a1,0xa33a,0xb2b3,
	0x4a44,0x5bcd,0x6956,0x78df,0x0c60,0x1de9,0x2f72,0x3efb,
	0xd68d,0xc704,0xf59f,0xe416,0x90a9,0x8120,0xb3bb,0xa232,
	0x5ac5,0x4b4c,0x79d7,0x685e,0x1ce1,0x0d68,0x3ff3,0x2e7a,
	0xe70e,0xf687,0xc41c,0xd595,0xa12a,0xb0a3,0x8238,0x93b1,
	0x6b46,0x7acf,0x4854,0x59dd,0x2d62,0x3ceb,0x0e70,0x1ff9,
	0xf78f,0xe606,0xd49d,0xc514,0xb1ab,0xa022,0x92b9,0x8330,
	0x7bc7,0x6a4e,0x58d5,0x495c,0x3de3,0x2c6a,0x1ef1,0x0f78
	};

	uint32_t crc32_table[256] = {
	0x00000000L, 0x6b6f1b22L, 0x578f860fL, 0x3ce09d2dL, 0x2e4ebc55L, 0x4521a777L, 0x79c13a5aL, 0x12ae2178L,
	0x5c9d78aaL, 0x37f26388L, 0x0b12fea5L, 0x607de587L, 0x72d3c4ffL, 0x19bcdfddL, 0x255c42f0L, 0x4e3359d2L,
	0x386b411fL, 0x53045a3dL, 0x6fe4c710L, 0x048bdc32L, 0x1625fd4aL, 0x7d4ae668L, 0x41aa7b45L, 0x2ac56067L,
	0x64f639b5L, 0x0f992297L, 0x3379bfbaL, 0x5816a498L, 0x4ab885e0L, 0x21d79ec2L, 0x1d3703efL, 0x765818cdL,
	0x70d6823eL, 0x1bb9991cL, 0x27590431L, 0x4c361f13L, 0x5e983e6bL, 0x35f72549L, 0x0917b864L, 0x6278a346L,
	0x2c4bfa94L, 0x4724e1b6L, 0x7bc47c9bL, 0x10ab67b9L, 0x020546c1L, 0x696a5de3L, 0x558ac0ceL, 0x3ee5dbecL,
	0x48bdc321L, 0x23d2d803L, 0x1f32452eL, 0x745d5e0cL, 0x66f37f74L, 0x0d9c6456L, 0x317cf97bL, 0x5a13e259L,
	0x1420bb8bL, 0x7f4fa0a9L, 0x43af3d84L, 0x28c026a6L, 0x3a6e07deL, 0x51011cfcL, 0x6de181d1L, 0x068e9af3L,
	0x60fcb437L, 0x0b93af15L, 0x37733238L, 0x5c1c291aL, 0x4eb20862L, 0x25dd1340L, 0x193d8e6dL, 0x7252954fL,
	0x3c61cc9dL, 0x570ed7bfL, 0x6bee4a92L, 0x008151b0L, 0x122f70c8L, 0x79406beaL, 0x45a0f6c7L, 0x2ecfede5L,
	0x5897f528L, 0x33f8ee0aL, 0x0f187327L, 0x64776805L, 0x76d9497dL, 0x1db6525fL, 0x2156cf72L, 0x4a39d450L,
	0x040a8d82L, 0x6f6596a0L, 0x53850b8dL, 0x38ea10afL, 0x2a4431d7L, 0x412b2af5L, 0x7dcbb7d8L, 0x16a4acfaL,
	0x102a3609L, 0x7b452d2bL, 0x47a5b006L, 0x2ccaab24L, 0x3e648a5cL, 0x550b917eL, 0x69eb0c53L, 0x02841771L,
	0x4cb74ea3L, 0x27d85581L, 0x1b38c8acL, 0x7057d38eL, 0x62f9f2f6L, 0x0996e9d4L, 0x357674f9L, 0x5e196fdbL,
	0x28417716L, 0x432e6c34L, 0x7fcef119L, 0x14a1ea3bL, 0x060fcb43L, 0x6d60d061L, 0x51804d4cL, 0x3aef566eL,
	0x74dc0fbcL, 0x1fb3149eL, 0x235389b3L, 0x483c9291L, 0x5a92b3e9L, 0x31fda8cbL, 0x0d1d35e6L, 0x66722ec4L,
	0x40a8d825L, 0x2bc7c307L, 0x17275e2aL, 0x7c484508L, 0x6ee66470L, 0x05897f52L, 0x3969e27fL, 0x5206f95dL,
	0x1c35a08fL, 0x775abbadL, 0x4bba2680L, 0x20d53da2L, 0x327b1cdaL, 0x591407f8L, 0x65f49ad5L, 0x0e9b81f7L,
	0x78c3993aL, 0x13ac8218L, 0x2f4c1f35L, 0x44230417L, 0x568d256fL, 0x3de23e4dL, 0x0102a360L, 0x6a6db842L,
	0x245ee190L, 0x4f31fab2L, 0x73d1679fL, 0x18be7cbdL, 0x0a105dc5L, 0x617f46e7L, 0x5d9fdbcaL, 0x36f0c0e8L,
	0x307e5a1bL, 0x5b114139L, 0x67f1dc14L, 0x0c9ec736L, 0x1e30e64eL, 0x755ffd6cL, 0x49bf6041L, 0x22d07b63L,
	0x6ce322b1L, 0x078c3993L, 0x3b6ca4beL, 0x5003bf9cL, 0x42ad9ee4L, 0x29c285c6L, 0x152218ebL, 0x7e4d03c9L,
	0x08151b04L, 0x637a0026L, 0x5f9a9d0bL, 0x34f58629L, 0x265ba751L, 0x4d34bc73L, 0x71d4215eL, 0x1abb3a7cL,
	0x548863aeL, 0x3fe7788cL, 0x0307e5a1L, 0x6868fe83L, 0x7ac6dffbL, 0x11a9c4d9L, 0x2d4959f4L, 0x462642d6L,
	0x20546c12L, 0x4b3b7730L, 0x77dbea1dL, 0x1cb4f13fL, 0x0e1ad047L, 0x6575cb65L, 0x59955648L, 0x32fa4d6aL,
	0x7cc914b8L, 0x17a60f9aL, 0x2b4692b7L, 0x40298995L, 0x5287a8edL, 0x39e8b3cfL, 0x05082ee2L, 0x6e6735c0L,
	0x183f2d0dL, 0x7350362fL, 0x4fb0ab02L, 0x24dfb020L, 0x36719158L, 0x5d1e8a7aL, 0x61fe1757L, 0x0a910c75L,
	0x44a255a7L, 0x2fcd4e85L, 0x132dd3a8L, 0x7842c88aL, 0x6aece9f2L, 0x0183f2d0L, 0x3d636ffdL, 0x560c74dfL,
	0x5082ee2cL, 0x3bedf50eL, 0x070d6823L, 0x6c627301L, 0x7ecc5279L, 0x15a3495bL, 0x2943d476L, 0x422ccf54L,
	0x0c1f9686L, 0x67708da4L, 0x5b901089L, 0x30ff0babL, 0x22512ad3L, 0x493e31f1L, 0x75deacdcL, 0x1eb1b7feL,
	0x68e9af33L, 0x0386b411L, 0x3f66293cL, 0x5409321eL, 0x46a71366L, 0x2dc80844L, 0x11289569L, 0x7a478e4bL,
	0x3474d799L, 0x5f1bccbbL, 0x63fb5196L, 0x08944ab4L, 0x1a3a6bccL, 0x715570eeL, 0x4db5edc3L, 0x26daf6e1L
	};

	uint32_t crccheck_table[256];
	uint32_t crctemp_table[256];

	static const size_t _4907MessageSize = 54;
	uint8_t _4907Message[_4907MessageSize] = {
			0xff, 0xff, 0xf5, 0xff, 0x00, 0x32, //framing and length
			0x21, 0x00, 0x00, 0x00, 0xee, //type and address lengths
			0x73, 0x4a, 0x1a, 0x2a, 0xaa, 0xaa, 0xa1, //destination address
			0x7a, 0x51, 0x1a, 0x2a, 0xaa, 0xa1, 0xa1, //source address
			0x00, //fixed
			0x00, //message number shifted left 1
			0x02, 0x03, //vital
			0x13, 0x2b, //label (4907)
			0x01, //version
			//HRI message
			0x12, //HRI message length
			0x00, 0x00, 0x00, 0x00, 0x00, //timestamp
			0x00, 0x00, 0x00, 0x00, //sequence number
			0x13, 0x2b, //label (4907)
			0x03, //version and vital
			//4907 message
			0x00, //state
			0x00, 0x00, //reserved
			//crc
			0x00, 0x00, //crc16
			0x00, 0x00, 0x00, 0x00 //vital crc
	};

	FrequencyThrottle<int> _4907Throttle;
	uint8_t _4907MessageNumber = 2;
	uint8_t _4907MessageSequenceNumber[4] = {0, 0, 0, 0};
};

/**
 * Construct a new HRIStatusPlugin with the given name.
 *
 * @param name The name to give the plugin for identification purposes
 */
HRIStatusPlugin::HRIStatusPlugin(string name) : PluginClient(name)
{
	AddMessageFilter<BsmMessage>(this, &HRIStatusPlugin::HandleBSMMessage);
	SubscribeToMessages();

	_trainComing = true;
	_serialDataTimeoutMS = 1500;

	_throttle.set_Frequency(std::chrono::milliseconds(2000));
	_4907Throttle.set_Frequency(std::chrono::milliseconds(1000));
}

HRIStatusPlugin::~HRIStatusPlugin()
{
	if (_portName == "")
	{
		if (!wdtPresent())
			AIOUSB_Exit();
	}
}

void HRIStatusPlugin::OnConfigChanged(const char *key, const char *value)
{
	PLOG(logINFO) << "OnConfigChanged";
	PluginClient::OnConfigChanged(key, value);
	UpdateConfigSettings();
}

void HRIStatusPlugin::OnStateChange(IvpPluginState state)
{
	PLOG(logINFO) << "OnStateChanged";
	PluginClient::OnStateChange(state);

	if (IvpPluginState::IvpPluginState_registered == state)
	{
		_isReceivingBsms = false;
		SetStatus("Receiving Bsms", _isReceivingBsms);
		_muteDsrcRadio = true;
		SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
		SetStatus("MuteDsrcRadio", _muteDsrcRadio);

		_trainComing = false;

		this->SetStatus<std::string>("Train", "Crossing is clear");
		_previousState = _trainComing;

		UpdateConfigSettings();
	}
}

void HRIStatusPlugin::UpdateConfigSettings()
{
	PLOG(logINFO) << "UpdateConfigSettings";

	std::string lanes;
	std::string intxnName;
	int intxnId;

	std::vector<std::string> tokens;

	GetConfigValue<uint64_t>("Frequency", _frequency, &_dataLock);
	GetConfigValue<uint64_t>("Monitor Frequency", _monitorFreq, &_dataLock);
	GetConfigValue<unsigned int>("RailPinNumber", _railPinNumber, &_dataLock);
	GetConfigValue("Serial Data Timeout", _serialDataTimeoutMS);
	GetConfigValue("Intersection Name", intxnName);
	GetConfigValue("Intersection ID", intxnId);

	GetConfigValue<bool>("Always Send", _alwaysSend);

	{
		lock_guard<mutex> lock(_stringConfigLock);
		GetConfigValue<std::string>("Lane Map", lanes);
		GetConfigValue<string>("Port Name", _portName);
	}

	boost::split(tokens, lanes, boost::is_any_of(",:"));

	std::lock_guard<mutex> lock(_dataLock);
	_laneMapping.clear();
	for(size_t i = 0; i < tokens.size(); i+=2)
	{
		_laneMapping.push_back(std::pair<int, std::string>(std::stoi(tokens[i]), tokens[i+1]));
	}

	// Build the static SPaT information
	message_tree_type &spatTree = _spat.get_storage().get_tree();
	spatTree.put("SPAT.intersections.IntersectionState.name", intxnName);
	message_tree_type &isTree = spatTree.get_child_optional("SPAT.intersections.IntersectionState").get();
	isTree.put("id.id", intxnId);
	isTree.put("revision", 1);
	bitset<16> status(0);
	isTree.put("status", status.to_string());
	isTree.put("moy", 0);
	isTree.put("timeStamp", 0);

	_newConfigValues = true;
}

void HRIStatusPlugin::HandleBSMMessage(BsmMessage &msg, routeable_message &routeableMsg)
{
	if(!_isReceivingBsms)
	{
		_isReceivingBsms = true;
		SetStatus("Receiving Bsms", _isReceivingBsms);
	}
	_throttle.Touch(0);
}

int HRIStatusPlugin::SetInterfaceAttribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	//PLOG(logDEBUG) << "SetInterfaceAttribs tcgetattr errror: " << errno;
            return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars

        tty.c_iflag = 0;				//no processing
        //tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        //tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

//        tty.c_cflag &= ~PARENB;   /* Disables the Parity Enable bit(PARENB),So No Parity   */
//        tty.c_cflag &= ~CSTOPB;   /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
//        tty.c_cflag &= ~CSIZE;	 /* Clears the mask for setting the data size             */
//        tty.c_cflag |=  CS8;      /* Set the data bits = 8                                 */
//
//        tty.c_cflag &= ~CRTSCTS;       /* No Hardware flow Control                         */
//        tty.c_cflag |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines       */
//
//
//        tty.c_iflag &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
//        tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                            */
//
//        tty.c_oflag &= ~OPOST;/*No Output Processing*/


        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
        	//PLOG(logDEBUG) << "SetInterfaceAttribs tcsetattr errror: " << errno;
                return -1;
        }
        return 0;
}

void HRIStatusPlugin::SetBlocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	//PLOG(logDEBUG) << "SetBlocking tcgetattr errror: " << errno;
            return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        //if (tcsetattr (fd, TCSANOW, &tty) != 0)
        	//PLOG(logDEBUG) << "SetBlocking tcsetattr errror: " << errno;
}


/**
 * Function to setup the ESP to read signals
 *
 * @return true if a device is found successfully false otherwise
 */
bool HRIStatusPlugin::DioSetup()
{
	unsigned long resultCode;
	resultCode = 30; // unknown error code during api initialization

	string type;
	if (wdtPresent()) {
		type = "POC-351VTC Device";
		resultCode = InitDIO() ? 0 : 1;
	}
	else {
		type = "ESP Box";
		resultCode = AIOUSB_Init();
	}

	if(resultCode == 0)
	{
		PLOG(logINFO) << "Connected to digital I/O on " << type;
		return true;
	}
	else
	{
		PLOG(logINFO) << "Failure to connect to digital I/O on " << type << ": " << resultCode;
		return false;
	}
}

/**
 * Function to get the state of a pin on ESP box
 *
 * @param pin pin number to query
 *
 * @return true if the pin is high, false otherwise.
 */
bool HRIStatusPlugin::GetPinState(int pinNumber)
{
	int i;
	if (_portName == "")
	{
		if (wdtPresent()) {
			return DIReadLine(pinNumber);
		} else {
			DIOBuf* readBuffer = NewDIOBuf(16);
			char * pinStatusString;
			unsigned long result = DIO_ReadIntoDIOBuf(diFirst, readBuffer);
			if(result == 0)
			{
				//PLOG(logINFO) << "Pins were read successfully. Preparing result...";
				pinStatusString = DIOBufToString(readBuffer);
				PLOG(logDEBUG)  << "The status of pin " << pinNumber << " is: " << (int)pinStatusString[GetBufferPosition(pinNumber)];
				if(pinStatusString[GetBufferPosition(pinNumber)] == '1')
					return true;
				else
					return false;
			}
			else
			{
				PLOG(logINFO) << "Error reading the pin";
			}
		}
	}
	else
	{
		if (_serialPortFd >= 0)
		{
			return _serialPinState;
		}
	}
	return false;
}

int HRIStatusPlugin::GetBufferPosition(int pin)
{
	int position = 0;
	if (pin < 8)
	{
		position = 23 - pin;
	}
	else
	{
		position = 31 - pin + 8;
	}
	return position;
}

/**
 * Function to monitor the rail signal on a separate thread. If the pin
 * is voltage low the train is coming.
 */
void HRIStatusPlugin::MonitorRailSignal()
{
	while(!_stopThreads)
	{
		if(!GetPinState(_railPinNumber))
		{
			// sets a global variable. Should it send an Application Message?
			_trainComing = true; //Atomic wrapper does not need mutex locked.

			if(_trainComing != _previousState)
			{
				PLOG(logINFO) << "Train is present at the crossing.";
				this->SetStatus<std::string>("Train", "Train present at crossing.");
				_previousState = _trainComing;
			}

		}
		else
		{
			_trainComing = false;
			if(_trainComing != _previousState)
			{
				PLOG(logINFO) << "Crossing is clear.";
				this->SetStatus<std::string>("Train", "Crossing is clear");
				_previousState = _trainComing;
			}
		}


		usleep(_monitorFreq * 1000); // check 10 times per second
	}
}

/**
 * Function to read serial port and set train present state
 */
void HRIStatusPlugin::SerialPortReader()
{
	int i;
	stringstream ss;
	string ss_string;

	memset(crccheck_table, 0, 256);
	memset(crctemp_table, 0, 256);

	if (_portName == "")
		return;

	while(!_stopThreads)
	{
		if (_serialPortFd >= 0)
		{
			//read serial port
			//PLOG(logDEBUG) << "Read open port";
			unsigned char buf [1024];
			uint64_t currentTime = Clock::GetMillisecondsSinceEpoch();
			if (currentTime - _lastSerialDataTime > _serialDataTimeoutMS)
			{
				_sendSPAT = false;
				_serialPinState = false;
			}
			int n = read (_serialPortFd, buf, sizeof buf);  // read up to 1024 characters if ready to read
			if (n == -1)
				_serialPortFd = -1;
			while (n > 0)
			{
				//PLOG(logDEBUG) << "Read " << n << " characters on serial port";

				if (_serialDataLength + n > sizeof _serialBuffer)
				{
					if (((_serialDataLength + n) / 2048) > 1000)
					{
						_serialDataLength = 0;
					}
					else
					{
						_serialBuffer = (unsigned char*)realloc(_serialBuffer, (((_serialDataLength + n) / 2048) + 1) * 2048);
					}
				}
				memcpy(&(_serialBuffer[_serialDataLength]), buf, n);
				_serialDataLength += n;


				//process data
				int length = _serialDataLength;
				int messageLength;
				for (i=0;i<length;i++)
				{
					//possible start of frame
					if (_serialBuffer[i] == 255)
					{
						//check if we have at least 6 bytes
						if (_serialDataLength - i >= 6)
						{
							//check for header
							if (_serialBuffer[i+1] == 255 && _serialBuffer[i+2] == 245 && _serialBuffer[i+3] == 255)
							{
								//get length
								messageLength = (_serialBuffer[i+4] * 256) + _serialBuffer[i+5];
								if ((_serialDataLength - i) >= (messageLength + 4))
								{
									//process message
									//get address lengths
									//PLOG(logDEBUG) << "Message length:" << messageLength;
									uint8_t sourceAddressLen = _serialBuffer[i+10] >> 4;
									uint8_t destAddressLen = _serialBuffer[i+10] & 0x0f;
									uint8_t totalAddressLen = (sourceAddressLen / 2) + (destAddressLen / 2);
									if (sourceAddressLen % 2 == 1)
										totalAddressLen++;
									if (destAddressLen % 2 == 1)
										totalAddressLen++;
									int label = (_serialBuffer[i+totalAddressLen+15] * 256) + _serialBuffer[i+totalAddressLen+16];
									if (label == 4904)
									{
										//get crc of 42 byte message
										uint32_t calculatedCrc = 0;
//										memset(crctemp_table, 0, 256);
										calculatedCrc = GetCrc32(calculatedCrc, &(_serialBuffer[i+10]), messageLength - 10);
										uint32_t messageCrc = *((uint32_t*)(&(_serialBuffer[i+messageLength])));
//										ss.str("");
//										for (int j = 0; j < messageLength + 4; j++)
//											ss << hex << setfill('0') << setw(2) << (unsigned int)_serialBuffer[i + j] << " ";
//										ss_string = ss.str();
										PLOG(logDEBUG) << "Got 4904 message, vital crc:" << messageCrc << ", calculated crc:" << calculatedCrc;
//										PLOG(logDEBUG) << ss_string;
										if (calculatedCrc == messageCrc)
										{
//											for (int j = 0; j < 256; j++)
//											{
//												if (crctemp_table[j] == 1)
//													crccheck_table[j] = 1;
//											}
//											ss.str("");
//											for (int j = 0; j < 256; j++)
//											{
//												if (crccheck_table[j] == 0)
//													ss << j << " ";
//											}
//											ss_string = ss.str();
//											PLOG(logDEBUG) << ss_string;
											//get WSA bit for crossing 1

//											using std::chrono::system_clock;
//											std::time_t tt;
//											system_clock::time_point current_time = system_clock::now();
//											tt = system_clock::to_time_t(current_time);
//											auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time.time_since_epoch()) -
//											          std::chrono::duration_cast<std::chrono::seconds>(current_time.time_since_epoch());

											unsigned char tpd = _serialBuffer[i+totalAddressLen+33] & 0x04;
											if (tpd > 0)
											{
												PLOG(logDEBUG) << "Got 4904 message, HRI Active";
												//PLOG(logDEBUG) << "  milliseconds: " << ms.count();
												_serialPinState = false;
												_lastSerialDataTime = Clock::GetMillisecondsSinceEpoch();
											}
											else
											{
												PLOG(logDEBUG) << "Got 4904 message, HRI NOT Active";
												//PLOG(logDEBUG) << "  milliseconds: " << ms.count();
												_serialPinState = true;
												_lastSerialDataTime = Clock::GetMillisecondsSinceEpoch();
											}
											_sendSPAT = true;
										}
									}
									//increment index past message
									i += (messageLength + 4);
									if (i >=_serialDataLength)
									{
										_serialDataLength = 0;
									}
								}
								else
								{
									//have header but not enough bytes, copy and exit loop
									memcpy(_serialBuffer, &(_serialBuffer[i]), _serialDataLength - i);
									_serialDataLength = _serialDataLength - i;
									i = length;
								}
							}
						}
						else
						{
							//not enough bytes left, copy and exit loop
							memcpy(_serialBuffer, &(_serialBuffer[i]), _serialDataLength - i);
							_serialDataLength = _serialDataLength - i;
							i = length;
						}
					}
					else
					{
						//check if last byte and no header start
						if (i ==_serialDataLength -1)
						{
							_serialDataLength = 0;
						}
					}
				}
				n = read (_serialPortFd, buf, sizeof buf);
				if (n == -1)
					_serialPortFd = -1;
			}
		}
		usleep(100000); // check 10 times per second
	}
}

uint16_t HRIStatusPlugin::GetCrc16(uint16_t crc, uint8_t *data, uint16_t length)
{
	if (length)
	{
		while (length--)
		{
			crc = (crc >> 8) ^ crc16_table[(crc ^ *data++) & 0xff];
		}
	}
	return crc;
}

uint32_t HRIStatusPlugin::GetCrc32(uint32_t crc, uint8_t *data, uint16_t length)
{
//	uint32_t index;
	if (length)
	{
		while (length--)
		{
//			index = (crc ^ *data) & 0xffL;
//			crctemp_table[index] = 1;
			crc = (crc >> 8) ^ crc32_table[(crc ^ *data++) & 0xffL];
		}
	}
	return crc;
}

void HRIStatusPlugin::UpdateTimestamp(message_document &md)
{
	struct timeval tv;
	Clock::GetTimevalSinceEpoch(Clock::GetMillisecondsSinceEpoch(), tv);
	struct tm * utctime = gmtime( (const time_t *)&tv.tv_sec );

	// In SPAT, the time stamp is split into minute of the year and millisecond of the minute
	// Calculate the minute of the year
	unsigned long long int minOfYear = utctime->tm_min + (utctime->tm_hour * 60) + (utctime->tm_yday * 24 * 60);

	// Calculate the millisecond of the minute
	unsigned long long int msOfMin = (1000 * utctime->tm_sec) + (tv.tv_usec / 1000);

	// Update the document
    pugi::xpath_node minOfYearNode = md.select_node("/SPAT/intersections/IntersectionState/moy");
    minOfYearNode.node().text().set(minOfYear);
    pugi::xpath_node msOfMinNode = md.select_node("/SPAT/intersections/IntersectionState/timeStamp");
    msOfMinNode.node().text().set(msOfMin);
}


void HRIStatusPlugin::UpdateMovementState(message_document &md)
{
	pugi::xpath_node intersectionState = md.select_node("//IntersectionState");

	intersectionState.node().remove_child("states");

	pugi :: xml_node states = intersectionState.node().append_child("states");

	std::lock_guard<mutex> lock(_dataLock);
	for(std::pair<int, std::string> newState : _laneMapping)
	{
		pugi::xml_node movementState = states.append_child("MovementState");
		movementState.append_child("signalGroup").append_child(pugi::node_pcdata).set_value(std::to_string(newState.first).c_str());
		pugi::xml_node movementEvent = movementState.append_child("state-time-speed").append_child("MovementEvent");
		pugi::xml_node eventState = movementEvent.append_child("eventState");

		if(_trainComing)
		{
			if(newState.second == "tracked")
			{
				eventState.append_child("protected-Movement-Allowed");
			}
			else
			{
				eventState.append_child("stop-And-Remain");
			}
		}
		else
		{
			if(newState.second == "tracked")
			{
				eventState.append_child("stop-And-Remain");
			}
			else
			{
				eventState.append_child("permissive-Movement-Allowed");
			}
		}

		pugi::xml_node timing = movementEvent.append_child("timing");
		timing.append_child("minEndTime").append_child(pugi::node_pcdata).set_value("32850");
		timing.append_child("maxEndTime").append_child(pugi::node_pcdata).set_value("32850");
	}

}

int HRIStatusPlugin::Main()
{
//	PLOG(logINFO) << "Starting Plugin";
//
//	//wait for initial config values
//	while (!_newConfigValues)
//		this_thread::sleep_for(std::chrono::milliseconds(100));
//
//	PLOG(logINFO) << "Updated Config";
//
//	while (!IsPluginState(PluginState::error))
//	{
//		if (_newConfigValues)
//		{
//
//		}
//
//		//sleep
//		this_thread::sleep_for(std::chrono::milliseconds(10));
//	}
//
//	return (EXIT_SUCCESS);

	uint64_t lastSendTime = 0;
	//= GetMsTimeSinceEpoch();

	PLOG(logINFO) << "Starting Plugin";

	//wait for initial config values
	while (!_newConfigValues)
		this_thread::sleep_for(std::chrono::milliseconds(100));

	usleep(2000000); //Allow 2 seconds for initialization

	PLOG(logINFO) << "Port Name: " << _portName;

	if (_portName == "")
	{
		DioSetup();
	}
	else
	{
		_serialPortFd = open (_portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
		if (_serialPortFd < 0)
		{
			PLOG(logERROR) << "Error opening serial port";
		}
		else
		{
			SetInterfaceAttribs (_serialPortFd, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
			//SetInterfaceAttribs (_serialPortFd, B9600, 0);  // set speed to 9600 bps, 8n1 (no parity)
			SetBlocking (_serialPortFd, 0);                // set no blocking
		}

	}

	std::thread trainWatch(&HRIStatusPlugin::MonitorRailSignal, this);
	std::thread serialPortReader(&HRIStatusPlugin::SerialPortReader, this);

	usleep(2000000); //wait for thread to spin up

	while (!IsPluginState(IvpPluginState_error))
	{
		//retry opening serial port
		if (_portName != "" && _serialPortFd == -1)
		{
			_serialPortFd = open (_portName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
			if (_serialPortFd < 0)
			{
				PLOG(logERROR) << "Error opening serial port";
			}
			else
			{
				SetInterfaceAttribs (_serialPortFd, B115200, 0);  // set speed to 115200 bps, 8n1 (no parity)
				SetBlocking (_serialPortFd, 0);                // set no blocking
			}

		}


		message_container_type copy;
		{
			lock_guard<mutex> lock(_dataLock);
			copy = _spat;
		}

		if (!copy.get_storage().get_tree().empty()) {
			SpatMessage spat(copy);
			message_document md(spat);
			UpdateTimestamp(md);
			UpdateMovementState(md);
			md.flush();
			spat.flush();

			//md.print(cout);

			SpatEncodedMessage spatEnc;
			//spat.flush();
			//PLOG(logDEBUG) << spat;
			spatEnc.initialize(spat);
			//PLOG(logDEBUG) << spatEnc;
			spatEnc.set_flags(IvpMsgFlags_RouteDSRC);
			spatEnc.addDsrcMetadata(172, 0x8002);

			//cout << spatEnc;

			// Broadcast the message
			if (_throttle.Monitor(0))
			{
				//PLOG(logDEBUG) << "BSMs Not Found";
				if(_isReceivingBsms)
				{
					_isReceivingBsms = false;
					SetStatus("Receiving Bsms", _isReceivingBsms);
				}
			}

			if(_alwaysSend)
			{
				if(_muteDsrcRadio)
				{
					_muteDsrcRadio = false;
					SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
					SetStatus("MuteDsrcRadio", _muteDsrcRadio);
				}
			}
			else
			{
				if(_trainComing || _isReceivingBsms)
				{
					if(_muteDsrcRadio)
					{
						_muteDsrcRadio = false;
						SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
						SetStatus("MuteDsrcRadio", _muteDsrcRadio);
					}
				}
				else
				{
					if(!_muteDsrcRadio)
					{
						_muteDsrcRadio = true;
						SetSystemConfigValue("MuteDsrcRadio", _muteDsrcRadio, false);
						SetStatus("MuteDsrcRadio", _muteDsrcRadio);
					}
				}
			}

			//always send spat if using analog input method
			//if using serial data only send SPAT if we got a valid serial message
			if (_sendSPAT == true)
				BroadcastMessage(static_cast<routeable_message &>(spatEnc));
		}

		//send HRI state message 4907
		if (_portName != "" && _serialPortFd >= 0 && _4907Throttle.Monitor(0))
		{
			//set message number
			_4907Message[26] = _4907MessageNumber;
			//set timestamp

			//set sequence number
			memcpy(&(_4907Message[38]), _4907MessageSequenceNumber, 4);
			//set state, RHBW=1, RSO=1, VAS=2, LTI=0, VP=0
			_4907Message[45] = 0xe0;
			//get crc16
			uint8_t crcByte = 0;
			uint16_t calculatedCrc16 = 0xffff;
			calculatedCrc16 = ~GetCrc16(calculatedCrc16, &(_4907Message[32]), 16);
			//set crc16
			crcByte = (uint8_t)(calculatedCrc16 & 0x00ff);
			memcpy(&(_4907Message[48]), &crcByte, 1);
			crcByte = (uint8_t)((calculatedCrc16 >> 8) & 0x00ff);
			memcpy(&(_4907Message[49]), &crcByte, 1);
			//memcpy(&(_4907Message[48]), &calculatedCrc16, 2);
			//get vital crc
			uint32_t calculatedCrc = 0;
			calculatedCrc = GetCrc32(calculatedCrc, &(_4907Message[10]), 40);
			//set vital crc
			crcByte = (uint8_t)(calculatedCrc & 0x000000ff);
			memcpy(&(_4907Message[50]), &crcByte, 1);
			crcByte = (uint8_t)((calculatedCrc >> 8) & 0x000000ff);
			memcpy(&(_4907Message[51]), &crcByte, 1);
			crcByte = (uint8_t)((calculatedCrc >> 16) & 0x000000ff);
			memcpy(&(_4907Message[52]), &crcByte, 1);
			crcByte = (uint8_t)((calculatedCrc >> 24) & 0x000000ff);
			memcpy(&(_4907Message[53]), &crcByte, 1);
			//memcpy(&(_4907Message[50]), &calculatedCrc, 4);
			//for (int iii = 0;iii<_4907MessageSize;iii++)
			//{
			//	PLOG(logDEBUG) << "BYTE: " << std::hex << (int)_4907Message[iii];
			//}
			//send message
			size_t rc = write(_serialPortFd, _4907Message, _4907MessageSize);
			//if (rc == -1)
			//	PLOG(logDEBUG) << "Error sending 4907 message: " << errno;
			//increment message number
			_4907MessageNumber += 2;
			//increment message sequence number
			_4907MessageSequenceNumber[3]++;
			if (_4907MessageSequenceNumber[3] == 0)
			{
				_4907MessageSequenceNumber[2]++;
				if (_4907MessageSequenceNumber[2] == 0)
				{
					_4907MessageSequenceNumber[1]++;
					if (_4907MessageSequenceNumber[1] == 0)
					{
						_4907MessageSequenceNumber[0]++;
					}
				}
			}

		}

		usleep(_frequency * 1000);
	}

	_stopThreads = true;
	trainWatch.join();
	serialPortReader.join();

	return 0;


}


} /* namespace HRIStatusPlugin */

int main(int argc, char *argv[])
{
	return run_plugin<HRIStatusPlugin::HRIStatusPlugin>("HRIStatusPlugin", argc, argv);
}
