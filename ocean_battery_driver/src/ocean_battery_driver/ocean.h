#pragma once
/**
 */
#include <string>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>

namespace willowgarage
{
  namespace ocean
  {
    class ocean
    {
    public:
      static const int NMEA_MAX = 120;
      static const int MAXTAGLEN = 8;
      static const int MAXCHANNELS = 16;

      static const int INPUT_BUF_SIZE = 128;
      static const int OUTPUT_BUF_SIZE = 128;
      static const unsigned int MAX_PACKET_LENGTH = 120;
      static const int BAD_PACKET = -1;
      static const int NO_PACKET = 0;
      static const int NMEA_PACKET = 1;

      //Battery Registers
      //Takend from the Smart Battery Data Specification Revision 1.1, Dec. 11, 1998
      //
      static const int manufacturerAccess     = 0x00;
      static const int remainingCapacityAlarm = 0x01;
      static const int remainingTimeAlarm     = 0x02;
      static const int batteryMode            = 0x03;
      static const int atRate                 = 0x04;
      static const int atRateTimeToFull       = 0x05;
      static const int atRateTimeToEmpty      = 0x06;
      static const int atRateOK               = 0x07;
      static const int temperature            = 0x08; 
      static const int voltage                = 0x09; 
      static const int current                = 0x0a; 
      static const int averageCurrent         = 0x0b; 
      static const int maxError               = 0x0c; 
      static const int relativeStateOfCharge  = 0x0d;
      static const int absoluteStateOfCharge  = 0x0e; 
      static const int remainingCapacity      = 0x0f; 
      static const int fullChargeCapacity     = 0x10; 
      static const int runTimeToEmpty         = 0x11; 
      static const int averageTimeToEmpty     = 0x12; 
      static const int averageTimeToFull      = 0x13; 
      //static const int                        = 0x14; 
      //static const int                        = 0x15; 
      static const int batteryStatus          = 0x16; 
      static const int cycleCount             = 0x17; 
      static const int designCapacity         = 0x18; 
      static const int designVoltage          = 0x19; 
      static const int specificationInfo      = 0x1a; 
      static const int manufactureDate        = 0x1b; 
      static const int serialNumber           = 0x1c; 
      static const int manufactureName        = 0x20; 
      static const int deviceName             = 0x21; 
      static const int deviceChemistry        = 0x22; 
      static const int manufactureData        = 0x23; 

      ocean (int debug = 0);
      ~ocean ();

      int run ();
      void setDebugLevel (int);
      void initialize (const std::string &input_dev);

    private:
      void flush (void);        //Flushes the serial buffer
      int get_speed (void);
      void set_speed (int speed);
      void report (int errlevel, const char *fmt, ...);
      char *gpsd_hexdump (void *binbuf, size_t binbuflen);
      void nextstate (unsigned char c);
      void packet_accept (int packet_type);
      void packet_discard ();
      void character_discard ();
      ssize_t packet_parse (size_t newdata);
      ssize_t packet_get ();
      void packet_reset ();
      int nmea_send (const char *fmt, ...);
      void nmea_add_checksum (char *sentence);
      time_t mkgmtime (register struct tm *t);
      unsigned int nmea_parse ();
      unsigned int processController (int count, char *field[]);
      unsigned int processSystem (int count, char *field[]);
      unsigned int processBattery (int count, char *field[]);

    private:
      int inputDevice;
#if (FILE_LOGGING > 0)
      int outputFile;
#endif
      struct termios ttyset;
      int currentBaudRate;
      int currentBaudRateIndex;
      int currentPort;          //Port on GPS A=0, B=1, C=2
      char *idString;
      int debuglevel;
      int sentenceCount;
      int currentBattery;

      //Parsing stuff
      int packetState,          // Current statemachine location
        packetType;             // Type of packet just identified
      unsigned char inbuffer[INPUT_BUF_SIZE + 1];
      unsigned char outbuffer[OUTPUT_BUF_SIZE + 1];
      size_t inbuflen, outbuflen;
      unsigned char *inbufptr;
      unsigned long char_counter;
      char tag[MAXTAGLEN];
      int sentenceLength;
      int cycleComplete;
      int acknowledge;          // set this true when we receive a valid acnowledge

    public:                    //Expose this for easy access to results
      static const unsigned int MAX_BAT_COUNT = 4;
      static const unsigned int MAX_BAT_REG = 0x30;
      time_t lastTimeSystem, lastTimeController, lastTimeBattery[MAX_BAT_COUNT];
      unsigned short timeLeft;
      unsigned short averageCharge;
      unsigned short present;
      unsigned short charging;
      unsigned short discharging;
      unsigned short reserved;
      unsigned short powerPresent;
      unsigned short powerNG;
      unsigned short inhibited;
      unsigned short batReg[MAX_BAT_COUNT][MAX_BAT_REG];
      unsigned short batRegFlag[MAX_BAT_COUNT][MAX_BAT_REG];
      time_t batRegTime[MAX_BAT_COUNT][MAX_BAT_REG];
      char message[64];
    };
  }
}
