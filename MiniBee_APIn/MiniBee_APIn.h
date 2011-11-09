#ifndef MiniBee_APIn_h
#define MiniBee_APIn_h

// #include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <WProgram.h>
// #include <EEPROM.h>
// #include <Wire.h>

#include <XBee.h>


#define MINIBEE_REVISION 'D'
#define MINIBEE_LIBVERSION 3

/// all together: 3644 bytes
#define MINIBEE_ENABLE_TWI 1  /// TWI/I2C takes up 2064 bytes
#define MINIBEE_ENABLE_SHT 1  /// SHT takes up 1140 bytes
#define MINIBEE_ENABLE_PING 1 /// Ping takes up 440 bytes

#define MINIBEE_ENABLE_TWI_ADXL 1 /// 962 bytes - without: 18176
#define MINIBEE_ENABLE_TWI_LISDL 1 /// 614 bytes - without: 18524
#define MINIBEE_ENABLE_TWI_HMC 1 /// 1644 bytes - without: 17494 
#define MINIBEE_ENABLE_TWI_BMP 1 /// 4182 bytes - without: 14956
#define MINIBEE_ENABLE_TWI_TMP 1 /// 250 bytes - without: 18888



#if MINIBEE_REVISION == 'D'
#define NRPINS 17
#endif
#if MINIBEE_REVISION == 'B'
#define NRPINS 17
#endif
#if MINIBEE_REVISION == 'A'
#define NRPINS 19
#endif

#if MINIBEE_ENABLE_TWI == 1
#include <Wire.h>

#if MINIBEE_ENABLE_TWI_ADXL == 1
#include <ADXL345.h>
#endif

#if MINIBEE_ENABLE_TWI_LISDL == 1
#include <LIS302DL.h>
#endif

#if MINIBEE_ENABLE_TWI_TMP == 1
#include <TMP102.h>
#endif

#if MINIBEE_ENABLE_TWI_BMP == 1
#include <BMP085.h>
#endif

#if MINIBEE_ENABLE_TWI_HMC == 1
#include <HMC5843.h>
#endif

#endif

class MiniBee_API{
  public:
    MiniBee_API();
    void setup( long );
    void loopStep();
    
    void setID( uint8_t id );
    uint8_t getId(void);
    
    void sendData( void );
    void sendActive( void );
    void sendPaused( void );
    
    void readXBeeSerial();
    void sendXBeeSerial();
    
    uint8_t* sendAtCommand();
    void sendTx16( char type, uint8_t* data, uint8_t length );
    
//     void setCustomParser( void (*customFunc)(uint8_t *, int ) );

    void setRunning( uint8_t ); 
    void setLoopback( uint8_t ); 


  private:
    uint8_t readSensors( uint8_t db );
    void readXBeePacket();
    void routeMsg(uint8_t type, uint8_t *msg, uint8_t size);
    bool checkIDMsg( uint8_t mid );
    
    void flashLed(int pin, int times, int wait);
    
    char msg_type;
    uint8_t status;
    uint8_t actcount;

//     char *outMessage;
    uint8_t outMessage[2];

    uint8_t *data;
    int datacount;
    int datasize;
    
    uint8_t node_id;
    uint8_t config_id;
    uint8_t configInfo[2];
    
    uint8_t prev_id_msg;
    
    uint8_t msg_id_send;
    int msgInterval;
    uint8_t samplesPerMsg;
    int smpInterval;
    uint8_t curSample;
    
    bool loopback;

};

#endif	
