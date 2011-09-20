#ifndef MiniBee_APIn_h
#define MiniBee_APIn_h

#include <XBee.h>

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

    char *data;
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

};

#endif	
