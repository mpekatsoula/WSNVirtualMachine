#ifndef CONSTANTS_H
#define CONSTANTS_H

#define GROUP_ID 2

#define MAX_REGS 10
#define MAX_APPS 3
#define MAX_HOPS 10

#define HEARTBEAT 30

#define NULL_HANDLER 0
#define INIT_HANDLER 1
#define TIMER_HANDLER 2
#define MSG_HANDLER 4

#define CACHE_TIMEOUT 1000

#define SerialQueueSize 30
#define SerialMsgSize 20

#define INSTALL 11
#define UNINSTALL 12

#define CACHE_SIZE 10

enum { AM_NODETOPC = 6 };

/* Application struct */
typedef struct _app {

  uint8_t init[255];
  uint8_t timer[255];
  uint8_t msg[255];
  uint8_t id; // application id. Usefull for unloading apps
  uint8_t PC; // program counter
  uint8_t handler; // running handler
  uint8_t next_handler; // next handler to run
  uint8_t hops;
  int8_t waiting_sensor;
  int8_t regs[MAX_REGS];
  am_addr_t sink;
  bool isActive;
  bool hasMSGHandler;
  
} app;

typedef struct _radio_msg {
  
  nx_uint16_t group_id;
  nx_uint8_t appid;
  nx_int8_t results[2];
  
} radio_msg;

typedef struct _binary_msg {
  
  nx_uint16_t group_id;
  nx_uint8_t unique_id;
  nx_uint8_t type;
  nx_uint8_t appid;
  nx_uint8_t hops;
  nx_uint8_t binary[32];
    
} binary_msg;

typedef struct test_serial_msg_resp {
  
  nx_uint8_t instr;
  nx_uint8_t pc;
  nx_uint8_t appid;

    
} serial_response;
#endif