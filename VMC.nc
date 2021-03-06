#include "constants.h"

module VMC
{
  uses 
  {
    interface Timer<TMilli> as Timer[uint8_t id];
    interface Timer<TMilli> as HeartbeatTimer;
    interface Leds;
    interface Boot;
    interface Read<uint16_t>;
    interface Packet as SerialPacket;
    interface AMPacket as SerialAMPacket;
    interface AMSend as SerialAMSend;
    interface Receive as SerialReceive;
    interface SplitControl as SerialAMControl;
    
    interface AMPacket as RadioAMPacket;
    interface Receive as RadioReceive;
    interface AMSend as RadioAMSend;
    interface Packet as RadioPacket;
    interface SplitControl as AMControl;
  }
}
implementation
{

  /* Declarations */
  task void execute_instructionTask();
  void send_serial();
  void send_serial_msg( uint8_t instr, uint8_t appid, uint8_t pc );
  void forward_app( uint8_t *bin, uint8_t appid, uint8_t type, uint8_t uid, uint8_t hops );
  void sendMsg( am_addr_t sink, uint8_t id, int8_t r7,  int8_t r8,  int8_t sendboth );
  
  /* Serial variables */
  bool serial_busy = FALSE;
  bool busy = FALSE;
  message_t radio_p; 
  message_t serial_p;
  uint8_t serial_msgs = 0; 
  uint8_t serial_msgs_start = 0;
  uint8_t serial_msgs_queue[SerialQueueSize][SerialMsgSize];

  /* Store CACHE_SIZE messages */
  uint8_t cache[CACHE_SIZE];
  uint8_t local_unique_id; // Only for sink
  uint8_t cache_pos = 0;
  
  app apps[MAX_APPS];
  uint8_t active_app = MAX_APPS;
  uint32_t cache_time = 0;
  uint8_t cache_value = 100;

  /* Check if i have the message in my cache. If yes return true */
  bool search_cache( uint16_t uid ) {

    uint8_t i;
    
    for ( i = 0; i < CACHE_SIZE; i++ )
      if ( cache[i] == uid ) 
        return TRUE;
  
    return FALSE;
    
  }
    
  /* Store a message in cache */
  void store_in_cache( uint16_t uid ) {

    cache[cache_pos] = uid;
    cache_pos = ( cache_pos + 1 ) % CACHE_SIZE;  
  
  }
  
  /* Initialize VM */
  void system_init() {

    uint8_t i;
    for ( i = 0; i < MAX_APPS; i++ ) {

      apps[i].isActive = FALSE;
      apps[i].handler = NULL_HANDLER;
      apps[i].hasMSGHandler = FALSE;
      apps[i].id = 0;

    }
    
    /* Initialize cache and local unique_id */
    for ( i = 0; i < CACHE_SIZE; i++ )
      cache[i] = 0;
    local_unique_id = 1;
    
  }

  /* Choose next application to run with Round Robin */
  void next_app() {

    uint8_t i;

    for ( i = 0; i < MAX_APPS; i++ ) {
      active_app = (active_app + 1 ) % MAX_APPS;
      if ( apps[active_app].isActive && apps[active_app].handler != NULL_HANDLER ) {
        post execute_instructionTask();
        return;
      }
    }

    /* This means that no active application is present */
    active_app = MAX_APPS;

  }

  /* Read data from the sensor */
  void read_sensor ( int8_t reg ) {

    uint32_t time_now = call Timer.getNow[0]();
    
    if ( time_now - cache_time < CACHE_TIMEOUT && cache_time != 0  ) {

      apps[active_app].regs[reg] = cache_value;
      dbg("SENSOR", "Read from cache.\n");
      return;

    }
    
    /* Store the waiting reg id */
    apps[active_app].waiting_sensor = reg;
    call Read.read();
  }

  /* Unload a binary from the VM */
  void unload_binary ( uint8_t appid, uint8_t uid ) {
    
    /* Find the application */
    uint8_t i,id;
    id = MAX_APPS;
    
    /* Forward application */
    forward_app( NULL, appid, 12, uid, 0 );
    
    for ( i = 0; i < MAX_APPS; i++ ) {

      if ( apps[i].id == appid ) {
        id = i;
        break;
      }
    }

    /* No application found */
    if ( id == MAX_APPS ) {
      dbg("BIN","No application with id:%d is active.\n", appid);
      return;
    }

    apps[id].isActive = FALSE;
    apps[id].handler = NULL_HANDLER;
    apps[id].id = 0;
    apps[id].hasMSGHandler = FALSE;
    
    switch ( id ) {

      case 0:
      call Leds.led0Off();
      break;

      case 1:
      call Leds.led1Off();
      break;

      case 2:
      call Leds.led2Off();
      break;

    }

    /* Deactivate all timers except 0 */
    if ( id != 0 )
      call Timer.stop[id]();

    /* If i am the active application find next, if any */
    if ( id == active_app ) {
      for ( i = 0; i < MAX_APPS ; i++ ) {
        active_app = (active_app + 1 ) % MAX_APPS;
        if ( apps[active_app].isActive && apps[active_app].handler != NULL_HANDLER )
          break;
       }
     }
     
    dbg("BIN","Application with id:%d unloaded.\n", appid );     
     /* If no other application is active, stop */
     if ( active_app == id )
      active_app = MAX_APPS;

  }

  /* Load a binary into the VM */
  int8_t load_binary ( uint8_t *bin, uint8_t appid, uint8_t uid, uint8_t hops, void *msg ) {

    uint8_t i, id;
    id = MAX_APPS;

    /* Forward application */
    forward_app( bin, appid, 11, uid, hops );
    
    /* If i took heartbeat update fathers etc */
    if ( appid == HEARTBEAT ) {
    
      for ( i = 0; i < MAX_APPS; i++ ) {

        if ( apps[i].isActive && apps[i].sink != call RadioAMPacket.address() )
          apps[i].sink = call RadioAMPacket.source(msg); 
        
      }
      
      /* -1 in order not to update the father again */
      return -1;
    }
    
    /* Find a slot for our app. If the app already exists, overwrite it */
    for ( i = 0; i < MAX_APPS; i++ ) {

      if ( apps[i].id == appid ) {
        id = i;
        break;
      }

      /* Store first empty slot */
      if ( !apps[i].isActive && id == MAX_APPS )
        id = i;
    }

    /* No space avail */
    if ( id == MAX_APPS ) {
      dbg("BIN","No space available.\n");
      return -1;
    }

    /* Copy init handler */
    for ( i = 0; i < bin[1]; i++ )
      apps[id].init[i] = bin[4 + i];

    /* Copy timer handler */
    for ( i = 0; i < bin[2]; i++ )
      apps[id].timer[i] = bin[4 + bin[1] + i];

    /* Copy message handler */
    for ( i = 0; i < bin[3]; i++ ) {
      apps[id].msg[i] = bin[4 + bin[1] + bin[2] + i];
      apps[id].hasMSGHandler = TRUE;
    }
      
    /* Initialize app */
    apps[id].handler = INIT_HANDLER;
    apps[id].isActive = TRUE;
    apps[id].id = appid;
    apps[id].waiting_sensor = MAX_REGS;
    apps[id].hops = hops++;
    
    /* Start application timer */
    if ( bin[2] > 0 ) {
      //apps[id].next_handler = TIMER_HANDLER;
      call Timer.startPeriodic[id]( 1000 );
    }

    /* Initialize registers */
    for ( i = 0; i < MAX_REGS; i++ )
      apps[id].regs[i] = 0;

#ifdef DEBUG
    dbg("BIN", "Appid: %d id: %d \n", apps[id].id, id);
    for ( i = 0; i < 255; i++ ) {

      dbg("BIN", "Init: 0x%x \n", apps[id].init[i]);
      if ( apps[id].init[i] == 0 )
        break;
    }

    for ( i = 0; i < 255; i++ ) {

      dbg("BIN", "Timer: 0x%x \n", apps[id].timer[i]);
      if ( apps[id].timer[i] == 0 )
        break; 
    }
#endif
    
    next_app();
    
    return id;
  }
  
  task void execute_instructionTask() {

    uint8_t *instr;
    uint32_t delay;
    int8_t r; // temp register
    app *p = &apps[active_app];

    /* In case the task is posted, but the binary unloaded */
    if ( active_app == MAX_APPS )
      return;


    if ( p->handler == INIT_HANDLER )
      instr = p->init + p->PC;
    else if ( p->handler == TIMER_HANDLER )
      instr = p->timer + p->PC;
    else 
      instr = p->msg + p->PC;
      
     if ( apps[active_app].waiting_sensor != MAX_REGS )
       next_app();
    /* Send command through serial */

    switch ( instr[0] & 0xF0 ) {

      /* ret */
      case 0x00:

        dbg("TASK", "[APP:%d][PC:%d] ret \n\n",active_app, p->PC);
        
        p->handler = p->next_handler; // in case the timer was invoked
        p->next_handler = NULL_HANDLER;
        p->PC = 0;

        
      break;

      /* set */
      case 0x10:

        r = instr[0] & 0x0F;
        dbg("TASK", "[APP:%d][PC:%d] set: r%d = %d\n",active_app, p->PC, r, (int8_t) instr[1]);
        p->PC += 2;

        /* Error checking */
        if ( r <= 0 || r > MAX_REGS ) 
          break;

        p->regs[r-1] = (int8_t) instr[1];

      break;
      
      /* cpy */
      case 0x20:
  
        r = instr[0] & 0x0F;

        dbg("TASK", "[APP:%d][PC:%d] cpy: r%d = r%d\n",active_app, p->PC, r, instr[1]);
        p->PC += 2;  
        
        /* Error checking */
        if ( r <= 0 || r > MAX_REGS || instr[1] <= 0 || instr[1] > MAX_REGS ) 
          break;
  
        p->regs[r-1] = p->regs[instr[1]-1];

      break;
      
      /* add */
      case 0x30:
      
        r = instr[0] & 0x0F;
        
        dbg("TASK", "[APP:%d][PC:%d] add: r%d = r%d + r%d \n",active_app, p->PC, r, r, instr[1]); 
        p->PC += 2;         
        
        /* Error checking */
        if ( r <= 0 || r > MAX_REGS || instr[1] <= 0 || instr[1] > MAX_REGS ) 
          break;

        p->regs[r-1] += p->regs[instr[1] - 1];

      break;

      /* sub */
      case 0x40:
  
        r = instr[0] & 0x0F;

        dbg("TASK", "[APP:%d][PC:%d] sub: r%d = r%d - r%d \n",active_app, p->PC, r, r, instr[1]);
        p->PC += 2; 

        /* Error checking */
        if ( r <= 0 || r > MAX_REGS || instr[1] <= 0 || instr[1] > MAX_REGS ) 
          break;

        p->regs[r-1] -= p->regs[instr[1] - 1];

      break;

      /* inc */
      case 0x50:

        r = instr[0] & 0x0F;

        dbg("TASK", "[APP:%d][PC:%d] inc: r%d++ value:%d \n",active_app, p->PC, r,p->regs[r-1] );
        p->PC += 1; 

        /* Error checking */
        if ( r <= 0 || r > MAX_REGS ) 
          break;

        p->regs[r-1]++;

      break;

      /* dec */
      case 0x60:
  
        r = instr[0] & 0x0F;

        dbg("TASK", "[APP:%d][PC:%d] dec: r%d-- \n",active_app, p->PC, r );
        p->PC += 1;

        /* Error checking */
        if ( r <= 0 || r > MAX_REGS ) 
          break;

        p->regs[r-1]--;

      break;

      /* max */ 
      case 0x70:
  
        r = instr[0] & 0x0F;

        dbg("TASK", "[APP:%d][PC:%d] max: r%d = max(r%d,r%d) \n",active_app, p->PC, r, r, instr[1] );
        p->PC += 2;

        /* Error checking */
        if ( r <= 0 || r > MAX_REGS || instr[1] <= 0 || instr[1] > MAX_REGS ) 
          break;
        
        if ( p->regs[r-1] < p->regs[instr[1]-1] )
          p->regs[r-1] = p->regs[instr[1]-1];
  
      break;
     
      /* min */
      case 0x80:
      
        r = instr[0] & 0x0F;
        
        dbg("TASK", "[APP:%d][PC:%d] min: r%d = min(r%d,r%d) \n",active_app, p->PC, r, r, instr[1] );
        p->PC += 2;
        
        /* Error checking */
        if ( r <= 0 || r > MAX_REGS || instr[1] <= 0 || instr[1] > MAX_REGS ) 
          break;
        
        if ( p->regs[r-1] > p->regs[instr[1]-1] )
          p->regs[r-1] = p->regs[instr[1]-1];
        
      break;
      
      /* bgz */
      case 0x90:
  
        r = instr[0] & 0x0F;
        
        dbg("TASK", "[APP:%d][PC:%d] bgz: if ( r%d (= %d) > 0 ) pc = pc + %d \n",active_app, p->PC, r, p->regs[r-1], (int8_t) instr[1] );
        p->PC++;
        
        /* Error checking */
        if ( r <= 0 || r > MAX_REGS ) {
          p->PC++;
          break;
        }

        if ( p->regs[r-1] > 0 )
          p->PC += (int8_t) instr[1];
        else
          p->PC++;
          
      break;
      
      /* bez */
      case 0xA0:
  
        r = instr[0] & 0x0F;
        
        dbg("TASK", "[APP:%d][PC:%d] bez: if ( r%d (= %d) == 0 ) pc = pc + %d \n",active_app, p->PC, r,p->regs[r-1],  (int8_t) instr[1] );
        p->PC++;
        
        /* Error checking */
        if ( r <= 0 || r > MAX_REGS ) {
          p->PC++;
          break;
        }
        
        if ( p->regs[r-1] == 0 )
          p->PC += (int8_t) instr[1];
        else
          p->PC++;

      break;
      
      /* bra */
      case 0xB0:

        dbg("TASK", "[APP:%d][PC:%d] bra: pc = pc + %d \n",active_app, p->PC, (int8_t) instr[1] );
        p->PC += 1 + (int8_t) instr[1];

      break;  
  
      /* led */
      case 0xC0:
  
        r = instr[0] & 0x0F;

        dbg("TASK", "[APP:%d][PC:%d] led: %d  \n",active_app, p->PC, r );
	
        /* Each app Opens a different led */
        switch ( active_app ) {

          case 0:

            if ( r )
              call Leds.led0On();
            else
              call Leds.led0Off();

          break;

          case 1:

            if ( r )
              call Leds.led1On();
            else
              call Leds.led1Off();

          break;

          case 2:

            if ( r )
              call Leds.led2On();
            else
              call Leds.led2Off();

          break;

        }

        p->PC += 1;

      break;

      /* rdb */
      case 0xD0:

        r = instr[0] & 0x0F; 

        p->PC += 1;

        /* Error checking */
        if ( r <= 0 || r > MAX_REGS )
          break;

        read_sensor( r - 1);
        dbg("TASK", "[APP:%d][PC:%d] rdb: r%d \n",active_app, p->PC, r );

      break;

      /* tmr */
      case 0xE0:
             
        r = instr[0] & 0x0F;
        
        dbg("TASK", "[APP:%d][PC:%d] tmr: Set timer to %d sec. Hops: %d \n",active_app, p->PC, instr[1], p->hops );
                
        /* If aggregation mode is active, set delay */
        if ( r == 1 )
          delay = 150*(MAX_HOPS - p->hops);
        else
          delay = 0;
          
        if ( instr[1] == 0 )
          call Timer.stop[active_app]( );
        else
          call Timer.startPeriodic[active_app]( instr[1] * 1000 + delay );

        p->PC += 2;

      break;

      
      /* snd */
      case 0xF0:

        r = instr[0] & 0x0F;
        
        dbg("TASK", "[APP:%d][PC:%d] snd: Send r6(%d) \n",active_app, p->PC, p->regs[6] );

        /* Send message over radio */
        sendMsg(p->sink, p->id,p->regs[6], p->regs[7], r);
        
        p->PC += 1;

      break;
    }

    /* Choose next application */
    next_app();

  }


  event void Boot.booted()  {

    system_init();
    call SerialAMControl.start();
    call AMControl.start();
    /* Start a timer in order to know time */
    call Timer.startPeriodic[0]( 1000 );
        
  }

  event void Timer.fired[uint8_t id]()  {


      serial_response* returnmes = (serial_response *) (call SerialPacket.getPayload(&serial_p,sizeof(serial_response)));
      returnmes->instr = 1;
      returnmes->pc = 1;
      returnmes->appid = 1;
      if (call SerialAMSend.send(AM_BROADCAST_ADDR, &serial_p, sizeof(serial_response)) == SUCCESS)
        serial_busy = TRUE;

    if ( !apps[id].isActive || apps[id].PC != 0 )
      return;
      
    dbg("TIMER", "[%s] Timer[%d] was fired.\n", sim_time_string() ,id );
    
    /* If the application is already executing, store next_handler
     * in order not to loose the invocation                        */
    if ( apps[id].handler == NULL_HANDLER )
      apps[id].handler = TIMER_HANDLER;
    else
      apps[id].next_handler = TIMER_HANDLER;

    /* If no other applications are running, need to  
     * post task in order to start executing me       */
    if ( active_app == MAX_APPS ) {
    
      active_app = id;
      post execute_instructionTask();
    
    } 
  }
  
  /* Heartbeat timer */
  event void HeartbeatTimer.fired()  {
    
    /* Send heartbeat as a fake binary */
    load_binary ( NULL, HEARTBEAT, local_unique_id, 0, NULL );
    local_unique_id++;
    
  }
  
  event void Read.readDone( error_t result, uint16_t val )  {
       
    uint8_t i;
       
    /* Update cache */
    if ( val > 127 )
      cache_value = 127;
    else 
      cache_value = (uint8_t) val;
    cache_time = call Timer.getNow[0]();

    /* Search for all the apps and store value */
    for ( i = 0; i < MAX_APPS; i++ ) {
      if ( apps[i].isActive && apps[i].waiting_sensor != MAX_REGS) {
        int8_t r = apps[i].waiting_sensor;

        if ( val > 127 )
          apps[i].regs[r] = 127;
        else
          apps[i].regs[r] = (uint8_t) val;
        apps[i].waiting_sensor = MAX_REGS;

        dbg("SENSOR", "Read from sensor:%d and stored it into r%d \n", val, r + 1 );
      }
     }
  }
  
  default command void Timer.startOneShot[uint8_t id](uint32_t milli){
    dbg("ERR", "Error in timer startOneShot.\n");
  }

  default command void Timer.startPeriodic[uint8_t id](uint32_t milli){
    dbg("ERR", "Error in timer startPeriodic.\n");
  }

  default command void Timer.stop[uint8_t id](){
    dbg("ERR", "Error in timer stop.\n");
  }
  
  default command uint32_t Timer.getNow[uint8_t id](){
    dbg("ERR", "Error in timer getNow.\n");
    return 0;
  }
  
  /***************************************************************************************************/
  /*************************************** Radio Communication ***************************************/
  /***************************************************************************************************/
  
  /* Broadcast an application for installation or removal */
  void forward_app( uint8_t *bin, uint8_t appid, uint8_t type, uint8_t uid, uint8_t hops ) {
    
    uint8_t i;
    
    if ( !busy ) {
      
        binary_msg* fwd_msg = (binary_msg *) (call RadioPacket.getPayload(&radio_p,sizeof(binary_msg)));
        
        if ( fwd_msg == NULL ) {
          dbg("ERR", "Error in forward_app.\n");
          return;
        }
        
        if ( bin != NULL )
          for ( i = 0; i < bin[0]; i++ )
            fwd_msg->binary[i] = bin[i];
          
        fwd_msg->type = type; // INSTALL or UNINSTALL
        fwd_msg->appid = appid;
        fwd_msg->group_id = GROUP_ID;
        fwd_msg->unique_id = uid;
        fwd_msg->hops = hops + 1;
        
        /* Broadcast it */
        if (call RadioAMSend.send(AM_BROADCAST_ADDR, &radio_p, sizeof(binary_msg)) == SUCCESS)
          busy = TRUE;
        else
          dbg("ERR", "Error in forward_app 2.\n");
    }
    
  }
  
  /* Send an application message over radio */
  void sendMsg( am_addr_t sink, uint8_t id, int8_t r7,  int8_t r8,  int8_t sendboth ) {

    if ( !busy ) {
    
      radio_msg *message = (radio_msg*) (call RadioPacket.getPayload(&radio_p,sizeof(radio_msg)));         

      /* If I am sink return result to serial */
      if ( sink == call RadioAMPacket.address() ) {
      
        send_serial_msg( r7, r8, sendboth );
        return;
      }
      
      message->results[0] = r7;
      if ( sendboth )
        message->results[1] = r8;

      message->group_id = GROUP_ID;
      message->appid = id;

      dbg("ERR", "Sending results to %d\n",sink);
      
      call RadioAMPacket.setDestination(&radio_p, sink);
      if (call RadioAMSend.send(sink, &radio_p, sizeof(radio_msg)) == SUCCESS)
        busy = TRUE;
      else 
        dbg("ERR", "Something went wrong.\n");
        
    }
  }
  
  /* Radio message receive */
  event message_t* RadioReceive.receive(message_t* msg, void* payload, uint8_t len) {
    
    int8_t appid;
    
    /* Get a binary message */
    if (len == sizeof(binary_msg)) {

      binary_msg* radioMsg = (binary_msg*)payload;
      
      /* Search cache for message */
      if ( radioMsg->group_id == GROUP_ID && !search_cache( radioMsg->unique_id ) ) {

        if ( radioMsg->type == INSTALL ) {
          
          appid = load_binary( (uint8_t *)radioMsg->binary, radioMsg->appid, radioMsg->unique_id, radioMsg->hops, msg );
            
          /* Store my father as the first who sent me a message */
          if ( appid != -1 )
            apps[appid].sink = call RadioAMPacket.source(msg);
            
        }
        else {
          
          unload_binary(radioMsg->appid, radioMsg->unique_id);
            
        }
        
        /* Store message in cache */
        store_in_cache( radioMsg->unique_id );

      }
    }
    else if ( len == sizeof( radio_msg ) ) {
    
      uint8_t i;
      radio_msg* radioMsg = (radio_msg*)payload;
      
      /* If the application provides a message handler, activate it */
      for ( i = 0; i < MAX_APPS && radioMsg->group_id == GROUP_ID; i++ ){
      
        /* Search for application */
        if ( apps[i].id == radioMsg->appid && apps[i].isActive ) {
          
          if ( apps[i].hasMSGHandler ) {
          
            apps[i].regs[8] = radioMsg->results[0];
            apps[i].regs[9] = radioMsg->results[1];
            
            if ( apps[i].handler != NULL_HANDLER && apps[i].handler != MSG_HANDLER )
              apps[i].next_handler = MSG_HANDLER;
            else
              apps[i].handler = MSG_HANDLER;
            
            dbg("ERR", "Eimai o %d kai pira: %d - %d\n", TOS_NODE_ID, apps[i].regs[8],apps[i].regs[9] );
            /* Invoke */
            if ( active_app == MAX_APPS )
              next_app();
            
          }
          else {
          
            sendMsg(apps[i].sink, apps[i].id, apps[i].regs[9], apps[i].regs[10], 1);
            
          }
          break;
        }
      
      }
    
    }
 
    return msg;

  }
  
  
  /* When a message is sent, change the busy state */
  event void RadioAMSend.sendDone(message_t* msg, error_t err)   {

    if (&radio_p == msg)
      busy = FALSE;

  }
        
  event void AMControl.startDone( error_t err ) {

    if (err != SUCCESS) {
    
      call AMControl.start();
      
    }
    else {
      if ( TOS_NODE_ID == 0) {
        uint8_t bin1[] = { 0x0C, 0x03, 0x05, 0x00, 0xE0, 0x3C, 0x00, 0xD7, 0xF0, 0xE0, 0x3C, 0x00 };
        uint8_t bin2[] = { 0x10, 0x03, 0x05, 0x04, 0xE0, 0x3C, 0x00, 0xD7, 0xF0, 0xE0, 0x3C, 0x00, 0x27, 0x09, 0xF0, 0x00 };
        uint8_t bin3[] = { 0x1C, 0x07, 0x0C, 0x05, 0x17, 0x00, 0x18, 0x7F, 0xE1, 0x3C, 0x00, 0xD1, 0x57, 0x88, 0x01, 0xF1, 0x17, 0x00, 0x18, 0x7F, 0xE1, 0x3C, 0x00, 0x37, 0x09, 0x88, 0x0A, 0x00 };
        /* Store message in cache */
        store_in_cache( local_unique_id );
        load_binary ( bin3, 1, local_unique_id, 0, NULL );
        local_unique_id++;
        call HeartbeatTimer.startPeriodic(5000);
      }
    }

  }

  event void AMControl.stopDone(error_t err) { }
  
  /***************************************************************************************************/
  /*************************************** Serial Communication **************************************/
  /***************************************************************************************************/
  
  void send_serial_msg( uint8_t instr, uint8_t appid, uint8_t pc ) {
  
      if ( !serial_busy ) {
      
        serial_response* returnmes = (serial_response *) (call SerialPacket.getPayload(&serial_p,sizeof(serial_response)));
        returnmes->instr = instr;
	returnmes->pc = pc;
	returnmes->appid = appid;
	if (call SerialAMSend.send(AM_BROADCAST_ADDR, &serial_p, sizeof(serial_response)) == SUCCESS)
          serial_busy = TRUE;
          
      }
  }
  
  event void SerialAMControl.startDone( error_t err ) {
  
    if (err != SUCCESS)
      call SerialAMControl.start();

  }

  event void SerialAMControl.stopDone(error_t err) { }


  event void SerialAMSend.sendDone(message_t* msg, error_t err)  {

    if (&serial_p == msg)
      serial_busy = FALSE;

  }

  event message_t* SerialReceive.receive(message_t* msg, void* payload, uint8_t len) {

    binary_msg* serial_msg = (binary_msg *) payload;
    int8_t appid;
    
    if ( serial_msg->type == INSTALL ) {

      appid = load_binary( (uint8_t *)serial_msg->binary, serial_msg->appid, local_unique_id, 0, NULL );
      local_unique_id++;
      
      if ( appid != -1 )
        apps[appid].sink = call RadioAMPacket.address();
        
      
      call HeartbeatTimer.startPeriodic(5000);
      
    }
    else if ( serial_msg->type == UNINSTALL ){
    
      unload_binary ( serial_msg->appid, local_unique_id );
      local_unique_id++;
      
    }

    return msg;
    
  }
  
}
