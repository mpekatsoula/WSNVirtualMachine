module VMC
{
  uses interface Timer<TMilli> as Timer0;
  uses interface Leds;
  uses interface Boot;
}
implementation
{


  task void next_instruction() {
  
    uint8_t *instr;
  
    switch ( instr[0] & 0xF0 ) {
  
      /* ret */
      case 0x00:
  
  
      break;
  
      /* set */
      case 0x01:
  
  
      break;
      
      /* cpy */
      case 0x02:
  
  
      break;
      
      /* add */
      case 0x03:
  
  
      break;
            
      /* sub */
      case 0x04:
  
  
      break;
      
      /* inc */
      case 0x05:
  
  
      break;
           
      /* dec */
      case 0x06:
  
  
      break;
      
      /* max */ 
      case 0x07:
  
  
      break;
     
      /* min */
      case 0x08:
  
  
      break;
      
      /* bgz */
      case 0x09:
  
  
      break;
      
      /* bez */
      case 0x0A:
  
  
      break;
      
      /* bra */
      case 0x0B:
  
  
      break;  
  
      /* led */
      case 0x0C:
  
  
      break;
      
      /* rdb */
      case 0x0D:
  
  
      break;      
  
      /* tmr */
      case 0x0D:
  
  
      break;   
      
  
    }
  
  }






  event void Boot.booted()
  {
    call Timer0.startPeriodic( 250 );
  }

  event void Timer0.fired()
  {
    dbg("BlinkC", "Timer 0 fired @ %s.\n", sim_time_string());
    call Leds.led0Toggle();
  }
  

}

