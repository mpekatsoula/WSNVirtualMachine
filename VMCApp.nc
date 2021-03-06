configuration VMCApp
{
}
implementation
{
  components MainC, VMC, LedsC;
  components new TimerMilliC() as Timer0;
  components new TimerMilliC() as Timer1;
  components new TimerMilliC() as Timer2;
  components new TimerMilliC() as Timer3;
  components new DemoSensorC() as Sensor;
  components SerialActiveMessageC;
  components ActiveMessageC;
  components new SerialAMReceiverC(6);
  components new SerialAMSenderC(6);
  components new AMSenderC(7);
  components new AMReceiverC(7);
  
  VMC.Boot -> MainC;

  VMC.Timer[0] -> Timer0;
  VMC.Timer[1] -> Timer1;
  VMC.Timer[2] -> Timer2;
  VMC.HeartbeatTimer -> Timer3;
  VMC.Leds -> LedsC;
  
  /* Serial Communication */
  VMC.SerialPacket -> SerialAMSenderC;
  VMC.SerialAMPacket -> SerialAMSenderC;
  VMC.SerialAMSend -> SerialAMSenderC;
  VMC.SerialAMControl -> SerialActiveMessageC;
  VMC.SerialReceive -> SerialAMReceiverC;

  /* Radio Communication */
  VMC.AMControl -> ActiveMessageC;
  VMC.RadioPacket -> AMSenderC;
  VMC.RadioAMPacket -> AMSenderC;
  VMC.RadioAMSend -> AMSenderC;
  VMC.RadioReceive -> AMReceiverC;
  
  /* Sensor Connection */
  VMC.Read -> Sensor;
}

