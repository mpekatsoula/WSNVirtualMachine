#define nx_struct struct
#define nx_union union
#define dbg(mode, format, ...) ((void)0)
#define dbg_clear(mode, format, ...) ((void)0)
#define dbg_active(mode) 0
# 151 "/usr/lib/gcc/xscale-elf/3.4.3/include/stddef.h" 3
typedef long int ptrdiff_t;
#line 213
typedef long unsigned int size_t;
#line 325
typedef int wchar_t;
# 8 "/usr/lib/ncc/deputy_nodeputy.h"
struct __nesc_attr_nonnull {
}  ;
#line 9
struct __nesc_attr_bnd {
#line 9
  void *lo, *hi;
}  ;
#line 10
struct __nesc_attr_bnd_nok {
#line 10
  void *lo, *hi;
}  ;
#line 11
struct __nesc_attr_count {
#line 11
  int n;
}  ;
#line 12
struct __nesc_attr_count_nok {
#line 12
  int n;
}  ;
#line 13
struct __nesc_attr_one {
}  ;
#line 14
struct __nesc_attr_one_nok {
}  ;
#line 15
struct __nesc_attr_dmemset {
#line 15
  int a1, a2, a3;
}  ;
#line 16
struct __nesc_attr_dmemcpy {
#line 16
  int a1, a2, a3;
}  ;
#line 17
struct __nesc_attr_nts {
}  ;
# 4 "/opt/tinyos-2.1.0/tos/chips/pxa27x/inttypes.h"
typedef signed char int8_t;
typedef unsigned char uint8_t;

typedef short int16_t;
typedef unsigned short uint16_t;

typedef int int32_t;
typedef unsigned int uint32_t;

typedef long long int64_t;
typedef unsigned long long uint64_t;

typedef int32_t intptr_t;
typedef uint32_t uintptr_t;
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline uint8_t __nesc_ntoh_uint8(const void * source)  ;




static __inline uint8_t __nesc_hton_uint8(void * target, uint8_t value)  ;
#line 264
static __inline uint16_t __nesc_ntoh_uint16(const void * source)  ;




static __inline uint16_t __nesc_hton_uint16(void * target, uint16_t value)  ;
#line 385
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_int8_t;typedef int8_t __nesc_nxbase_nx_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_int16_t;typedef int16_t __nesc_nxbase_nx_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_int32_t;typedef int32_t __nesc_nxbase_nx_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_int64_t;typedef int64_t __nesc_nxbase_nx_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nx_uint8_t;typedef uint8_t __nesc_nxbase_nx_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nx_uint16_t;typedef uint16_t __nesc_nxbase_nx_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nx_uint32_t;typedef uint32_t __nesc_nxbase_nx_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nx_uint64_t;typedef uint64_t __nesc_nxbase_nx_uint64_t  ;


typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_int8_t;typedef int8_t __nesc_nxbase_nxle_int8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_int16_t;typedef int16_t __nesc_nxbase_nxle_int16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_int32_t;typedef int32_t __nesc_nxbase_nxle_int32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_int64_t;typedef int64_t __nesc_nxbase_nxle_int64_t  ;
typedef struct { unsigned char data[1]; } __attribute__((packed)) nxle_uint8_t;typedef uint8_t __nesc_nxbase_nxle_uint8_t  ;
typedef struct { unsigned char data[2]; } __attribute__((packed)) nxle_uint16_t;typedef uint16_t __nesc_nxbase_nxle_uint16_t  ;
typedef struct { unsigned char data[4]; } __attribute__((packed)) nxle_uint32_t;typedef uint32_t __nesc_nxbase_nxle_uint32_t  ;
typedef struct { unsigned char data[8]; } __attribute__((packed)) nxle_uint64_t;typedef uint64_t __nesc_nxbase_nxle_uint64_t  ;
# 6 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/lock.h" 3
typedef int _LOCK_T;
typedef int _LOCK_RECURSIVE_T;
# 14 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/_types.h" 3
typedef long _off_t;
__extension__ 
#line 15
typedef long long _off64_t;


typedef int _ssize_t;
# 354 "/usr/lib/gcc/xscale-elf/3.4.3/include/stddef.h" 3
typedef unsigned int wint_t;
# 35 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/_types.h" 3
#line 27
typedef struct __nesc_unnamed4242 {

  int __count;
  union __nesc_unnamed4243 {

    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;

typedef _LOCK_RECURSIVE_T _flock_t;


typedef void *_iconv_t;
# 19 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/sys/reent.h" 3
typedef unsigned long __ULong;
#line 40
struct _Bigint {

  struct _Bigint *_next;
  int _k, _maxwds, _sign, _wds;
  __ULong _x[1];
};


struct __tm {

  int __tm_sec;
  int __tm_min;
  int __tm_hour;
  int __tm_mday;
  int __tm_mon;
  int __tm_year;
  int __tm_wday;
  int __tm_yday;
  int __tm_isdst;
};







struct _on_exit_args {
  void *_fnargs[32];
  void *_dso_handle[32];

  __ULong _fntypes;


  __ULong _is_cxa;
};









struct _atexit {
  struct _atexit *_next;
  int _ind;

  void (*_fns[32])(void );
  struct _on_exit_args _on_exit_args;
};









struct __sbuf {
  unsigned char *_base;
  int _size;
};






typedef long _fpos_t;
#line 166
struct __sFILE {
  unsigned char *_p;
  int _r;
  int _w;
  short _flags;
  short _file;
  struct __sbuf _bf;
  int _lbfsize;






  char *_cookie;

  int (*_read)();
  int (*_write)();

  _fpos_t (*_seek)();
  int (*_close)();


  struct __sbuf _ub;
  unsigned char *_up;
  int _ur;


  unsigned char _ubuf[3];
  unsigned char _nbuf[1];


  struct __sbuf _lb;


  int _blksize;
  int _offset;


  struct _reent *_data;



  _flock_t _lock;
};
#line 259
typedef struct __sFILE __FILE;


struct _glue {

  struct _glue *_next;
  int _niobs;
  __FILE *_iobs;
};
#line 290
struct _rand48 {
  unsigned short _seed[3];
  unsigned short _mult[3];
  unsigned short _add;
};
#line 565
struct _reent {

  int _errno;




  __FILE *_stdin, *_stdout, *_stderr;

  int _inc;
  char _emergency[25];

  int _current_category;
  char *_current_locale;

  int __sdidinit;

  void (*__cleanup)();


  struct _Bigint *_result;
  int _result_k;
  struct _Bigint *_p5s;
  struct _Bigint **_freelist;


  int _cvtlen;
  char *_cvtbuf;

  union __nesc_unnamed4244 {

    struct __nesc_unnamed4245 {

      unsigned int _unused_rand;
      char *_strtok_last;
      char _asctime_buf[26];
      struct __tm _localtime_buf;
      int _gamma_signgam;
      __extension__ unsigned long long _rand_next;
      struct _rand48 _r48;
      _mbstate_t _mblen_state;
      _mbstate_t _mbtowc_state;
      _mbstate_t _wctomb_state;
      char _l64a_buf[8];
      char _signal_buf[24];
      int _getdate_err;
      _mbstate_t _mbrlen_state;
      _mbstate_t _mbrtowc_state;
      _mbstate_t _mbsrtowcs_state;
      _mbstate_t _wcrtomb_state;
      _mbstate_t _wcsrtombs_state;
    } _reent;



    struct __nesc_unnamed4246 {


      unsigned char *_nextf[30];
      unsigned int _nmalloc[30];
    } _unused;
  } _new;


  struct _atexit *_atexit;
  struct _atexit _atexit0;


  void (**_sig_func)(int arg_0x40297010);




  struct _glue __sglue;
  __FILE __sf[3];
};
#line 799
struct _reent;
struct _reent;
# 24 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/string.h" 3
char *memcpy();

char *memset();
# 28 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/stdlib.h" 3
#line 24
typedef struct __nesc_unnamed4247 {

  int quot;
  int rem;
} div_t;





#line 30
typedef struct __nesc_unnamed4248 {

  long quot;
  long rem;
} ldiv_t;






#line 37
typedef struct __nesc_unnamed4249 {

  long long int quot;
  long long int rem;
} lldiv_t;
# 17 "/usr/lib/gcc/xscale-elf/3.4.3/../../../../xscale-elf/include/math.h" 3
union __dmath {

  __ULong i[2];
  double d;
};




union __dmath;
#line 72
typedef float float_t;
typedef double double_t;
#line 292
struct exception {


  int type;
  char *name;
  double arg1;
  double arg2;
  double retval;
  int err;
};
#line 347
enum __fdlibm_version {

  __fdlibm_ieee = -1, 
  __fdlibm_svid, 
  __fdlibm_xopen, 
  __fdlibm_posix
};




enum __fdlibm_version;
# 23 "/opt/tinyos-2.1.0/tos/system/tos.h"
typedef uint8_t bool;
enum __nesc_unnamed4250 {
#line 24
  FALSE = 0, TRUE = 1
};
typedef nx_int8_t nx_bool;







struct __nesc_attr_atmostonce {
};
#line 35
struct __nesc_attr_atleastonce {
};
#line 36
struct __nesc_attr_exactlyonce {
};
# 40 "/opt/tinyos-2.1.0/tos/types/TinyError.h"
enum __nesc_unnamed4251 {
  SUCCESS = 0, 
  FAIL = 1, 
  ESIZE = 2, 
  ECANCEL = 3, 
  EOFF = 4, 
  EBUSY = 5, 
  EINVAL = 6, 
  ERETRY = 7, 
  ERESERVE = 8, 
  EALREADY = 9, 
  ENOMEM = 10, 
  ENOACK = 11, 
  ELAST = 11
};

typedef uint8_t error_t  ;

static inline error_t ecombine(error_t r1, error_t r2)  ;
# 83 "/opt/tinyos-2.1.0/tos/chips/pxa27x/pxa27xhardware.h"
extern void enableICache();
extern void initSyncFlash();

static __inline uint32_t _pxa27x_clzui(uint32_t i);





typedef uint32_t __nesc_atomic_t;


__inline __nesc_atomic_t __nesc_atomic_start(void )  ;
#line 111
__inline void __nesc_atomic_end(__nesc_atomic_t oldState)  ;
#line 129
static __inline void __nesc_enable_interrupt();
#line 143
static __inline void __nesc_disable_interrupt();
#line 169
typedef uint8_t mcu_power_t  ;
# 107 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/hardware.h"
enum __nesc_unnamed4252 {
  TOSH_period16 = 0x00, 
  TOSH_period32 = 0x01, 
  TOSH_period64 = 0x02, 
  TOSH_period128 = 0x03, 
  TOSH_period256 = 0x04, 
  TOSH_period512 = 0x05, 
  TOSH_period1024 = 0x06, 
  TOSH_period2048 = 0x07
};





const uint8_t TOSH_IRP_TABLE[40] = { 0x05, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0x04, 
0x01, 
0x03, 
0x02, 
0x08, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0x07, 
0xFF, 
0x06, 
0xFF, 
0x0A, 
0x00, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0x09, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF, 
0xFF };
#line 243
static inline void TOSH_SET_PIN_DIRECTIONS(void );
# 29 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.h"
typedef struct __nesc_unnamed4253 {
#line 29
  int notUsed;
} 
#line 29
TMilli;
typedef struct __nesc_unnamed4254 {
#line 30
  int notUsed;
} 
#line 30
T32khz;
typedef struct __nesc_unnamed4255 {
#line 31
  int notUsed;
} 
#line 31
TMicro;
# 19 "constants.h"
enum __nesc_unnamed4256 {
#line 19
  AM_NODETOPC = 6
};
#line 34
#line 22
typedef struct _app {

  uint8_t init[255];
  uint8_t timer[255];
  uint8_t id;
  uint8_t PC;
  uint8_t handler;
  uint8_t next_handler;
  int8_t waiting_sensor;
  int8_t regs[6];
  bool isActive;
} 
app;








#line 37
typedef struct test_serial_msg {

  nx_uint8_t type;
  nx_uint8_t appid;
  nx_uint8_t binary[32];
} 
test_serial_msg;








#line 45
typedef struct test_serial_msg_resp {

  nx_uint8_t instr;
  nx_uint8_t pc;
  nx_uint8_t appid;
} 

serial_response;
# 32 "/opt/tinyos-2.1.0/tos/types/Leds.h"
enum __nesc_unnamed4257 {
  LEDS_LED0 = 1 << 0, 
  LEDS_LED1 = 1 << 1, 
  LEDS_LED2 = 1 << 2, 
  LEDS_LED3 = 1 << 3, 
  LEDS_LED4 = 1 << 4, 
  LEDS_LED5 = 1 << 5, 
  LEDS_LED6 = 1 << 6, 
  LEDS_LED7 = 1 << 7
};
# 39 "/opt/tinyos-2.1.0/tos/chips/cc2420/CC2420.h"
typedef uint8_t cc2420_status_t;
#line 66
#line 49
typedef nx_struct cc2420_header_t {
  nxle_uint8_t length;
  nxle_uint16_t fcf;
  nxle_uint8_t dsn;
  nxle_uint16_t destpan;
  nxle_uint16_t dest;
  nxle_uint16_t src;



  nxle_uint8_t network;



  nxle_uint8_t type;
} __attribute__((packed)) 

cc2420_header_t;





#line 71
typedef nx_struct cc2420_footer_t {
} __attribute__((packed)) cc2420_footer_t;
#line 97
#line 81
typedef nx_struct cc2420_metadata_t {
  nx_uint8_t rssi;
  nx_uint8_t lqi;
  nx_uint8_t tx_power;
  nx_bool crc;
  nx_bool ack;
  nx_bool timesync;
  nx_uint32_t timestamp;
  nx_uint16_t rxInterval;
} __attribute__((packed)) 






cc2420_metadata_t;





#line 100
typedef nx_struct cc2420_packet_t {
  cc2420_header_t packet;
  nx_uint8_t data[];
} __attribute__((packed)) cc2420_packet_t;
#line 134
enum __nesc_unnamed4258 {

  MAC_HEADER_SIZE = sizeof(cc2420_header_t ) - 1, 

  MAC_FOOTER_SIZE = sizeof(uint16_t ), 

  MAC_PACKET_SIZE = MAC_HEADER_SIZE + 100 + MAC_FOOTER_SIZE, 

  CC2420_SIZE = MAC_HEADER_SIZE + MAC_FOOTER_SIZE
};

enum cc2420_enums {
  CC2420_TIME_ACK_TURNAROUND = 7, 
  CC2420_TIME_VREN = 20, 
  CC2420_TIME_SYMBOL = 2, 
  CC2420_BACKOFF_PERIOD = 20 / CC2420_TIME_SYMBOL, 
  CC2420_MIN_BACKOFF = 20 / CC2420_TIME_SYMBOL, 
  CC2420_ACK_WAIT_DELAY = 256
};

enum cc2420_status_enums {
  CC2420_STATUS_RSSI_VALID = 1 << 1, 
  CC2420_STATUS_LOCK = 1 << 2, 
  CC2420_STATUS_TX_ACTIVE = 1 << 3, 
  CC2420_STATUS_ENC_BUSY = 1 << 4, 
  CC2420_STATUS_TX_UNDERFLOW = 1 << 5, 
  CC2420_STATUS_XOSC16M_STABLE = 1 << 6
};

enum cc2420_config_reg_enums {
  CC2420_SNOP = 0x00, 
  CC2420_SXOSCON = 0x01, 
  CC2420_STXCAL = 0x02, 
  CC2420_SRXON = 0x03, 
  CC2420_STXON = 0x04, 
  CC2420_STXONCCA = 0x05, 
  CC2420_SRFOFF = 0x06, 
  CC2420_SXOSCOFF = 0x07, 
  CC2420_SFLUSHRX = 0x08, 
  CC2420_SFLUSHTX = 0x09, 
  CC2420_SACK = 0x0a, 
  CC2420_SACKPEND = 0x0b, 
  CC2420_SRXDEC = 0x0c, 
  CC2420_STXENC = 0x0d, 
  CC2420_SAES = 0x0e, 
  CC2420_MAIN = 0x10, 
  CC2420_MDMCTRL0 = 0x11, 
  CC2420_MDMCTRL1 = 0x12, 
  CC2420_RSSI = 0x13, 
  CC2420_SYNCWORD = 0x14, 
  CC2420_TXCTRL = 0x15, 
  CC2420_RXCTRL0 = 0x16, 
  CC2420_RXCTRL1 = 0x17, 
  CC2420_FSCTRL = 0x18, 
  CC2420_SECCTRL0 = 0x19, 
  CC2420_SECCTRL1 = 0x1a, 
  CC2420_BATTMON = 0x1b, 
  CC2420_IOCFG0 = 0x1c, 
  CC2420_IOCFG1 = 0x1d, 
  CC2420_MANFIDL = 0x1e, 
  CC2420_MANFIDH = 0x1f, 
  CC2420_FSMTC = 0x20, 
  CC2420_MANAND = 0x21, 
  CC2420_MANOR = 0x22, 
  CC2420_AGCCTRL = 0x23, 
  CC2420_AGCTST0 = 0x24, 
  CC2420_AGCTST1 = 0x25, 
  CC2420_AGCTST2 = 0x26, 
  CC2420_FSTST0 = 0x27, 
  CC2420_FSTST1 = 0x28, 
  CC2420_FSTST2 = 0x29, 
  CC2420_FSTST3 = 0x2a, 
  CC2420_RXBPFTST = 0x2b, 
  CC2420_FMSTATE = 0x2c, 
  CC2420_ADCTST = 0x2d, 
  CC2420_DACTST = 0x2e, 
  CC2420_TOPTST = 0x2f, 
  CC2420_TXFIFO = 0x3e, 
  CC2420_RXFIFO = 0x3f
};

enum cc2420_ram_addr_enums {
  CC2420_RAM_TXFIFO = 0x000, 
  CC2420_RAM_RXFIFO = 0x080, 
  CC2420_RAM_KEY0 = 0x100, 
  CC2420_RAM_RXNONCE = 0x110, 
  CC2420_RAM_SABUF = 0x120, 
  CC2420_RAM_KEY1 = 0x130, 
  CC2420_RAM_TXNONCE = 0x140, 
  CC2420_RAM_CBCSTATE = 0x150, 
  CC2420_RAM_IEEEADR = 0x160, 
  CC2420_RAM_PANID = 0x168, 
  CC2420_RAM_SHORTADR = 0x16a
};

enum cc2420_nonce_enums {
  CC2420_NONCE_BLOCK_COUNTER = 0, 
  CC2420_NONCE_KEY_SEQ_COUNTER = 2, 
  CC2420_NONCE_FRAME_COUNTER = 3, 
  CC2420_NONCE_SOURCE_ADDRESS = 7, 
  CC2420_NONCE_FLAGS = 15
};

enum cc2420_main_enums {
  CC2420_MAIN_RESETn = 15, 
  CC2420_MAIN_ENC_RESETn = 14, 
  CC2420_MAIN_DEMOD_RESETn = 13, 
  CC2420_MAIN_MOD_RESETn = 12, 
  CC2420_MAIN_FS_RESETn = 11, 
  CC2420_MAIN_XOSC16M_BYPASS = 0
};

enum cc2420_mdmctrl0_enums {
  CC2420_MDMCTRL0_RESERVED_FRAME_MODE = 13, 
  CC2420_MDMCTRL0_PAN_COORDINATOR = 12, 
  CC2420_MDMCTRL0_ADR_DECODE = 11, 
  CC2420_MDMCTRL0_CCA_HYST = 8, 
  CC2420_MDMCTRL0_CCA_MOD = 6, 
  CC2420_MDMCTRL0_AUTOCRC = 5, 
  CC2420_MDMCTRL0_AUTOACK = 4, 
  CC2420_MDMCTRL0_PREAMBLE_LENGTH = 0
};

enum cc2420_mdmctrl1_enums {
  CC2420_MDMCTRL1_CORR_THR = 6, 
  CC2420_MDMCTRL1_DEMOD_AVG_MODE = 5, 
  CC2420_MDMCTRL1_MODULATION_MODE = 4, 
  CC2420_MDMCTRL1_TX_MODE = 2, 
  CC2420_MDMCTRL1_RX_MODE = 0
};

enum cc2420_rssi_enums {
  CC2420_RSSI_CCA_THR = 8, 
  CC2420_RSSI_RSSI_VAL = 0
};

enum cc2420_syncword_enums {
  CC2420_SYNCWORD_SYNCWORD = 0
};

enum cc2420_txctrl_enums {
  CC2420_TXCTRL_TXMIXBUF_CUR = 14, 
  CC2420_TXCTRL_TX_TURNAROUND = 13, 
  CC2420_TXCTRL_TXMIX_CAP_ARRAY = 11, 
  CC2420_TXCTRL_TXMIX_CURRENT = 9, 
  CC2420_TXCTRL_PA_CURRENT = 6, 
  CC2420_TXCTRL_RESERVED = 5, 
  CC2420_TXCTRL_PA_LEVEL = 0
};

enum cc2420_rxctrl0_enums {
  CC2420_RXCTRL0_RXMIXBUF_CUR = 12, 
  CC2420_RXCTRL0_HIGH_LNA_GAIN = 10, 
  CC2420_RXCTRL0_MED_LNA_GAIN = 8, 
  CC2420_RXCTRL0_LOW_LNA_GAIN = 6, 
  CC2420_RXCTRL0_HIGH_LNA_CURRENT = 4, 
  CC2420_RXCTRL0_MED_LNA_CURRENT = 2, 
  CC2420_RXCTRL0_LOW_LNA_CURRENT = 0
};

enum cc2420_rxctrl1_enums {
  CC2420_RXCTRL1_RXBPF_LOCUR = 13, 
  CC2420_RXCTRL1_RXBPF_MIDCUR = 12, 
  CC2420_RXCTRL1_LOW_LOWGAIN = 11, 
  CC2420_RXCTRL1_MED_LOWGAIN = 10, 
  CC2420_RXCTRL1_HIGH_HGM = 9, 
  CC2420_RXCTRL1_MED_HGM = 8, 
  CC2420_RXCTRL1_LNA_CAP_ARRAY = 6, 
  CC2420_RXCTRL1_RXMIX_TAIL = 4, 
  CC2420_RXCTRL1_RXMIX_VCM = 2, 
  CC2420_RXCTRL1_RXMIX_CURRENT = 0
};

enum cc2420_rsctrl_enums {
  CC2420_FSCTRL_LOCK_THR = 14, 
  CC2420_FSCTRL_CAL_DONE = 13, 
  CC2420_FSCTRL_CAL_RUNNING = 12, 
  CC2420_FSCTRL_LOCK_LENGTH = 11, 
  CC2420_FSCTRL_LOCK_STATUS = 10, 
  CC2420_FSCTRL_FREQ = 0
};

enum cc2420_secctrl0_enums {
  CC2420_SECCTRL0_RXFIFO_PROTECTION = 9, 
  CC2420_SECCTRL0_SEC_CBC_HEAD = 8, 
  CC2420_SECCTRL0_SEC_SAKEYSEL = 7, 
  CC2420_SECCTRL0_SEC_TXKEYSEL = 6, 
  CC2420_SECCTRL0_SEC_RXKEYSEL = 5, 
  CC2420_SECCTRL0_SEC_M = 2, 
  CC2420_SECCTRL0_SEC_MODE = 0
};

enum cc2420_secctrl1_enums {
  CC2420_SECCTRL1_SEC_TXL = 8, 
  CC2420_SECCTRL1_SEC_RXL = 0
};

enum cc2420_battmon_enums {
  CC2420_BATTMON_BATT_OK = 6, 
  CC2420_BATTMON_BATTMON_EN = 5, 
  CC2420_BATTMON_BATTMON_VOLTAGE = 0
};

enum cc2420_iocfg0_enums {
  CC2420_IOCFG0_BCN_ACCEPT = 11, 
  CC2420_IOCFG0_FIFO_POLARITY = 10, 
  CC2420_IOCFG0_FIFOP_POLARITY = 9, 
  CC2420_IOCFG0_SFD_POLARITY = 8, 
  CC2420_IOCFG0_CCA_POLARITY = 7, 
  CC2420_IOCFG0_FIFOP_THR = 0
};

enum cc2420_iocfg1_enums {
  CC2420_IOCFG1_HSSD_SRC = 10, 
  CC2420_IOCFG1_SFDMUX = 5, 
  CC2420_IOCFG1_CCAMUX = 0
};

enum cc2420_manfidl_enums {
  CC2420_MANFIDL_PARTNUM = 12, 
  CC2420_MANFIDL_MANFID = 0
};

enum cc2420_manfidh_enums {
  CC2420_MANFIDH_VERSION = 12, 
  CC2420_MANFIDH_PARTNUM = 0
};

enum cc2420_fsmtc_enums {
  CC2420_FSMTC_TC_RXCHAIN2RX = 13, 
  CC2420_FSMTC_TC_SWITCH2TX = 10, 
  CC2420_FSMTC_TC_PAON2TX = 6, 
  CC2420_FSMTC_TC_TXEND2SWITCH = 3, 
  CC2420_FSMTC_TC_TXEND2PAOFF = 0
};

enum cc2420_sfdmux_enums {
  CC2420_SFDMUX_SFD = 0, 
  CC2420_SFDMUX_XOSC16M_STABLE = 24
};

enum __nesc_unnamed4259 {

  CC2420_INVALID_TIMESTAMP = 0x80000000L
};
# 6 "/opt/tinyos-2.1.0/tos/types/AM.h"
typedef nx_uint8_t nx_am_id_t;
typedef nx_uint8_t nx_am_group_t;
typedef nx_uint16_t nx_am_addr_t;

typedef uint8_t am_id_t;
typedef uint8_t am_group_t;
typedef uint16_t am_addr_t;

enum __nesc_unnamed4260 {
  AM_BROADCAST_ADDR = 0xffff
};









enum __nesc_unnamed4261 {
  TOS_AM_GROUP = 0x22, 
  TOS_AM_ADDRESS = 1
};
# 72 "/opt/tinyos-2.1.0/tos/lib/serial/Serial.h"
typedef uint8_t uart_id_t;



enum __nesc_unnamed4262 {
  HDLC_FLAG_BYTE = 0x7e, 
  HDLC_CTLESC_BYTE = 0x7d
};



enum __nesc_unnamed4263 {
  TOS_SERIAL_ACTIVE_MESSAGE_ID = 0, 
  TOS_SERIAL_CC1000_ID = 1, 
  TOS_SERIAL_802_15_4_ID = 2, 
  TOS_SERIAL_UNKNOWN_ID = 255
};


enum __nesc_unnamed4264 {
  SERIAL_PROTO_ACK = 67, 
  SERIAL_PROTO_PACKET_ACK = 68, 
  SERIAL_PROTO_PACKET_NOACK = 69, 
  SERIAL_PROTO_PACKET_UNKNOWN = 255
};
#line 110
#line 98
typedef struct radio_stats {
  uint8_t version;
  uint8_t flags;
  uint8_t reserved;
  uint8_t platform;
  uint16_t MTU;
  uint16_t radio_crc_fail;
  uint16_t radio_queue_drops;
  uint16_t serial_crc_fail;
  uint16_t serial_tx_fail;
  uint16_t serial_short_packets;
  uint16_t serial_proto_drops;
} radio_stats_t;







#line 112
typedef nx_struct serial_header {
  nx_am_addr_t dest;
  nx_am_addr_t src;
  nx_uint8_t length;
  nx_am_group_t group;
  nx_am_id_t type;
} __attribute__((packed)) serial_header_t;




#line 120
typedef nx_struct serial_packet {
  serial_header_t header;
  nx_uint8_t data[];
} __attribute__((packed)) serial_packet_t;



#line 125
typedef nx_struct serial_metadata {
  nx_uint8_t ack;
} __attribute__((packed)) serial_metadata_t;
# 49 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/platform_message.h"
#line 46
typedef union message_header {
  cc2420_header_t cc2420;
  serial_header_t serial;
} message_header_t;



#line 51
typedef union message_footer {
  cc2420_footer_t cc2420;
} message_footer_t;




#line 55
typedef union message_metadata {
  cc2420_metadata_t cc2420;
  serial_metadata_t serial;
} message_metadata_t;
# 19 "/opt/tinyos-2.1.0/tos/types/message.h"
#line 14
typedef nx_struct message_t {
  nx_uint8_t header[sizeof(message_header_t )];
  nx_uint8_t data[100];
  nx_uint8_t footer[sizeof(message_footer_t )];
  nx_uint8_t metadata[sizeof(message_metadata_t )];
} __attribute__((packed)) message_t;
# 33 "/opt/tinyos-2.1.0/tos/types/Resource.h"
typedef uint8_t resource_client_id_t;
# 40 "/opt/tinyos-2.1.0/tos/types/I2C.h"
typedef struct __nesc_unnamed4265 {
} 
#line 40
TI2CExtdAddr;
typedef struct __nesc_unnamed4266 {
} 
#line 41
TI2CBasicAddr;

typedef uint8_t i2c_flags_t;

enum __nesc_unnamed4267 {
  I2C_START = 0x01, 
  I2C_STOP = 0x02, 
  I2C_ACK_END = 0x04
};
# 80 "/opt/tinyos-2.1.0/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b);
# 36 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/pxa27x_serial.h"
typedef uint8_t uart_status_t;





#line 38
typedef enum __nesc_unnamed4268 {
  EVEN, 
  ODD, 
  NONE
} uart_parity_t;
typedef TMilli PMICM$chargeMonitorTimer$precision_tag;
typedef TMilli /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$precision_tag;
typedef /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$precision_tag;
typedef /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$precision_tag /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$precision_tag;
typedef /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type;
typedef /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$precision_tag /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$precision_tag;
typedef TMilli /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$frequency_tag;
typedef /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$frequency_tag /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$precision_tag;
typedef uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type;
typedef TMilli /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$frequency_tag;
typedef /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$frequency_tag /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$LocalTime$precision_tag;
typedef /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$frequency_tag /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$precision_tag;
typedef uint32_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$size_type;
enum CounterMilliC$__nesc_unnamed4269 {
  CounterMilliC$OST_CLIENT_ID = 1U
};
enum HilTimerMilliC$__nesc_unnamed4270 {
  HilTimerMilliC$OST_CLIENT_ID = 0U
};
typedef uint16_t VMC$Read$val_t;
typedef TMilli VMC$Timer$precision_tag;
typedef uint16_t DemoSensorP$Light$val_t;
typedef uint16_t DemoSensorP$Read$val_t;
typedef uint16_t DemoSensorP$IRLight$val_t;
typedef uint16_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRPhoto$val_t;
typedef uint16_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$val_t;
enum /*VMCApp.Sensor.Sensor*/Tsl2561C$0$__nesc_unnamed4271 {
  Tsl2561C$0$BB_KEY = 0U, Tsl2561C$0$IR_KEY = 1U, Tsl2561C$0$ADV_KEY = 2U, Tsl2561C$0$READER_ID = 0U
};
enum Tsl2561InternalC$__nesc_unnamed4272 {
  Tsl2561InternalC$ADV_ID = 1U
};
typedef TI2CBasicAddr /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$addr_size;
typedef TI2CBasicAddr /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$addr_size;
enum SerialAMQueueP$__nesc_unnamed4273 {
  SerialAMQueueP$NUM_CLIENTS = 1U
};
# 32 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/PlatformReset.nc"
static void PlatformP$PlatformReset$reset(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PlatformP$Init$init(void );
# 139 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void PlatformP$OST0M3$fired(void );
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void PMICM$PI2C$interruptI2C(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PMICM$Init$init(void );
# 56 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMIC.nc"
static error_t PMICM$PMIC$enableManualCharging(bool enable);
#line 51
static error_t PMICM$PMIC$setCoreVoltage(uint8_t trimValue);


static error_t PMICM$PMIC$getBatteryVoltage(uint8_t *val);
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void PMICM$PMICGPIO$interruptGPIOPin(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void PMICM$chargeMonitorTimer$fired(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired(void );
#line 125
static uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$getNow(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x403d9ce0);
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x403d9ce0);
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x403d9ce0, 
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x403d9ce0);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired(void );
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow(void );
#line 118
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop(void );
# 139 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask(void );
# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(/*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type t0, /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$size_type /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xOSTimerM$OST4_11Irq$fired(void );
#line 75
static void HplPXA27xOSTimerM$OST0Irq$fired(void );
#line 75
static void HplPXA27xOSTimerM$OST1Irq$fired(void );
#line 75
static void HplPXA27xOSTimerM$OST2Irq$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t HplPXA27xOSTimerM$Init$init(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xOSTimerM$OST3Irq$fired(void );
# 71 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOSMR(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138, 
# 71 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
uint32_t val);
#line 64
static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSCR(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138);
# 139 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$default$fired(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138);
# 78 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSMR(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138);
# 85 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOMCR(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138, 
# 85 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
uint32_t val);
#line 112
static bool HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138);
# 103 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static bool HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138);
# 57 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOSCR(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138, 
# 57 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
uint32_t val);
#line 119
static void HplPXA27xOSTimerM$PXA27xOST$setOIERbit(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138, 
# 119 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
bool flag);
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerWatchdog.nc"
static void HplPXA27xOSTimerM$PXA27xWD$enableWatchdog(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xInterruptM$PXA27xIrq$default$fired(
# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x4044fc48);
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xInterruptM$PXA27xIrq$enable(
# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x4044fc48);
# 60 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static error_t HplPXA27xInterruptM$PXA27xIrq$allocate(
# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x4044fc48);
# 139 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow(void );
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR(void );
#line 45
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(uint32_t val);



static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR(void );

static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(uint32_t val);
#line 43
static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR(void );
#line 42
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(uint32_t val);
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq0$fired(void );
#line 75
static void HplPXA27xGPIOM$GPIOIrq$fired(void );
# 134 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50, 
# 134 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
uint8_t func);
#line 101
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50, 
# 101 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 83
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50, 
# 83 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 52
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50, 
# 52 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool dir);
#line 72
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50);
# 124 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static bool HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50);
# 66 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t HplPXA27xGPIOM$Init$init(void );
# 132 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIO.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIO$default$fired(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq1$fired(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x40333758);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$default$runTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x40333758);
# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
static void SchedulerBasicP$Scheduler$init(void );
#line 61
static void SchedulerBasicP$Scheduler$taskLoop(void );
#line 54
static bool SchedulerBasicP$Scheduler$runNextTask(void );
# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
static void McuSleepC$McuSleep$sleep(void );
# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
static void VMC$Boot$booted(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void VMC$execute_instructionTask$runTask(void );
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static void VMC$Read$readDone(error_t result, VMC$Read$val_t val);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static void VMC$SerialAMControl$startDone(error_t error);
#line 117
static void VMC$SerialAMControl$stopDone(error_t error);
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



VMC$SerialReceive$receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 99 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static void VMC$SerialAMSend$sendDone(
#line 92
message_t * msg, 






error_t error);
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t VMC$Timer$default$getNow(
# 7 "VMC.nc"
uint8_t arg_0x405b2258);
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void VMC$Timer$fired(
# 7 "VMC.nc"
uint8_t arg_0x405b2258);
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void VMC$Timer$default$startPeriodic(
# 7 "VMC.nc"
uint8_t arg_0x405b2258, 
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
uint32_t dt);
#line 67
static void VMC$Timer$default$stop(
# 7 "VMC.nc"
uint8_t arg_0x405b2258);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t LedsP$Init$init(void );
# 50 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
static void LedsP$Leds$led0Off(void );










static void LedsP$Leds$led1On(void );




static void LedsP$Leds$led1Off(void );
#line 83
static void LedsP$Leds$led2Off(void );
#line 45
static void LedsP$Leds$led0On(void );
#line 78
static void LedsP$Leds$led2On(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40678c20);
# 50 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40678c20);
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40678c20);
# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static void HalPXA27xGeneralIOM$GpioInterrupt$default$fired(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40677670);
# 43 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40677670);
# 42 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static error_t HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40677670);
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40675010);
# 33 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$makeInput(
# 44 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40668e88);
# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$makeOutput(
# 44 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40668e88);
# 29 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$set(
# 44 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40668e88);
# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void HalPXA27xGeneralIOM$GeneralIO$clr(
# 44 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40668e88);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static void DemoSensorP$SensorControl$startDone(error_t error);
#line 117
static void DemoSensorP$SensorControl$stopDone(error_t error);
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static void DemoSensorP$Light$readDone(error_t result, DemoSensorP$Light$val_t val);
# 41 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
static void DemoSensorP$HalTsl2561Advanced$setIntegrationDone(error_t error);







static void DemoSensorP$HalTsl2561Advanced$enableAlertDone(error_t error);
static void DemoSensorP$HalTsl2561Advanced$alertThreshold(void );
#line 39
static void DemoSensorP$HalTsl2561Advanced$setGainDone(error_t error);



static void DemoSensorP$HalTsl2561Advanced$setPersistenceDone(error_t error);

static void DemoSensorP$HalTsl2561Advanced$setTLowDone(error_t error);

static void DemoSensorP$HalTsl2561Advanced$setTHighDone(error_t error);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static error_t DemoSensorP$Read$read(void );







static void DemoSensorP$IRLight$readDone(error_t result, DemoSensorP$IRLight$val_t val);
# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHHIGHDone(error_t error);
#line 47
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh1Done(error_t error, uint16_t val);
#line 67
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$alertThreshold(void );
#line 62
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setINTERRUPTDone(error_t error);
#line 44
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh0Done(error_t error, uint16_t val);
#line 56
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHLOWDone(error_t error);
#line 50
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setCONTROLDone(error_t error);


static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTIMINGDone(error_t error);
#line 65
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$getIDDone(error_t error, uint8_t idval);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$read(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$granted(void );
#line 92
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$granted(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$runTask(void );
# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static void HalTsl2561ControlP$HplTSL256x$setTHRESHHIGHDone(error_t error);
#line 47
static void HalTsl2561ControlP$HplTSL256x$measureCh1Done(error_t error, uint16_t val);
#line 67
static void HalTsl2561ControlP$HplTSL256x$alertThreshold(void );
#line 62
static void HalTsl2561ControlP$HplTSL256x$setINTERRUPTDone(error_t error);
#line 44
static void HalTsl2561ControlP$HplTSL256x$measureCh0Done(error_t error, uint16_t val);
#line 56
static void HalTsl2561ControlP$HplTSL256x$setTHRESHLOWDone(error_t error);
#line 50
static void HalTsl2561ControlP$HplTSL256x$setCONTROLDone(error_t error);


static void HalTsl2561ControlP$HplTSL256x$setTIMINGDone(error_t error);
#line 65
static void HalTsl2561ControlP$HplTSL256x$getIDDone(error_t error, uint8_t idval);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void HalTsl2561ControlP$complete_Alert$runTask(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void HalTsl2561ControlP$Resource$granted(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void HalTsl2561ControlP$complete_Task$runTask(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init(void );
# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
static error_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id);
#line 43
static bool /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void );








static bool /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id);







static resource_client_id_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void );
# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(
# 52 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072b5b0);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072a200);
# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072a200);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072cab0);
# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072cab0);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072cab0);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void );
# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$start(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone$runTask(void );
#line 64
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone$runTask(void );
# 43 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh0(void );


static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh1(void );
#line 61
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setINTERRUPT(uint8_t val);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$Init$init(void );
# 101 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$writeDone(error_t error, uint16_t addr, uint8_t length, 
#line 98
uint8_t * data);
#line 91
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$readDone(error_t error, uint16_t addr, uint8_t length, 
#line 88
uint8_t * data);
# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptAlert$fired(void );
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$interruptI2C(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$Init$init(void );
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$interruptGPIOPin(void );
#line 150
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$interruptGPIOPin(void );
# 65 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$read(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 61
uint8_t * data);
#line 81
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$write(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 77
uint8_t * data);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$runTask(void );
#line 64
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$runTask(void );
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setICR(uint32_t val);



static uint32_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getISR(void );

static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISAR(uint32_t val);
#line 48
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISR(uint32_t val);
#line 43
static uint32_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getIDBR(void );
#line 42
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setIDBR(uint32_t val);
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$Init$init(void );
# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static void Tsl2561InternalP$ToHPLC$setTHRESHHIGHDone(error_t error);
#line 47
static void Tsl2561InternalP$ToHPLC$measureCh1Done(error_t error, uint16_t val);
#line 67
static void Tsl2561InternalP$ToHPLC$alertThreshold(void );
#line 62
static void Tsl2561InternalP$ToHPLC$setINTERRUPTDone(error_t error);
#line 44
static void Tsl2561InternalP$ToHPLC$measureCh0Done(error_t error, uint16_t val);
#line 56
static void Tsl2561InternalP$ToHPLC$setTHRESHLOWDone(error_t error);
#line 50
static void Tsl2561InternalP$ToHPLC$setCONTROLDone(error_t error);


static void Tsl2561InternalP$ToHPLC$setTIMINGDone(error_t error);
#line 65
static void Tsl2561InternalP$ToHPLC$getIDDone(error_t error, uint8_t idval);
#line 43
static error_t Tsl2561InternalP$HplTSL256x$measureCh0(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730);
# 46 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static error_t Tsl2561InternalP$HplTSL256x$measureCh1(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730);
# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static void Tsl2561InternalP$HplTSL256x$default$setTHRESHHIGHDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 47
static void Tsl2561InternalP$HplTSL256x$default$measureCh1Done(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 47 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error, uint16_t val);
#line 67
static void Tsl2561InternalP$HplTSL256x$default$alertThreshold(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730);
# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static void Tsl2561InternalP$HplTSL256x$default$setINTERRUPTDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 44
static void Tsl2561InternalP$HplTSL256x$default$measureCh0Done(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 44 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error, uint16_t val);
#line 56
static void Tsl2561InternalP$HplTSL256x$default$setTHRESHLOWDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 56 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 50
static void Tsl2561InternalP$HplTSL256x$default$setCONTROLDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 50 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);


static void Tsl2561InternalP$HplTSL256x$default$setTIMINGDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 53 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 65
static void Tsl2561InternalP$HplTSL256x$default$getIDDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 65 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error, uint8_t idval);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t Tsl2561InternalP$Init$init(void );
# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static void Tsl2561InternalP$InterruptAlert$fired(void );
# 89 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(
#line 85
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(
#line 60
message_t * msg, 
void * payload, 





uint8_t len);
# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(
# 36 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40826200, 
# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Packet.nc"
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$payloadLength(
#line 63
message_t * msg);
#line 115
static 
#line 112
void * 


/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$getPayload(
#line 110
message_t * msg, 




uint8_t len);
#line 95
static uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$maxPayloadLength(void );
#line 83
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(
# 37 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40826bc0, 
# 60 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 67 "/opt/tinyos-2.1.0/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$destination(
#line 63
message_t * amsg);
#line 92
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 136
static am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(
#line 132
message_t * amsg);
#line 151
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setType(
#line 147
message_t * amsg, 



am_id_t t);
# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static error_t SerialP$SplitControl$start(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SerialP$stopDoneTask$runTask(void );
#line 64
static void SerialP$RunTx$runTask(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t SerialP$Init$init(void );
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFlush.nc"
static void SerialP$SerialFlush$flushDone(void );
#line 38
static void SerialP$SerialFlush$default$flush(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SerialP$startDoneTask$runTask(void );
# 83 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
static void SerialP$SerialFrameComm$dataReceived(uint8_t data);





static void SerialP$SerialFrameComm$putDone(void );
#line 74
static void SerialP$SerialFrameComm$delimiterReceived(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SerialP$defaultSerialFlushTask$runTask(void );
# 60 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
static error_t SerialP$SendBytePacket$completeSend(void );
#line 51
static error_t SerialP$SendBytePacket$startSend(uint8_t first_byte);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask(void );
# 64 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(
# 40 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5030, 
# 56 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(
# 40 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5030, 
# 85 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask(void );
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(
# 39 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e89d0, 
# 60 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5b30, 
# 31 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5b30);
# 23 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5b30, 
# 23 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 70 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte(void );









static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error_t error);
# 51 "/opt/tinyos-2.1.0/tos/lib/serial/ReceiveBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket(void );






static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(uint8_t data);










static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(error_t result);
# 79 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
static void HdlcTranslateC$UartStream$receivedByte(uint8_t byte);
#line 99
static void HdlcTranslateC$UartStream$receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void HdlcTranslateC$UartStream$sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 45 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
static error_t HdlcTranslateC$SerialFrameComm$putDelimiter(void );
#line 68
static void HdlcTranslateC$SerialFrameComm$resetReceive(void );
#line 54
static error_t HdlcTranslateC$SerialFrameComm$putData(uint8_t data);
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialPacket.nc"
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$send(uint8_t *buf, uint16_t len);
#line 75
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$receive(uint8_t *buf, uint16_t len, uint16_t timeout);
#line 89
static uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$receiveDone(uint8_t *buf, uint16_t len, uart_status_t status);
#line 62
static uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$sendDone(uint8_t *buf, uint16_t len, uart_status_t status);
# 91 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$interruptDMA(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$Init$init(void );
# 81 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$interruptUART(void );
# 48 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$send(
#line 44
uint8_t * buf, 



uint16_t len);
# 91 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$interruptDMA(void );
# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialCntl.nc"
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialCntl$configPort(uint32_t baudrate, 
uint8_t databits, 
uart_parity_t parity, 
uint8_t stopbits, 
bool flow_cntl);
# 74 "/opt/tinyos-2.1.0/tos/interfaces/StdControl.nc"
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$start(void );









static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$stop(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$Init$init(void );
# 44 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLL(uint32_t val);
#line 60
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setMCR(uint32_t val);
#line 53
static uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIIR(void );
#line 47
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLH(uint32_t val);
#line 42
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setTHR(uint32_t val);
#line 63
static uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getLSR(void );
#line 55
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setFCR(uint32_t val);
#line 51
static uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIER(void );
#line 41
static uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getRBR(void );
#line 57
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setLCR(uint32_t val);
#line 50
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setIER(uint32_t val);
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$fired(void );
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void IM2InitSerialP$RXD$interruptGPIOPin(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t IM2InitSerialP$Init$init(void );
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void IM2InitSerialP$TXD$interruptGPIOPin(void );
# 63 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfo.nc"
static uint8_t /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getMapIndex(void );
#line 54
static uint32_t /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getAddr(void );








static uint8_t /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getMapIndex(void );
#line 54
static uint32_t /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getAddr(void );
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xDMAM$DMAIrq$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t HplPXA27xDMAM$Init$init(void );
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMACntl.nc"
static void HplPXA27xDMAM$HplPXA27xDMACntl$setDRCMR(uint8_t peripheral, uint8_t chnl);
#line 52
static uint32_t HplPXA27xDMAM$HplPXA27xDMACntl$getDINT(void );
# 89 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDTADR(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80, 
# 89 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
uint32_t val);
#line 87
static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDSADR(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80, 
# 87 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
uint32_t val);
#line 78
static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDALGNbit(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80, 
# 78 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
bool flag);


static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80, 
# 81 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
uint32_t val);
#line 77
static error_t HplPXA27xDMAM$HplPXA27xDMAChnl$setMap(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80, 
# 77 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
uint8_t dev);
#line 91
static void HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80);
# 83 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80, 
# 83 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
uint32_t val);
# 31 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t SerialPacketInfoActiveMessageP$Info$upperLength(message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t SerialPacketInfoActiveMessageP$Info$offset(void );







static uint8_t SerialPacketInfoActiveMessageP$Info$dataLinkLength(message_t *msg, uint8_t upperLen);
# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static error_t /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(
#line 85
message_t * msg, 



error_t error);
# 99 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(
# 40 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40a91ef8, 
# 92 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(
# 38 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40a91510, 
# 56 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
message_t * msg, 







uint8_t len);
#line 89
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(
# 38 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40a91510, 
# 85 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask(void );
#line 64
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PlatformP$InitL2$init(void );
#line 51
static error_t PlatformP$InitL0$init(void );
#line 51
static error_t PlatformP$InitL3$init(void );
# 71 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void PlatformP$OST0M3$setOSMR(uint32_t val);
#line 64
static uint32_t PlatformP$OST0M3$getOSCR(void );
#line 112
static bool PlatformP$OST0M3$clearOSSRbit(void );






static void PlatformP$OST0M3$setOIERbit(bool flag);
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerWatchdog.nc"
static void PlatformP$PXA27xWD$enableWatchdog(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t PlatformP$InitL1$init(void );
# 52 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/PlatformP.nc"
static inline error_t PlatformP$Init$init(void );
#line 117
static inline void PlatformP$PlatformReset$reset(void );






static inline void PlatformP$OST0M3$fired(void );
# 32 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/PlatformReset.nc"
static void PMICM$PlatformReset$reset(void );
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static uint32_t PMICM$PI2C$getICR(void );
#line 45
static void PMICM$PI2C$setICR(uint32_t val);



static uint32_t PMICM$PI2C$getISR(void );

static void PMICM$PI2C$setISAR(uint32_t val);
#line 43
static uint32_t PMICM$PI2C$getIDBR(void );
#line 42
static void PMICM$PI2C$setIDBR(uint32_t val);
# 134 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void PMICM$PMICGPIO$setGAFRpin(uint8_t func);
#line 101
static void PMICM$PMICGPIO$setGFERbit(bool flag);
#line 52
static void PMICM$PMICGPIO$setGPDRbit(bool dir);
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void PMICM$chargeMonitorTimer$startPeriodic(uint32_t dt);
#line 67
static void PMICM$chargeMonitorTimer$stop(void );
# 101 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
bool PMICM$gotReset;


static error_t PMICM$readPMIC(uint8_t address, uint8_t *value, uint8_t numBytes);
#line 151
static error_t PMICM$writePMIC(uint8_t address, uint8_t value);
#line 171
static inline void PMICM$startLDOs(void );
#line 219
static inline error_t PMICM$Init$init(void );
#line 256
static inline void PMICM$PI2C$interruptI2C(void );
#line 271
static inline void PMICM$PMICGPIO$interruptGPIOPin(void );
#line 300
static inline error_t PMICM$PMIC$setCoreVoltage(uint8_t trimValue);
#line 334
static error_t PMICM$getPMICADCVal(uint8_t channel, uint8_t *val);
#line 355
static inline error_t PMICM$PMIC$getBatteryVoltage(uint8_t *val);
#line 372
static inline error_t PMICM$PMIC$enableManualCharging(bool enable);
#line 407
static inline void PMICM$chargeMonitorTimer$fired(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask(void );
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow(void );
#line 118
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt);
#line 67
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$stop(void );




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$fired(
# 37 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
uint8_t arg_0x403d9ce0);
#line 60
enum /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_unnamed4274 {
#line 60
  VirtualizeTimerC$0$updateFromTimer = 0U
};
#line 60
typedef int /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_sillytask_updateFromTimer[/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer];
#line 42
enum /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_unnamed4275 {

  VirtualizeTimerC$0$NUM_TIMERS = 4U, 
  VirtualizeTimerC$0$END_OF_LIST = 255
};








#line 48
typedef struct /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$__nesc_unnamed4276 {

  uint32_t t0;
  uint32_t dt;
  bool isoneshot : 1;
  bool isrunning : 1;
  bool _reserved : 6;
} /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t;

/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$NUM_TIMERS];




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(uint32_t now);
#line 89
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask(void );
#line 128
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired(void );




static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot);









static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt);









static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(uint8_t num);
#line 178
static inline uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$getNow(uint8_t num);
#line 193
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$postTask(void );
# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getNow(void );
#line 92
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type dt);
#line 105
static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getAlarm(void );
#line 62
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$stop(void );
# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$fired(void );
# 63 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
enum /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$__nesc_unnamed4277 {
#line 63
  AlarmToTimerC$0$fired = 1U
};
#line 63
typedef int /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$__nesc_sillytask_fired[/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired];
#line 44
uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_dt;
bool /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_oneshot;

static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot);
#line 60
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop(void );


static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask(void );






static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired(void );
#line 82
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt);


static inline uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow(void );
# 71 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSMR(uint32_t val);
#line 64
static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR(void );
#line 78
static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSMR(void );






static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOMCR(uint32_t val);
#line 112
static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$clearOSSRbit(void );
#line 103
static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSSRbit(void );
#line 57
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSCR(uint32_t val);
#line 119
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(bool flag);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTInit$init(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$postTask(void );
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired(void );
# 56 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
enum /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$__nesc_unnamed4278 {
#line 56
  HalPXA27xAlarmM$0$lateAlarm = 2U
};
#line 56
typedef int /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$__nesc_sillytask_lateAlarm[/*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm];
#line 53
bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning;
uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT;

static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask(void );






static inline error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init(void );
#line 117
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop(void );
#line 132
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(uint32_t t0, uint32_t dt);
#line 152
static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow(void );



static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm(void );



static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xOSTimerM$OST4_11Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST4_11Irq$allocate(void );




static void HplPXA27xOSTimerM$OST0Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST0Irq$allocate(void );




static void HplPXA27xOSTimerM$OST1Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST1Irq$allocate(void );




static void HplPXA27xOSTimerM$OST2Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST2Irq$allocate(void );




static void HplPXA27xOSTimerM$OST3Irq$enable(void );
#line 60
static error_t HplPXA27xOSTimerM$OST3Irq$allocate(void );
# 139 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$fired(
# 39 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
uint8_t arg_0x4044d138);
#line 53
bool HplPXA27xOSTimerM$gfInitialized = FALSE;

static inline void HplPXA27xOSTimerM$DispatchOSTInterrupt(uint8_t id);





static error_t HplPXA27xOSTimerM$Init$init(void );
#line 87
static void HplPXA27xOSTimerM$PXA27xOST$setOSCR(uint8_t chnl_id, uint32_t val);









static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSCR(uint8_t chnl_id);










static void HplPXA27xOSTimerM$PXA27xOST$setOSMR(uint8_t chnl_id, uint32_t val);





static inline uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSMR(uint8_t chnl_id);






static void HplPXA27xOSTimerM$PXA27xOST$setOMCR(uint8_t chnl_id, uint32_t val);
#line 138
static inline bool HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(uint8_t chnl_id);










static bool HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(uint8_t chnl_id);
#line 163
static void HplPXA27xOSTimerM$PXA27xOST$setOIERbit(uint8_t chnl_id, bool flag);
#line 186
static inline void HplPXA27xOSTimerM$PXA27xWD$enableWatchdog(void );









static inline void HplPXA27xOSTimerM$OST0Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST1Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST2Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST3Irq$fired(void );




static inline void HplPXA27xOSTimerM$OST4_11Irq$fired(void );
#line 233
static void HplPXA27xOSTimerM$PXA27xOST$default$fired(uint8_t chnl_id);
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xInterruptM$PXA27xIrq$fired(
# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
uint8_t arg_0x4044fc48);







static inline uint32_t HplPXA27xInterruptM$getICHP(void );








void hplarmv_irq(void ) __attribute((interrupt("IRQ")))   ;
#line 85
void hplarmv_fiq(void ) __attribute((interrupt("FIQ")))   ;



static uint8_t HplPXA27xInterruptM$usedPriorities = 0;




static error_t HplPXA27xInterruptM$allocate(uint8_t id, bool level, uint8_t priority);
#line 162
static void HplPXA27xInterruptM$enable(uint8_t id);
#line 188
static inline error_t HplPXA27xInterruptM$PXA27xIrq$allocate(uint8_t id);




static inline void HplPXA27xInterruptM$PXA27xIrq$enable(uint8_t id);
#line 227
static inline void HplPXA27xInterruptM$PXA27xIrq$default$fired(uint8_t id);
# 71 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSMR(uint32_t val);
#line 85
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOMCR(uint32_t val);
#line 112
static bool /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit(void );
#line 57
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSCR(uint32_t val);
#line 119
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOIERbit(bool flag);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTInit$init(void );
# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$overflow(void );
# 56 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init(void );
#line 91
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired(void );
#line 104
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow(void );
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$interruptI2C(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$enable(void );
#line 60
static error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$allocate(void );
# 52 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
bool /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$m_fInit = FALSE;

static inline error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init(void );
#line 90
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(uint32_t val);








static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR(void );







static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(uint32_t val);








static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR(void );
#line 132
static inline uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR(void );







static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(uint32_t val);
#line 157
static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq0$enable(void );
#line 60
static error_t HplPXA27xGPIOM$GPIOIrq0$allocate(void );




static void HplPXA27xGPIOM$GPIOIrq$enable(void );
#line 60
static error_t HplPXA27xGPIOM$GPIOIrq$allocate(void );
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
uint8_t arg_0x4050cc50);
# 132 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIO.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIO$fired(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xGPIOM$GPIOIrq1$enable(void );
#line 60
static error_t HplPXA27xGPIOM$GPIOIrq1$allocate(void );
# 58 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
bool HplPXA27xGPIOM$gfInitialized = FALSE;

static inline error_t HplPXA27xGPIOM$Init$init(void );
#line 85
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(uint8_t pin, bool dir);
#line 101
static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(uint8_t pin);





static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(uint8_t pin);





static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(uint8_t pin, bool flag);
#line 129
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(uint8_t pin, bool flag);
#line 150
static inline bool HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(uint8_t pin);







static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(uint8_t pin, uint8_t func);
#line 259
static inline void HplPXA27xGPIOM$HplPXA27xGPIO$default$fired(void );



static inline void HplPXA27xGPIOM$GPIOIrq$fired(void );
#line 307
static inline void HplPXA27xGPIOM$GPIOIrq0$fired(void );




static inline void HplPXA27xGPIOM$GPIOIrq1$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t RealMainP$SoftwareInit$init(void );
# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
static void RealMainP$Boot$booted(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t RealMainP$PlatformInit$init(void );
# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
static void RealMainP$Scheduler$init(void );
#line 61
static void RealMainP$Scheduler$taskLoop(void );
#line 54
static bool RealMainP$Scheduler$runNextTask(void );
# 52 "/opt/tinyos-2.1.0/tos/system/RealMainP.nc"
int main(void )   ;
# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(
# 45 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
uint8_t arg_0x40333758);
# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
static void SchedulerBasicP$McuSleep$sleep(void );
# 50 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
enum SchedulerBasicP$__nesc_unnamed4279 {

  SchedulerBasicP$NUM_TASKS = 20U, 
  SchedulerBasicP$NO_TASK = 255
};

uint8_t SchedulerBasicP$m_head;
uint8_t SchedulerBasicP$m_tail;
uint8_t SchedulerBasicP$m_next[SchedulerBasicP$NUM_TASKS];








static __inline uint8_t SchedulerBasicP$popTask(void );
#line 86
static inline bool SchedulerBasicP$isWaiting(uint8_t id);




static inline bool SchedulerBasicP$pushTask(uint8_t id);
#line 113
static inline void SchedulerBasicP$Scheduler$init(void );









static bool SchedulerBasicP$Scheduler$runNextTask(void );
#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void );
#line 159
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id);




static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id);
# 53 "/opt/tinyos-2.1.0/tos/chips/pxa27x/McuSleepC.nc"
static inline void McuSleepC$McuSleep$sleep(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t VMC$execute_instructionTask$postTask(void );
# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static error_t VMC$Read$read(void );
# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static error_t VMC$SerialAMControl$start(void );
# 115 "/opt/tinyos-2.1.0/tos/interfaces/Packet.nc"
static 
#line 112
void * 


VMC$SerialPacket$getPayload(
#line 110
message_t * msg, 




uint8_t len);
# 50 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
static void VMC$Leds$led0Off(void );










static void VMC$Leds$led1On(void );




static void VMC$Leds$led1Off(void );
#line 83
static void VMC$Leds$led2Off(void );
#line 45
static void VMC$Leds$led0On(void );
#line 78
static void VMC$Leds$led2On(void );
# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static error_t VMC$SerialAMSend$send(am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static uint32_t VMC$Timer$getNow(
# 7 "VMC.nc"
uint8_t arg_0x405b2258);
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
static void VMC$Timer$startPeriodic(
# 7 "VMC.nc"
uint8_t arg_0x405b2258, 
# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
uint32_t dt);
#line 67
static void VMC$Timer$stop(
# 7 "VMC.nc"
uint8_t arg_0x405b2258);
#line 22
enum VMC$__nesc_unnamed4280 {
#line 22
  VMC$execute_instructionTask = 3U
};
#line 22
typedef int VMC$__nesc_sillytask_execute_instructionTask[VMC$execute_instructionTask];




bool VMC$serial_busy = FALSE;
message_t VMC$serial_p;




app VMC$apps[3];
uint8_t VMC$active_app = 3;
uint32_t VMC$cache_time = 0;
uint8_t VMC$cache_value = 100;



static inline void VMC$system_init(void );
#line 55
static void VMC$next_app(void );
#line 73
static inline void VMC$read_sensor(int8_t reg);
#line 91
static inline void VMC$unload_binary(uint8_t appid);
#line 150
static inline void VMC$load_binary(uint8_t *bin, uint8_t appid);
#line 218
static inline void VMC$execute_instructionTask$runTask(void );
#line 510
static inline void VMC$Boot$booted(void );








static void VMC$Timer$fired(uint8_t id);
#line 551
static void VMC$Read$readDone(error_t result, uint16_t val);
#line 582
static inline void VMC$Timer$default$startPeriodic(uint8_t id, uint32_t milli);



static inline void VMC$Timer$default$stop(uint8_t id);



static inline uint32_t VMC$Timer$default$getNow(uint8_t id);
#line 607
static inline void VMC$SerialAMControl$startDone(error_t err);





static inline void VMC$SerialAMControl$stopDone(error_t err);


static inline void VMC$SerialAMSend$sendDone(message_t *msg, error_t err);






static inline message_t *VMC$SerialReceive$receive(message_t *msg, void *payload, uint8_t len);
# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void LedsP$Led0$makeOutput(void );
#line 29
static void LedsP$Led0$set(void );
static void LedsP$Led0$clr(void );




static void LedsP$Led1$makeOutput(void );
#line 29
static void LedsP$Led1$set(void );
static void LedsP$Led1$clr(void );




static void LedsP$Led2$makeOutput(void );
#line 29
static void LedsP$Led2$set(void );
static void LedsP$Led2$clr(void );
# 45 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void );
#line 63
static inline void LedsP$Leds$led0On(void );




static inline void LedsP$Leds$led0Off(void );









static inline void LedsP$Leds$led1On(void );




static inline void LedsP$Leds$led1Off(void );









static inline void LedsP$Leds$led2On(void );




static inline void LedsP$Leds$led2Off(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
static void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$fired(
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40678c20);
# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static void HalPXA27xGeneralIOM$GpioInterrupt$fired(
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40677670);
# 101 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40675010, 
# 101 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 83
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40675010, 
# 83 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool flag);
#line 52
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40675010, 
# 52 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
bool dir);
#line 72
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPCRbit(
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40675010);
# 124 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static bool HalPXA27xGeneralIOM$HplPXA27xGPIOPin$clearGEDRbit(
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40675010);
# 66 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPSRbit(
# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
uint8_t arg_0x40675010);




static void HalPXA27xGeneralIOM$GeneralIO$set(uint8_t pin);





static void HalPXA27xGeneralIOM$GeneralIO$clr(uint8_t pin);
#line 83
static inline void HalPXA27xGeneralIOM$GeneralIO$makeInput(uint8_t pin);










static inline void HalPXA27xGeneralIOM$GeneralIO$makeOutput(uint8_t pin);










static inline error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(uint8_t pin);







static inline error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(uint8_t pin);
#line 138
static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(uint8_t pin);



static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(uint8_t pin);







static inline void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(uint8_t pin);







static inline void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(uint8_t pin);



static inline void HalPXA27xGeneralIOM$GpioInterrupt$default$fired(uint8_t pin);
# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static error_t DemoSensorP$SensorControl$start(void );
# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static error_t DemoSensorP$Light$read(void );







static void DemoSensorP$Read$readDone(error_t result, DemoSensorP$Read$val_t val);
# 45 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
int DemoSensorP$status = 1;
static inline error_t DemoSensorP$Read$read(void );









static inline void DemoSensorP$SensorControl$startDone(error_t t);


static inline void DemoSensorP$SensorControl$stopDone(error_t t);

static inline void DemoSensorP$Light$readDone(error_t err, uint16_t value);




static inline void DemoSensorP$IRLight$readDone(error_t err, uint16_t value);



static inline void DemoSensorP$HalTsl2561Advanced$setGainDone(error_t err);


static inline void DemoSensorP$HalTsl2561Advanced$setIntegrationDone(error_t err);


static inline void DemoSensorP$HalTsl2561Advanced$setPersistenceDone(error_t err);


static inline void DemoSensorP$HalTsl2561Advanced$setTLowDone(error_t err);


static inline void DemoSensorP$HalTsl2561Advanced$setTHighDone(error_t err);


static inline void DemoSensorP$HalTsl2561Advanced$enableAlertDone(error_t err);


static inline void DemoSensorP$HalTsl2561Advanced$alertThreshold(void );
# 43 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh0(void );


static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh1(void );
# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRPhoto$readDone(error_t result, /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRPhoto$val_t val);
#line 63
static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$readDone(error_t result, /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$val_t val);
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$release(void );
#line 78
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$request(void );
#line 110
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$release(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$postTask(void );
# 60 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
enum /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$__nesc_unnamed4281 {
#line 60
  HalTsl2561ReaderP$0$signalDone_task = 4U
};
#line 60
typedef int /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$__nesc_sillytask_signalDone_task[/*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task];
#line 50
enum /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$__nesc_unnamed4282 {
  HalTsl2561ReaderP$0$S_OFF = 0, 
  HalTsl2561ReaderP$0$S_READY, 
  HalTsl2561ReaderP$0$S_READ_BB, 
  HalTsl2561ReaderP$0$S_READ_IR
};
uint8_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READY;
error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_error;
uint16_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_val;

static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$runTask(void );
#line 78
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$read(void );
#line 94
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$granted(void );








static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$granted(void );








static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh0Done(error_t error, uint16_t val);






static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh1Done(error_t error, uint16_t val);






static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setCONTROLDone(error_t error);
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTIMINGDone(error_t error);
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHLOWDone(error_t error);
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHHIGHDone(error_t error);
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setINTERRUPTDone(error_t error);
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$getIDDone(error_t error, uint8_t idval);
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$alertThreshold(void );
# 41 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
static void HalTsl2561ControlP$HalTsl2561Advanced$setIntegrationDone(error_t error);







static void HalTsl2561ControlP$HalTsl2561Advanced$enableAlertDone(error_t error);
static void HalTsl2561ControlP$HalTsl2561Advanced$alertThreshold(void );
#line 39
static void HalTsl2561ControlP$HalTsl2561Advanced$setGainDone(error_t error);



static void HalTsl2561ControlP$HalTsl2561Advanced$setPersistenceDone(error_t error);

static void HalTsl2561ControlP$HalTsl2561Advanced$setTLowDone(error_t error);

static void HalTsl2561ControlP$HalTsl2561Advanced$setTHighDone(error_t error);
# 61 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static error_t HalTsl2561ControlP$HplTSL256x$setINTERRUPT(uint8_t val);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t HalTsl2561ControlP$complete_Alert$postTask(void );
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static error_t HalTsl2561ControlP$Resource$release(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t HalTsl2561ControlP$complete_Task$postTask(void );
# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
enum HalTsl2561ControlP$__nesc_unnamed4283 {
#line 62
  HalTsl2561ControlP$complete_Alert = 5U
};
#line 62
typedef int HalTsl2561ControlP$__nesc_sillytask_complete_Alert[HalTsl2561ControlP$complete_Alert];



enum HalTsl2561ControlP$__nesc_unnamed4284 {
#line 66
  HalTsl2561ControlP$complete_Task = 6U
};
#line 66
typedef int HalTsl2561ControlP$__nesc_sillytask_complete_Task[HalTsl2561ControlP$complete_Task];
#line 47
enum HalTsl2561ControlP$__nesc_unnamed4285 {
  HalTsl2561ControlP$S_IDLE = 0, 
  HalTsl2561ControlP$S_GAIN, 
  HalTsl2561ControlP$S_INTEG, 
  HalTsl2561ControlP$S_PERSIST, 
  HalTsl2561ControlP$S_TLOW, 
  HalTsl2561ControlP$S_THIGH, 
  HalTsl2561ControlP$S_ENALERT
};
uint8_t HalTsl2561ControlP$state = HalTsl2561ControlP$S_IDLE;
error_t HalTsl2561ControlP$clientResult;


uint8_t HalTsl2561ControlP$iControlRegisterShadow = 0x0;

static inline void HalTsl2561ControlP$complete_Alert$runTask(void );



static inline void HalTsl2561ControlP$complete_Task$runTask(void );
#line 197
static inline void HalTsl2561ControlP$Resource$granted(void );







static inline void HalTsl2561ControlP$HplTSL256x$setTIMINGDone(error_t error);



static inline void HalTsl2561ControlP$HplTSL256x$setINTERRUPTDone(error_t error);



static inline void HalTsl2561ControlP$HplTSL256x$setTHRESHLOWDone(error_t error);



static inline void HalTsl2561ControlP$HplTSL256x$setTHRESHHIGHDone(error_t error);



static inline void HalTsl2561ControlP$HplTSL256x$alertThreshold(void );


static inline void HalTsl2561ControlP$HplTSL256x$getIDDone(error_t error, uint8_t idval);


static inline void HalTsl2561ControlP$HplTSL256x$setCONTROLDone(error_t error);
static inline void HalTsl2561ControlP$HplTSL256x$measureCh0Done(error_t error, uint16_t val);
static inline void HalTsl2561ControlP$HplTSL256x$measureCh1Done(error_t error, uint16_t val);
# 39 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
enum /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$__nesc_unnamed4286 {
#line 39
  FcfsResourceQueueC$0$NO_ENTRY = 0xFF
};
uint8_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[3U];
uint8_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
uint8_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qTail = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;

static inline error_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init(void );




static inline bool /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void );



static inline bool /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id);



static inline resource_client_id_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void );
#line 72
static inline error_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id);
# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(
# 52 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072b5b0);
# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072a200);
# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(
# 56 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072a200);
# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t id);
#line 43
static bool /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void );
#line 60
static resource_client_id_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void );
# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(
# 51 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
uint8_t arg_0x4072cab0);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void );
# 69 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
enum /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4287 {
#line 69
  SimpleArbiterP$0$grantedTask = 7U
};
#line 69
typedef int /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_sillytask_grantedTask[/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask];
#line 62
enum /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4288 {
#line 62
  SimpleArbiterP$0$RES_IDLE = 0, SimpleArbiterP$0$RES_GRANTING = 1, SimpleArbiterP$0$RES_BUSY = 2
};
#line 63
enum /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$__nesc_unnamed4289 {
#line 63
  SimpleArbiterP$0$NO_RES = 0xFF
};
uint8_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE;
uint8_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$NO_RES;
uint8_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId;



static inline error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id);
#line 97
static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id);
#line 154
static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void );









static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id);

static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id);



static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id);

static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$startDone(error_t error);
#line 117
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$stopDone(error_t error);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone$postTask(void );
#line 56
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone$postTask(void );
# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTHRESHHIGHDone(error_t error);
#line 47
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh1Done(error_t error, uint16_t val);
#line 67
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$alertThreshold(void );
#line 62
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setINTERRUPTDone(error_t error);
#line 44
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh0Done(error_t error, uint16_t val);
#line 56
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTHRESHLOWDone(error_t error);
#line 50
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setCONTROLDone(error_t error);


static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTIMINGDone(error_t error);
#line 65
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$getIDDone(error_t error, uint8_t idval);
# 65 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$read(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 61
uint8_t * data);
#line 81
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$write(i2c_flags_t flags, uint16_t addr, uint8_t length, 
#line 77
uint8_t * data);
# 43 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptAlert$enableFallingEdge(void );
# 33 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptPin$makeInput(void );
# 144 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
enum /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$__nesc_unnamed4290 {
#line 144
  HplTSL2561LogicP$0$StartDone = 8U
};
#line 144
typedef int /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$__nesc_sillytask_StartDone[/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone];




enum /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$__nesc_unnamed4291 {
#line 149
  HplTSL2561LogicP$0$StopDone = 9U
};
#line 149
typedef int /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$__nesc_sillytask_StopDone[/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone];
#line 60
enum /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$__nesc_unnamed4292 {
  HplTSL2561LogicP$0$STATE_IDLE, 
  HplTSL2561LogicP$0$STATE_STARTING, 
  HplTSL2561LogicP$0$STATE_STOPPING, 
  HplTSL2561LogicP$0$STATE_STOPPED, 
  HplTSL2561LogicP$0$STATE_READCH0, 
  HplTSL2561LogicP$0$STATE_READCH1, 
  HplTSL2561LogicP$0$STATE_SETCONTROL, 
  HplTSL2561LogicP$0$STATE_SETTIMING, 
  HplTSL2561LogicP$0$STATE_SETLOW, 
  HplTSL2561LogicP$0$STATE_SETHIGH, 
  HplTSL2561LogicP$0$STATE_SETINTERRUPT, 
  HplTSL2561LogicP$0$STATE_READID, 
  HplTSL2561LogicP$0$STATE_CLRINTERRUPTS, 
  HplTSL2561LogicP$0$STATE_ERROR
};

bool /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$interruptBit;

uint8_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer[4];
uint8_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState;
error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mSSError;

static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doWriteReg(uint8_t nextState, uint8_t reg, uint16_t val, uint8_t size);
#line 109
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doReadPrep(uint8_t nextState, uint8_t reg);
#line 144
static inline void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone$runTask(void );




static inline void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone$runTask(void );




static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$Init$init(void );







static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$start(void );
#line 185
static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh0(void );



static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh1(void );
#line 211
static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setINTERRUPT(uint8_t val);







static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$readDone(error_t i2c_error, uint16_t chipAddr, uint8_t len, uint8_t *buf);
#line 247
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$writeDone(error_t i2c_error, uint16_t chipAddr, uint8_t len, uint8_t *buf);
#line 297
static inline void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptAlert$fired(void );
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(uint32_t val);



static uint32_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$getISR(void );

static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISAR(uint32_t val);
#line 48
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISR(uint32_t val);
#line 43
static uint32_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$getIDBR(void );
#line 42
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setIDBR(uint32_t val);
# 134 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$setGAFRpin(uint8_t func);
#line 52
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$setGPDRbit(bool dir);
#line 134
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$setGAFRpin(uint8_t func);
#line 52
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$setGPDRbit(bool dir);
# 101 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$writeDone(error_t error, uint16_t addr, uint8_t length, 
#line 98
uint8_t * data);
#line 91
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$readDone(error_t error, uint16_t addr, uint8_t length, 
#line 88
uint8_t * data);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$postTask(void );
#line 56
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$postTask(void );
# 159 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
enum /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$__nesc_unnamed4293 {
#line 159
  HalPXA27xI2CMasterP$0$handleReadError = 10U
};
#line 159
typedef int /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$__nesc_sillytask_handleReadError[/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError];
#line 171
enum /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$__nesc_unnamed4294 {
#line 171
  HalPXA27xI2CMasterP$0$handleWriteError = 11U
};
#line 171
typedef int /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$__nesc_sillytask_handleWriteError[/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError];
#line 57
enum /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$__nesc_unnamed4295 {
  HalPXA27xI2CMasterP$0$I2C_STATE_IDLE, 
  HalPXA27xI2CMasterP$0$I2C_STATE_READSTART, 
  HalPXA27xI2CMasterP$0$I2C_STATE_READ, 
  HalPXA27xI2CMasterP$0$I2C_STATE_READEND, 
  HalPXA27xI2CMasterP$0$I2C_STATE_WRITE, 
  HalPXA27xI2CMasterP$0$I2C_STATE_WRITEEND, 
  HalPXA27xI2CMasterP$0$I2C_STATE_ERROR
};

uint8_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState;
uint16_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurTargetAddr;
uint8_t */*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf;
#line 69
uint8_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen;
#line 69
uint8_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex;
i2c_flags_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurFlags;
uint32_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags;

static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$readNextByte(void );
#line 93
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$writeNextByte(void );
#line 112
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$startI2CTransact(uint8_t nextState, uint16_t addr, uint8_t length, uint8_t *data, 
i2c_flags_t flags, bool bRnW);
#line 159
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$runTask(void );
#line 171
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$runTask(void );
#line 183
static inline error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$Init$init(void );
#line 199
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$read(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t *data);
#line 217
static inline error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$write(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t *data);







static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$interruptI2C(void );
#line 314
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$interruptGPIOPin(void );
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$interruptGPIOPin(void );
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$interruptI2C(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$enable(void );
#line 60
static error_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$allocate(void );
# 52 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
bool /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$m_fInit = FALSE;

static inline error_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$Init$init(void );
#line 90
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setIDBR(uint32_t val);








static uint32_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getIDBR(void );







static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setICR(uint32_t val);
#line 124
static inline void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISR(uint32_t val);







static inline uint32_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getISR(void );







static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISAR(uint32_t val);
#line 157
static inline void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$fired(void );
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t Tsl2561InternalP$SubInit$init(void );
# 43 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static error_t Tsl2561InternalP$ToHPLC$measureCh0(void );


static error_t Tsl2561InternalP$ToHPLC$measureCh1(void );
#line 59
static void Tsl2561InternalP$HplTSL256x$setTHRESHHIGHDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 47
static void Tsl2561InternalP$HplTSL256x$measureCh1Done(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 47 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error, uint16_t val);
#line 67
static void Tsl2561InternalP$HplTSL256x$alertThreshold(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730);
# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
static void Tsl2561InternalP$HplTSL256x$setINTERRUPTDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 44
static void Tsl2561InternalP$HplTSL256x$measureCh0Done(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 44 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error, uint16_t val);
#line 56
static void Tsl2561InternalP$HplTSL256x$setTHRESHLOWDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 56 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 50
static void Tsl2561InternalP$HplTSL256x$setCONTROLDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 50 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);


static void Tsl2561InternalP$HplTSL256x$setTIMINGDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 53 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error);
#line 65
static void Tsl2561InternalP$HplTSL256x$getIDDone(
# 38 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t arg_0x407fc730, 
# 65 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
error_t error, uint8_t idval);
# 42 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
static error_t Tsl2561InternalP$InterruptAlert$enableRisingEdge(void );
# 45 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
uint8_t Tsl2561InternalP$currentId;

static inline error_t Tsl2561InternalP$Init$init(void );







static inline error_t Tsl2561InternalP$HplTSL256x$measureCh0(uint8_t id);



static inline error_t Tsl2561InternalP$HplTSL256x$measureCh1(uint8_t id);
#line 88
static inline void Tsl2561InternalP$ToHPLC$measureCh0Done(error_t result, uint16_t val);


static inline void Tsl2561InternalP$ToHPLC$measureCh1Done(error_t result, uint16_t val);


static inline void Tsl2561InternalP$ToHPLC$setCONTROLDone(error_t error);


static inline void Tsl2561InternalP$ToHPLC$setTIMINGDone(error_t error);


static inline void Tsl2561InternalP$ToHPLC$setTHRESHLOWDone(error_t error);


static inline void Tsl2561InternalP$ToHPLC$setTHRESHHIGHDone(error_t error);


static inline void Tsl2561InternalP$ToHPLC$setINTERRUPTDone(error_t error);


static inline void Tsl2561InternalP$ToHPLC$getIDDone(error_t error, uint8_t idval);


static inline void Tsl2561InternalP$ToHPLC$alertThreshold(void );



static inline void Tsl2561InternalP$InterruptAlert$fired(void );

static inline void Tsl2561InternalP$HplTSL256x$default$measureCh0Done(uint8_t id, error_t error, uint16_t val);
static inline void Tsl2561InternalP$HplTSL256x$default$measureCh1Done(uint8_t id, error_t error, uint16_t val);
static inline void Tsl2561InternalP$HplTSL256x$default$setCONTROLDone(uint8_t id, error_t error);
static inline void Tsl2561InternalP$HplTSL256x$default$setTIMINGDone(uint8_t id, error_t error);
static inline void Tsl2561InternalP$HplTSL256x$default$setTHRESHLOWDone(uint8_t id, error_t error);
static inline void Tsl2561InternalP$HplTSL256x$default$setTHRESHHIGHDone(uint8_t id, error_t error);
static inline void Tsl2561InternalP$HplTSL256x$default$setINTERRUPTDone(uint8_t id, error_t error);
static inline void Tsl2561InternalP$HplTSL256x$default$getIDDone(uint8_t id, error_t error, uint8_t idval);
static inline void Tsl2561InternalP$HplTSL256x$default$alertThreshold(uint8_t id);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$send(
#line 56
message_t * msg, 







uint8_t len);
# 99 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$sendDone(
# 36 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40826200, 
# 92 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
message_t * msg, 






error_t error);
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$receive(
# 37 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
am_id_t arg_0x40826bc0, 
# 60 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 49 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(message_t * msg);







static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len);
#line 85
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(message_t *msg, error_t result);







static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(uint8_t id, message_t *msg, void *payload, uint8_t len);



static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(message_t *msg, void *payload, uint8_t len);








static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$payloadLength(message_t *msg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$setPayloadLength(message_t *msg, uint8_t len);



static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$maxPayloadLength(void );



static void */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$getPayload(message_t *msg, uint8_t len);
#line 132
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$destination(message_t *amsg);









static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setDestination(message_t *amsg, am_addr_t addr);
#line 156
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(message_t *amsg);




static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setType(message_t *amsg, am_id_t type);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
static void SerialP$SplitControl$startDone(error_t error);
#line 117
static void SerialP$SplitControl$stopDone(error_t error);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t SerialP$stopDoneTask$postTask(void );
# 74 "/opt/tinyos-2.1.0/tos/interfaces/StdControl.nc"
static error_t SerialP$SerialControl$start(void );









static error_t SerialP$SerialControl$stop(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t SerialP$RunTx$postTask(void );
# 38 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFlush.nc"
static void SerialP$SerialFlush$flush(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t SerialP$startDoneTask$postTask(void );
# 45 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
static error_t SerialP$SerialFrameComm$putDelimiter(void );
#line 68
static void SerialP$SerialFrameComm$resetReceive(void );
#line 54
static error_t SerialP$SerialFrameComm$putData(uint8_t data);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t SerialP$defaultSerialFlushTask$postTask(void );
# 70 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
static uint8_t SerialP$SendBytePacket$nextByte(void );









static void SerialP$SendBytePacket$sendCompleted(error_t error);
# 51 "/opt/tinyos-2.1.0/tos/lib/serial/ReceiveBytePacket.nc"
static error_t SerialP$ReceiveBytePacket$startPacket(void );






static void SerialP$ReceiveBytePacket$byteReceived(uint8_t data);










static void SerialP$ReceiveBytePacket$endPacket(error_t result);
# 189 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
enum SerialP$__nesc_unnamed4296 {
#line 189
  SerialP$RunTx = 12U
};
#line 189
typedef int SerialP$__nesc_sillytask_RunTx[SerialP$RunTx];
#line 320
enum SerialP$__nesc_unnamed4297 {
#line 320
  SerialP$startDoneTask = 13U
};
#line 320
typedef int SerialP$__nesc_sillytask_startDoneTask[SerialP$startDoneTask];





enum SerialP$__nesc_unnamed4298 {
#line 326
  SerialP$stopDoneTask = 14U
};
#line 326
typedef int SerialP$__nesc_sillytask_stopDoneTask[SerialP$stopDoneTask];








enum SerialP$__nesc_unnamed4299 {
#line 335
  SerialP$defaultSerialFlushTask = 15U
};
#line 335
typedef int SerialP$__nesc_sillytask_defaultSerialFlushTask[SerialP$defaultSerialFlushTask];
#line 79
enum SerialP$__nesc_unnamed4300 {
  SerialP$RX_DATA_BUFFER_SIZE = 2, 
  SerialP$TX_DATA_BUFFER_SIZE = 4, 
  SerialP$SERIAL_MTU = 255, 
  SerialP$SERIAL_VERSION = 1, 
  SerialP$ACK_QUEUE_SIZE = 5
};

enum SerialP$__nesc_unnamed4301 {
  SerialP$RXSTATE_NOSYNC, 
  SerialP$RXSTATE_PROTO, 
  SerialP$RXSTATE_TOKEN, 
  SerialP$RXSTATE_INFO, 
  SerialP$RXSTATE_INACTIVE
};

enum SerialP$__nesc_unnamed4302 {
  SerialP$TXSTATE_IDLE, 
  SerialP$TXSTATE_PROTO, 
  SerialP$TXSTATE_SEQNO, 
  SerialP$TXSTATE_INFO, 
  SerialP$TXSTATE_FCS1, 
  SerialP$TXSTATE_FCS2, 
  SerialP$TXSTATE_ENDFLAG, 
  SerialP$TXSTATE_ENDWAIT, 
  SerialP$TXSTATE_FINISH, 
  SerialP$TXSTATE_ERROR, 
  SerialP$TXSTATE_INACTIVE
};





#line 109
typedef enum SerialP$__nesc_unnamed4303 {
  SerialP$BUFFER_AVAILABLE, 
  SerialP$BUFFER_FILLING, 
  SerialP$BUFFER_COMPLETE
} SerialP$tx_data_buffer_states_t;

enum SerialP$__nesc_unnamed4304 {
  SerialP$TX_ACK_INDEX = 0, 
  SerialP$TX_DATA_INDEX = 1, 
  SerialP$TX_BUFFER_COUNT = 2
};






#line 122
typedef struct SerialP$__nesc_unnamed4305 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP$RX_DATA_BUFFER_SIZE + 1];
} SerialP$rx_buf_t;




#line 128
typedef struct SerialP$__nesc_unnamed4306 {
  uint8_t state;
  uint8_t buf;
} SerialP$tx_buf_t;





#line 133
typedef struct SerialP$__nesc_unnamed4307 {
  uint8_t writePtr;
  uint8_t readPtr;
  uint8_t buf[SerialP$ACK_QUEUE_SIZE + 1];
} SerialP$ack_queue_t;



SerialP$rx_buf_t SerialP$rxBuf;
SerialP$tx_buf_t SerialP$txBuf[SerialP$TX_BUFFER_COUNT];



uint8_t SerialP$rxState;
uint8_t SerialP$rxByteCnt;
uint8_t SerialP$rxProto;
uint8_t SerialP$rxSeqno;
uint16_t SerialP$rxCRC;



uint8_t SerialP$txState;
uint8_t SerialP$txByteCnt;
uint8_t SerialP$txProto;
uint8_t SerialP$txSeqno;
uint16_t SerialP$txCRC;
uint8_t SerialP$txPending;
uint8_t SerialP$txIndex;


SerialP$ack_queue_t SerialP$ackQ;

bool SerialP$offPending = FALSE;



static __inline void SerialP$txInit(void );
static __inline void SerialP$rxInit(void );
static __inline void SerialP$ackInit(void );

static __inline bool SerialP$ack_queue_is_full(void );
static __inline bool SerialP$ack_queue_is_empty(void );
static __inline void SerialP$ack_queue_push(uint8_t token);
static __inline uint8_t SerialP$ack_queue_top(void );
static inline uint8_t SerialP$ack_queue_pop(void );




static __inline void SerialP$rx_buffer_push(uint8_t data);
static __inline uint8_t SerialP$rx_buffer_top(void );
static __inline uint8_t SerialP$rx_buffer_pop(void );
static __inline uint16_t SerialP$rx_current_crc(void );

static void SerialP$rx_state_machine(bool isDelimeter, uint8_t data);
static void SerialP$MaybeScheduleTx(void );




static __inline void SerialP$txInit(void );
#line 205
static __inline void SerialP$rxInit(void );








static __inline void SerialP$ackInit(void );



static inline error_t SerialP$Init$init(void );
#line 232
static __inline bool SerialP$ack_queue_is_full(void );









static __inline bool SerialP$ack_queue_is_empty(void );





static __inline void SerialP$ack_queue_push(uint8_t token);









static __inline uint8_t SerialP$ack_queue_top(void );









static inline uint8_t SerialP$ack_queue_pop(void );
#line 295
static __inline void SerialP$rx_buffer_push(uint8_t data);



static __inline uint8_t SerialP$rx_buffer_top(void );



static __inline uint8_t SerialP$rx_buffer_pop(void );





static __inline uint16_t SerialP$rx_current_crc(void );










static inline void SerialP$startDoneTask$runTask(void );





static inline void SerialP$stopDoneTask$runTask(void );



static inline void SerialP$SerialFlush$flushDone(void );




static inline void SerialP$defaultSerialFlushTask$runTask(void );


static inline void SerialP$SerialFlush$default$flush(void );



static inline error_t SerialP$SplitControl$start(void );




static void SerialP$testOff(void );
#line 384
static inline void SerialP$SerialFrameComm$delimiterReceived(void );


static inline void SerialP$SerialFrameComm$dataReceived(uint8_t data);



static inline bool SerialP$valid_rx_proto(uint8_t proto);










static void SerialP$rx_state_machine(bool isDelimeter, uint8_t data);
#line 502
static void SerialP$MaybeScheduleTx(void );










static inline error_t SerialP$SendBytePacket$completeSend(void );








static inline error_t SerialP$SendBytePacket$startSend(uint8_t b);
#line 539
static inline void SerialP$RunTx$runTask(void );
#line 642
static inline void SerialP$SerialFrameComm$putDone(void );
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$postTask(void );
# 89 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$sendDone(
# 40 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5030, 
# 85 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$postTask(void );
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
static 
#line 63
message_t * 



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$receive(
# 39 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e89d0, 
# 60 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
message_t * msg, 
void * payload, 





uint8_t len);
# 31 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$upperLength(
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5b30, 
# 31 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t dataLinkLen);
#line 15
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5b30);
# 23 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$dataLinkLength(
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
uart_id_t arg_0x408e5b30, 
# 23 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
message_t *msg, uint8_t upperLen);
# 60 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$completeSend(void );
#line 51
static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$startSend(uint8_t first_byte);
# 147 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4308 {
#line 147
  SerialDispatcherP$0$signalSendDone = 16U
};
#line 147
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_sillytask_signalSendDone[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone];
#line 264
enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4309 {
#line 264
  SerialDispatcherP$0$receiveTask = 17U
};
#line 264
typedef int /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_sillytask_receiveTask[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask];
#line 55
#line 51
typedef enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4310 {
  SerialDispatcherP$0$SEND_STATE_IDLE = 0, 
  SerialDispatcherP$0$SEND_STATE_BEGIN = 1, 
  SerialDispatcherP$0$SEND_STATE_DATA = 2
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$send_state_t;

enum /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4311 {
  SerialDispatcherP$0$RECV_STATE_IDLE = 0, 
  SerialDispatcherP$0$RECV_STATE_BEGIN = 1, 
  SerialDispatcherP$0$RECV_STATE_DATA = 2
};






#line 63
typedef struct /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$__nesc_unnamed4312 {
  uint8_t which : 1;
  uint8_t bufZeroLocked : 1;
  uint8_t bufOneLocked : 1;
  uint8_t state : 2;
} /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recv_state_t;



/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recv_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState = { 0, 0, 0, /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_IDLE };
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType = TOS_SERIAL_UNKNOWN_ID;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex = 0;


message_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[2];
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messagePtrs[2] = { &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[0], &/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[1] };




uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer = (uint8_t * )&/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messages[0];

uint8_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer = (void *)0;
/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$send_state_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendLen = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex = 0;
error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError = SUCCESS;
bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendCancelled = FALSE;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendId = 0;


uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending = FALSE;
uart_id_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskType = 0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskWhich;
message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskBuf = (void *)0;
uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskSize = 0;

static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(uint8_t id, message_t *msg, uint8_t len);
#line 147
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask(void );
#line 167
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte(void );
#line 183
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error_t error);




static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$isCurrentBufferLocked(void );



static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$lockCurrentBuffer(void );








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$unlockBuffer(uint8_t which);








static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBufferSwap(void );




static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket(void );
#line 233
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(uint8_t b);
#line 264
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask(void );
#line 285
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(error_t result);
#line 344
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(uart_id_t id);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen);


static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen);




static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len);


static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(uart_id_t idxxx, message_t *msg, error_t error);
# 48 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
static error_t HdlcTranslateC$UartStream$send(
#line 44
uint8_t * buf, 



uint16_t len);
# 83 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
static void HdlcTranslateC$SerialFrameComm$dataReceived(uint8_t data);





static void HdlcTranslateC$SerialFrameComm$putDone(void );
#line 74
static void HdlcTranslateC$SerialFrameComm$delimiterReceived(void );
# 47 "/opt/tinyos-2.1.0/tos/lib/serial/HdlcTranslateC.nc"
#line 44
typedef struct HdlcTranslateC$__nesc_unnamed4313 {
  uint8_t sendEscape : 1;
  uint8_t receiveEscape : 1;
} HdlcTranslateC$HdlcState;


HdlcTranslateC$HdlcState HdlcTranslateC$state = { 0, 0 };
uint8_t HdlcTranslateC$txTemp;
uint8_t HdlcTranslateC$m_data;


static inline void HdlcTranslateC$SerialFrameComm$resetReceive(void );





static inline void HdlcTranslateC$UartStream$receivedByte(uint8_t data);
#line 86
static error_t HdlcTranslateC$SerialFrameComm$putDelimiter(void );





static error_t HdlcTranslateC$SerialFrameComm$putData(uint8_t data);
#line 104
static inline void HdlcTranslateC$UartStream$sendDone(uint8_t *buf, uint16_t len, 
error_t error);










static inline void HdlcTranslateC$UartStream$receiveDone(uint8_t *buf, uint16_t len, error_t error);
# 89 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialPacket.nc"
static uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$receiveDone(uint8_t *buf, uint16_t len, uart_status_t status);
#line 62
static uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$sendDone(uint8_t *buf, uint16_t len, uart_status_t status);
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTInit$init(void );
# 89 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDTADR(uint32_t val);
#line 87
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDSADR(uint32_t val);
#line 78
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDALGNbit(bool flag);


static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCSR(uint32_t val);
#line 77
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setMap(uint8_t dev);





static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCMD(uint32_t val);
# 44 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setDLL(uint32_t val);
#line 60
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setMCR(uint32_t val);
#line 53
static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIIR(void );
#line 47
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setDLH(uint32_t val);
#line 42
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setTHR(uint32_t val);
#line 63
static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getLSR(void );
#line 55
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setFCR(uint32_t val);
#line 51
static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER(void );
#line 41
static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getRBR(void );
#line 57
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setLCR(uint32_t val);
#line 50
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(uint32_t val);
# 79 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$receivedByte(uint8_t byte);
#line 99
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$receiveDone(
#line 95
uint8_t * buf, 



uint16_t len, error_t error);
#line 57
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$sendDone(
#line 53
uint8_t * buf, 



uint16_t len, error_t error);
# 63 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfo.nc"
static uint8_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTTxDMAInfo$getMapIndex(void );
#line 54
static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTTxDMAInfo$getAddr(void );
# 89 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDTADR(uint32_t val);
#line 87
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDSADR(uint32_t val);
#line 78
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDALGNbit(bool flag);


static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCSR(uint32_t val);
#line 77
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setMap(uint8_t dev);





static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCMD(uint32_t val);
# 63 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfo.nc"
static uint8_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTRxDMAInfo$getMapIndex(void );
#line 54
static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTRxDMAInfo$getAddr(void );
# 99 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentBuf;
#line 99
uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf;
uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentLen;
#line 100
uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentLen;
#line 100
uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentIdx;
uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow;
bool /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamSendIF = FALSE;
bool /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamRcvIF = FALSE;
bool /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbRcvByteEvtEnabled = TRUE;

static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$Init$init(void );
#line 128
static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$start(void );






static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$stop(void );
#line 161
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$send(uint8_t *buf, uint16_t len);
#line 204
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$send(uint8_t *buf, uint16_t len);
#line 257
static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$receive(uint8_t *buf, uint16_t len, 
uint16_t timeout);
#line 309
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamRcvSignal(void );
#line 326
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamSendSignal(void );
#line 343
static inline void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$interruptDMA(void );








static inline void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$interruptDMA(void );







static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialCntl$configPort(uint32_t baudrate, 
uint8_t databits, 
uart_parity_t parity, 
uint8_t stopbits, 
bool flow_cntl);
#line 425
static inline void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$interruptUART(void );
#line 476
static inline uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$sendDone(uint8_t *buf, 
uint16_t len, 
uart_status_t status);



static inline uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$receiveDone(uint8_t *buf, 
uint16_t len, 
uart_status_t status);
# 81 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$interruptUART(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$enable(void );
#line 60
static error_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$allocate(void );
# 55 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
bool /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$m_fInit = FALSE;

static inline error_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$Init$init(void );
#line 90
static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getRBR(void );
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setTHR(uint32_t val);
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLL(uint32_t val);
#line 104
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLH(uint32_t val);
#line 116
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setIER(uint32_t val);
static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIER(void );
static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIIR(void );
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setFCR(uint32_t val);
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setLCR(uint32_t val);

static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setMCR(uint32_t val);

static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getLSR(void );
#line 136
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$fired(void );
# 134 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void IM2InitSerialP$RXD$setGAFRpin(uint8_t func);
#line 52
static void IM2InitSerialP$RXD$setGPDRbit(bool dir);
#line 134
static void IM2InitSerialP$TXD$setGAFRpin(uint8_t func);
#line 52
static void IM2InitSerialP$TXD$setGPDRbit(bool dir);
# 47 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/IM2InitSerialP.nc"
static inline error_t IM2InitSerialP$Init$init(void );








static inline void IM2InitSerialP$TXD$interruptGPIOPin(void );
static inline void IM2InitSerialP$RXD$interruptGPIOPin(void );
# 50 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfoC.nc"
static inline uint32_t /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getAddr(void );



static inline uint8_t /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getMapIndex(void );
#line 50
static inline uint32_t /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getAddr(void );



static inline uint8_t /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getMapIndex(void );
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
static void HplPXA27xDMAM$DMAIrq$enable(void );
#line 60
static error_t HplPXA27xDMAM$DMAIrq$allocate(void );
# 91 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
static void HplPXA27xDMAM$HplPXA27xDMAChnl$interruptDMA(
# 40 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
uint8_t arg_0x40a13b80);









static inline error_t HplPXA27xDMAM$Init$init(void );





static void HplPXA27xDMAM$HplPXA27xDMACntl$setDRCMR(uint8_t peripheral, uint8_t val);
#line 70
static inline uint32_t HplPXA27xDMAM$HplPXA27xDMACntl$getDINT(void );




static inline error_t HplPXA27xDMAM$HplPXA27xDMAChnl$setMap(uint8_t chnl, uint8_t dev);



static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDALGNbit(uint8_t chnl, bool flag);
#line 94
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(uint8_t chnl, uint32_t val);






static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(uint8_t chnl, uint32_t val);



static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDSADR(uint8_t chnl, uint32_t val);

static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDTADR(uint8_t chnl, uint32_t val);


static inline void HplPXA27xDMAM$DMAIrq$fired(void );
#line 123
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(uint8_t chnl);
# 40 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$offset(void );


static inline uint8_t SerialPacketInfoActiveMessageP$Info$dataLinkLength(message_t *msg, uint8_t upperLen);


static inline uint8_t SerialPacketInfoActiveMessageP$Info$upperLength(message_t *msg, uint8_t dataLinkLen);
# 99 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$sendDone(
#line 92
message_t * msg, 






error_t error);
# 64 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static error_t /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$send(
#line 56
message_t * msg, 







uint8_t len);
# 92 "/opt/tinyos-2.1.0/tos/interfaces/AMPacket.nc"
static void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setDestination(
#line 88
message_t * amsg, 



am_addr_t addr);
#line 151
static void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setType(
#line 147
message_t * amsg, 



am_id_t t);
# 45 "/opt/tinyos-2.1.0/tos/system/AMQueueEntryP.nc"
static inline error_t /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(am_addr_t dest, 
message_t *msg, 
uint8_t len);









static inline void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(message_t *m, error_t err);
# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(
# 40 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
am_id_t arg_0x40a91ef8, 
# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
am_addr_t addr, 
#line 60
message_t * msg, 








uint8_t len);
# 89 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(
# 38 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
uint8_t arg_0x40a91510, 
# 85 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
message_t * msg, 



error_t error);
# 67 "/opt/tinyos-2.1.0/tos/interfaces/Packet.nc"
static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$payloadLength(
#line 63
message_t * msg);
#line 83
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$setPayloadLength(
#line 79
message_t * msg, 



uint8_t len);
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$postTask(void );
# 67 "/opt/tinyos-2.1.0/tos/interfaces/AMPacket.nc"
static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(
#line 63
message_t * amsg);
#line 136
static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(
#line 132
message_t * amsg);
# 118 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_unnamed4314 {
#line 118
  AMQueueImplP$0$CancelTask = 18U
};
#line 118
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_sillytask_CancelTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask];
#line 161
enum /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_unnamed4315 {
#line 161
  AMQueueImplP$0$errorTask = 19U
};
#line 161
typedef int /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_sillytask_errorTask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask];
#line 49
#line 47
typedef struct /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$__nesc_unnamed4316 {
  message_t * msg;
} /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue_entry_t;

uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current = 1;
/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue_entry_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[1];
uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[1 / 8 + 1];

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend(void );

static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$nextPacket(void );
#line 82
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(uint8_t clientId, message_t *msg, 
uint8_t len);
#line 118
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask(void );
#line 155
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(uint8_t last, message_t * msg, error_t err);





static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask(void );




static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend(void );
#line 181
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(am_id_t id, message_t *msg, error_t err);
#line 207
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(uint8_t id, message_t *msg, error_t err);
# 95 "/opt/tinyos-2.1.0/tos/chips/pxa27x/pxa27xhardware.h"
__inline  __nesc_atomic_t __nesc_atomic_start(void )
{
  uint32_t result = 0;
  uint32_t temp = 0;

   __asm volatile (
  "mrs %0,CPSR\n\t"
  "orr %1,%2,%4\n\t"
  "msr CPSR_cf,%3" : 
  "=r"(result), "=r"(temp) : 
  "0"(result), "1"(temp), "i"(0x000000C0));

   __asm volatile ("" :  :  : "memory");
  return result;
}

__inline  void __nesc_atomic_end(__nesc_atomic_t oldState)
{
  uint32_t statusReg = 0;

   __asm volatile ("" :  :  : "memory");
  oldState &= 0x000000C0;
   __asm volatile (
  "mrs %0,CPSR\n\t"
  "bic %0, %1, %2\n\t"
  "orr %0, %1, %3\n\t"
  "msr CPSR_c, %1" : 
  "=r"(statusReg) : 
  "0"(statusReg), "i"(0x000000C0), "r"(oldState));


  return;
}

# 59 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline uint32_t HplPXA27xInterruptM$getICHP(void )
#line 59
{
  uint32_t val;

   __asm volatile ("mrc p6,0,%0,c5,c0,0\n\t" : "=r"(val));
  return val;
}

# 55 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline void HplPXA27xOSTimerM$DispatchOSTInterrupt(uint8_t id)
{
  HplPXA27xOSTimerM$PXA27xOST$fired(id);
  return;
}

# 86 "/opt/tinyos-2.1.0/tos/chips/pxa27x/pxa27xhardware.h"
static __inline uint32_t _pxa27x_clzui(uint32_t i)
#line 86
{
  uint32_t count;

#line 88
   __asm volatile ("clz %0,%1" : "=r"(count) : "r"(i));
  return count;
}

# 216 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline void HplPXA27xOSTimerM$OST4_11Irq$fired(void )
{
  uint32_t statusReg;
  uint8_t chnl;

  statusReg = * (volatile uint32_t *)0x40A00014;
  statusReg &= ~((((1 << 3) | (1 << 2)) | (1 << 1)) | (1 << 0));

  while (statusReg) {
      chnl = 31 - _pxa27x_clzui(statusReg);
      HplPXA27xOSTimerM$DispatchOSTInterrupt(chnl);
      statusReg &= ~(1 << chnl);
    }

  return;
}

#line 211
static inline void HplPXA27xOSTimerM$OST3Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(3);
}

#line 206
static inline void HplPXA27xOSTimerM$OST2Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(2);
}

#line 201
static inline void HplPXA27xOSTimerM$OST1Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(1);
}

#line 196
static inline void HplPXA27xOSTimerM$OST0Irq$fired(void )
{
  HplPXA27xOSTimerM$DispatchOSTInterrupt(0);
}

# 140 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(uint32_t val)
#line 140
{
  switch (1) {
      case 0: * (volatile uint32_t *)0x403016A0 = val;
#line 142
      break;
      case 1: * (volatile uint32_t *)0x40F001A0 = val;
#line 143
      break;
      default: break;
    }
  return;
}

# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void PMICM$PI2C$setISAR(uint32_t val){
#line 51
  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setISAR(val);
#line 51
}
#line 51
# 132 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR(void )
#line 132
{
  switch (1) {
      case 0: return * (volatile uint32_t *)0x40301698;
#line 134
      break;
      case 1: return * (volatile uint32_t *)0x40F00198;
#line 135
      break;
      default: return 0;
    }
}

# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static uint32_t PMICM$PI2C$getISR(void ){
#line 49
  unsigned int __nesc_result;
#line 49

#line 49
  __nesc_result = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getISR();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 256 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline void PMICM$PI2C$interruptI2C(void )
#line 256
{
  uint32_t status;
#line 257
  uint32_t update = 0;

#line 258
  status = PMICM$PI2C$getISR();
  if (status & (1 << 6)) {
      update |= 1 << 6;
    }


  if (status & (1 << 10)) {
      update |= 1 << 10;
    }

  PMICM$PI2C$setISAR(update);
}

# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$interruptI2C(void ){
#line 54
  PMICM$PI2C$interruptI2C();
#line 54
}
#line 54
# 157 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired(void )
#line 157
{

  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$interruptI2C();
  return;
}

# 259 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$HplPXA27xGPIO$default$fired(void )
#line 259
{
  return;
}

# 132 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIO.nc"
inline static void HplPXA27xGPIOM$HplPXA27xGPIO$fired(void ){
#line 132
  HplPXA27xGPIOM$HplPXA27xGPIO$default$fired();
#line 132
}
#line 132
# 263 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$GPIOIrq$fired(void )
{

  uint32_t DetectReg;
  uint8_t pin;

  HplPXA27xGPIOM$HplPXA27xGPIO$fired();
  /* atomic removed: atomic calls only */

  DetectReg = * (volatile uint32_t *)0x40E00048 & ~((1 << 1) | (1 << 0));

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin);
      DetectReg &= ~(1 << pin);
    }
  /* atomic removed: atomic calls only */
  DetectReg = * (volatile uint32_t *)0x40E0004C;

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin + 32);
      DetectReg &= ~(1 << pin);
    }
  /* atomic removed: atomic calls only */
  DetectReg = * (volatile uint32_t *)0x40E00050;

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin + 64);
      DetectReg &= ~(1 << pin);
    }
  /* atomic removed: atomic calls only */
  DetectReg = * (volatile uint32_t *)0x40E00148;

  while (DetectReg) {
      pin = 31 - _pxa27x_clzui(DetectReg);
      HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(pin + 96);
      DetectReg &= ~(1 << pin);
    }

  return;
}






static inline void HplPXA27xGPIOM$GPIOIrq1$fired(void )
{
  HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(1);
}

#line 307
static inline void HplPXA27xGPIOM$GPIOIrq0$fired(void )
{
  HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(0);
}

# 101 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$writeDone(error_t error, uint16_t addr, uint8_t length, uint8_t * data){
#line 101
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$writeDone(error, addr, length, data);
#line 101
}
#line 101
# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(uint32_t val){
#line 45
  /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setICR(val);
#line 45
}
#line 45
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 42 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setIDBR(uint32_t val){
#line 42
  /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setIDBR(val);
#line 42
}
#line 42
# 91 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$readDone(error_t error, uint16_t addr, uint8_t length, uint8_t * data){
#line 91
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$readDone(error, addr, length, data);
#line 91
}
#line 91
# 43 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static uint32_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$getIDBR(void ){
#line 43
  unsigned int __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getIDBR();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 124 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISR(uint32_t val)
#line 124
{
  switch (0) {
      case 0: * (volatile uint32_t *)0x40301698 = val;
#line 126
      break;
      case 1: * (volatile uint32_t *)0x40F00198 = val;
#line 127
      break;
      default: break;
    }
}

# 48 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISR(uint32_t val){
#line 48
  /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISR(val);
#line 48
}
#line 48
# 132 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline uint32_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getISR(void )
#line 132
{
  switch (0) {
      case 0: return * (volatile uint32_t *)0x40301698;
#line 134
      break;
      case 1: return * (volatile uint32_t *)0x40F00198;
#line 135
      break;
      default: return 0;
    }
}

# 49 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static uint32_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$getISR(void ){
#line 49
  unsigned int __nesc_result;
#line 49

#line 49
  __nesc_result = /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getISR();
#line 49

#line 49
  return __nesc_result;
#line 49
}
#line 49
# 225 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$interruptI2C(void )
#line 225
{
  uint32_t valISR;


  valISR = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$getISR();
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISR((1 << 6) | (1 << 7));





  switch (/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState) {
      case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_IDLE: 

        break;

      case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_READSTART: 
        if (valISR & ((1 << 10) | (1 << 5))) {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_ERROR;
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$postTask();
            break;
          }
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$readNextByte();
      break;

      case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_READ: 
        if (valISR & ((1 << 10) | (1 << 5))) {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_ERROR;
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$postTask();
            break;
          }
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf[/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex] = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$getIDBR();
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex++;
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$readNextByte();
      break;

      case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_READEND: 
        if (valISR & ((1 << 10) | (1 << 5))) {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_ERROR;
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$postTask();
            break;
          }
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf[/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex] = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$getIDBR();
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_IDLE;
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$readDone(SUCCESS, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurTargetAddr, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf);
      break;

      case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_WRITE: 
        if (valISR & ((1 << 10) | (1 << 5))) {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_ERROR;
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$postTask();
            break;
          }
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setIDBR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf[/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex]);
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex++;
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$writeNextByte();

      break;

      case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_WRITEEND: 
        if (valISR & ((1 << 10) | (1 << 5))) {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_ERROR;
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$postTask();
            break;
          }
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_IDLE;

      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags);
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$writeDone(SUCCESS, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurTargetAddr, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf);
      break;

      default: 
        break;
    }


  return;
}

# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$interruptI2C(void ){
#line 54
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$interruptI2C();
#line 54
}
#line 54
# 157 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$fired(void )
#line 157
{

  /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$interruptI2C();
  return;
}

# 124 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getLSR(void )
#line 124
{
#line 124
  return * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x14);
}

# 63 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getLSR(void ){
#line 63
  unsigned int __nesc_result;
#line 63

#line 63
  __nesc_result = /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getLSR();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 387 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFrameComm$dataReceived(uint8_t data)
#line 387
{
  SerialP$rx_state_machine(FALSE, data);
}

# 83 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC$SerialFrameComm$dataReceived(uint8_t data){
#line 83
  SerialP$SerialFrameComm$dataReceived(data);
#line 83
}
#line 83
# 384 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFrameComm$delimiterReceived(void )
#line 384
{
  SerialP$rx_state_machine(TRUE, 0);
}

# 74 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC$SerialFrameComm$delimiterReceived(void ){
#line 74
  SerialP$SerialFrameComm$delimiterReceived();
#line 74
}
#line 74
# 61 "/opt/tinyos-2.1.0/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$UartStream$receivedByte(uint8_t data)
#line 61
{






  if (data == HDLC_FLAG_BYTE) {

      HdlcTranslateC$SerialFrameComm$delimiterReceived();
      return;
    }
  else {
#line 73
    if (data == HDLC_CTLESC_BYTE) {

        HdlcTranslateC$state.receiveEscape = 1;
        return;
      }
    else {
#line 78
      if (HdlcTranslateC$state.receiveEscape) {

          HdlcTranslateC$state.receiveEscape = 0;
          data = data ^ 0x20;
        }
      }
    }
#line 83
  HdlcTranslateC$SerialFrameComm$dataReceived(data);
}

# 79 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$receivedByte(uint8_t byte){
#line 79
  HdlcTranslateC$UartStream$receivedByte(byte);
#line 79
}
#line 79
# 90 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getRBR(void )
#line 90
{
#line 90
  return * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0);
}

# 41 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getRBR(void ){
#line 41
  unsigned int __nesc_result;
#line 41

#line 41
  __nesc_result = /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getRBR();
#line 41

#line 41
  return __nesc_result;
#line 41
}
#line 41
# 117 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIER(void )
#line 117
{
#line 117
  return * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x04);
}

# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER(void ){
#line 51
  unsigned int __nesc_result;
#line 51

#line 51
  __nesc_result = /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIER();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 116 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setIER(uint32_t val)
#line 116
{
#line 116
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x04) = val;
}

# 50 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(uint32_t val){
#line 50
  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setIER(val);
#line 50
}
#line 50
# 118 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline uint32_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIIR(void )
#line 118
{
#line 118
  return * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x08);
}

# 53 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIIR(void ){
#line 53
  unsigned int __nesc_result;
#line 53

#line 53
  __nesc_result = /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$getIIR();
#line 53

#line 53
  return __nesc_result;
#line 53
}
#line 53
# 425 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$interruptUART(void )
#line 425
{
  uint8_t error;
#line 426
  uint8_t intSource;
  uint8_t ucByte;

  intSource = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIIR();
  intSource &= 0x3 << 1;
  intSource = intSource >> 1;

  switch (intSource) {
      case 0: 
        break;
      case 1: 
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER() & ~(1 << 1));
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamSendSignal();
      break;
      case 2: 
        while (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getLSR() & (1 << 0)) {
            ucByte = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getRBR();

            if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf != (void *)0) {
                /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf[/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentIdx] = ucByte;
                /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentIdx++;
                if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentIdx >= /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentLen) {
                  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamRcvSignal();
                  }
              }
            else {
#line 450
              if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbRcvByteEvtEnabled) {
                  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$receivedByte(ucByte);
                }
              }
          }
#line 454
      break;
      case 3: 
        error = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getLSR();
      break;
      default: 
        break;
    }
  return;
}

# 81 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$interruptUART(void ){
#line 81
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$interruptUART();
#line 81
}
#line 81
# 136 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$fired(void )
#line 136
{

  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$interruptUART();
}

# 94 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(uint8_t chnl, uint32_t val)
#line 94
{


  * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40000000 + (uint32_t )(chnl << 2)) = val;
}


static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(uint8_t chnl, uint32_t val)
#line 101
{
#line 101
  * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x4000020C + (uint32_t )(chnl << 4)) = val;
}

# 81 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCSR(uint32_t val){
#line 81
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(3, val);
#line 81
}
#line 81


inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCMD(uint32_t val){
#line 83
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(3, val);
#line 83
}
#line 83
# 343 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$interruptDMA(void )
#line 343
{
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCMD(0);
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCSR((((1 << 9) | (1 << 2)) | (1 << 1)) | (1 << 0));
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamRcvSignal();
  if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbRcvByteEvtEnabled) {
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER() | (1 << 0));
    }
#line 349
  return;
}

# 81 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCSR(uint32_t val){
#line 81
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(2, val);
#line 81
}
#line 81


inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCMD(uint32_t val){
#line 83
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(2, val);
#line 83
}
#line 83
# 352 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$interruptDMA(void )
#line 352
{
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCMD(0);
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCSR((((1 << 9) | (1 << 2)) | (1 << 1)) | (1 << 0));
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamSendSignal();
  return;
}

# 123 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(uint8_t chnl)
#line 123
{
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCMD(chnl, 0);
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDCSR(chnl, (((1 << 9) | (1 << 2))
   | (1 << 1)) | (1 << 0));
}

# 91 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void HplPXA27xDMAM$HplPXA27xDMAChnl$interruptDMA(uint8_t arg_0x40a13b80){
#line 91
  switch (arg_0x40a13b80) {
#line 91
    case 2:
#line 91
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$interruptDMA();
#line 91
      break;
#line 91
    case 3:
#line 91
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$interruptDMA();
#line 91
      break;
#line 91
    default:
#line 91
      HplPXA27xDMAM$HplPXA27xDMAChnl$default$interruptDMA(arg_0x40a13b80);
#line 91
      break;
#line 91
    }
#line 91
}
#line 91
# 70 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline uint32_t HplPXA27xDMAM$HplPXA27xDMACntl$getDINT(void )
#line 70
{
#line 70
  return * (volatile uint32_t *)0x400000F0;
}

#line 110
static inline void HplPXA27xDMAM$DMAIrq$fired(void )
#line 110
{
  uint32_t IntReg;
  uint8_t chnl;

#line 113
  IntReg = HplPXA27xDMAM$HplPXA27xDMACntl$getDINT();

  while (IntReg) {
      chnl = 31 - _pxa27x_clzui(IntReg);
      HplPXA27xDMAM$HplPXA27xDMAChnl$interruptDMA(chnl);
      IntReg &= ~(1 << chnl);
    }
  return;
}

# 227 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline void HplPXA27xInterruptM$PXA27xIrq$default$fired(uint8_t id)
{
  return;
}

# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xInterruptM$PXA27xIrq$fired(uint8_t arg_0x4044fc48){
#line 75
  switch (arg_0x4044fc48) {
#line 75
    case 6:
#line 75
      /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$fired();
#line 75
      break;
#line 75
    case 7:
#line 75
      HplPXA27xOSTimerM$OST4_11Irq$fired();
#line 75
      break;
#line 75
    case 8:
#line 75
      HplPXA27xGPIOM$GPIOIrq0$fired();
#line 75
      break;
#line 75
    case 9:
#line 75
      HplPXA27xGPIOM$GPIOIrq1$fired();
#line 75
      break;
#line 75
    case 10:
#line 75
      HplPXA27xGPIOM$GPIOIrq$fired();
#line 75
      break;
#line 75
    case 18:
#line 75
      /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$fired();
#line 75
      break;
#line 75
    case 20:
#line 75
      /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$fired();
#line 75
      break;
#line 75
    case 25:
#line 75
      HplPXA27xDMAM$DMAIrq$fired();
#line 75
      break;
#line 75
    case 26:
#line 75
      HplPXA27xOSTimerM$OST0Irq$fired();
#line 75
      break;
#line 75
    case 27:
#line 75
      HplPXA27xOSTimerM$OST1Irq$fired();
#line 75
      break;
#line 75
    case 28:
#line 75
      HplPXA27xOSTimerM$OST2Irq$fired();
#line 75
      break;
#line 75
    case 29:
#line 75
      HplPXA27xOSTimerM$OST3Irq$fired();
#line 75
      break;
#line 75
    default:
#line 75
      HplPXA27xInterruptM$PXA27xIrq$default$fired(arg_0x4044fc48);
#line 75
      break;
#line 75
    }
#line 75
}
#line 75
# 45 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
inline static error_t SerialP$SerialFrameComm$putDelimiter(void ){
#line 45
  unsigned char __nesc_result;
#line 45

#line 45
  __nesc_result = HdlcTranslateC$SerialFrameComm$putDelimiter();
#line 45

#line 45
  return __nesc_result;
#line 45
}
#line 45









inline static error_t SerialP$SerialFrameComm$putData(uint8_t data){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = HdlcTranslateC$SerialFrameComm$putData(data);
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 513 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline error_t SerialP$SendBytePacket$completeSend(void )
#line 513
{
  bool ret = FAIL;

  /* atomic removed: atomic calls only */
#line 515
  {
    SerialP$txBuf[SerialP$TX_DATA_INDEX].state = SerialP$BUFFER_COMPLETE;
    ret = SUCCESS;
  }
  return ret;
}

# 60 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$completeSend(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = SerialP$SendBytePacket$completeSend();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 167 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte(void )
#line 167
{
  uint8_t b;
  uint8_t indx;

  /* atomic removed: atomic calls only */
#line 170
  {
    b = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex];
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex++;
    indx = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex;
  }
  if (indx > /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendLen) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$completeSend();
      return 0;
    }
  else {
      return b;
    }
}

# 70 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
inline static uint8_t SerialP$SendBytePacket$nextByte(void ){
#line 70
  unsigned char __nesc_result;
#line 70

#line 70
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$nextByte();
#line 70

#line 70
  return __nesc_result;
#line 70
}
#line 70
# 642 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFrameComm$putDone(void )
#line 642
{
  {
    error_t txResult = SUCCESS;

    switch (SerialP$txState) {

        case SerialP$TXSTATE_PROTO: 

          txResult = SerialP$SerialFrameComm$putData(SerialP$txProto);

        SerialP$txState = SerialP$TXSTATE_INFO;



        SerialP$txCRC = crcByte(SerialP$txCRC, SerialP$txProto);
        break;

        case SerialP$TXSTATE_SEQNO: 
          txResult = SerialP$SerialFrameComm$putData(SerialP$txSeqno);
        SerialP$txState = SerialP$TXSTATE_INFO;
        SerialP$txCRC = crcByte(SerialP$txCRC, SerialP$txSeqno);
        break;

        case SerialP$TXSTATE_INFO: /* atomic removed: atomic calls only */
          {
            txResult = SerialP$SerialFrameComm$putData(SerialP$txBuf[SerialP$txIndex].buf);
            SerialP$txCRC = crcByte(SerialP$txCRC, SerialP$txBuf[SerialP$txIndex].buf);
            ++SerialP$txByteCnt;

            if (SerialP$txIndex == SerialP$TX_DATA_INDEX) {
                uint8_t nextByte;

#line 673
                nextByte = SerialP$SendBytePacket$nextByte();
                if (SerialP$txBuf[SerialP$txIndex].state == SerialP$BUFFER_COMPLETE || SerialP$txByteCnt >= SerialP$SERIAL_MTU) {
                    SerialP$txState = SerialP$TXSTATE_FCS1;
                  }
                else {
                    SerialP$txBuf[SerialP$txIndex].buf = nextByte;
                  }
              }
            else {
                SerialP$txState = SerialP$TXSTATE_FCS1;
              }
          }
        break;

        case SerialP$TXSTATE_FCS1: 
          txResult = SerialP$SerialFrameComm$putData(SerialP$txCRC & 0xff);
        SerialP$txState = SerialP$TXSTATE_FCS2;
        break;

        case SerialP$TXSTATE_FCS2: 
          txResult = SerialP$SerialFrameComm$putData((SerialP$txCRC >> 8) & 0xff);
        SerialP$txState = SerialP$TXSTATE_ENDFLAG;
        break;

        case SerialP$TXSTATE_ENDFLAG: 
          txResult = SerialP$SerialFrameComm$putDelimiter();
        SerialP$txState = SerialP$TXSTATE_ENDWAIT;
        break;

        case SerialP$TXSTATE_ENDWAIT: 
          SerialP$txState = SerialP$TXSTATE_FINISH;
        case SerialP$TXSTATE_FINISH: 
          SerialP$MaybeScheduleTx();
        break;
        case SerialP$TXSTATE_ERROR: 
          default: 
            txResult = FAIL;
        break;
      }

    if (txResult != SUCCESS) {
        SerialP$txState = SerialP$TXSTATE_ERROR;
        SerialP$MaybeScheduleTx();
      }
  }
}

# 89 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
inline static void HdlcTranslateC$SerialFrameComm$putDone(void ){
#line 89
  SerialP$SerialFrameComm$putDone();
#line 89
}
#line 89
# 48 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
inline static error_t HdlcTranslateC$UartStream$send(uint8_t * buf, uint16_t len){
#line 48
  unsigned char __nesc_result;
#line 48

#line 48
  __nesc_result = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$send(buf, len);
#line 48

#line 48
  return __nesc_result;
#line 48
}
#line 48
# 104 "/opt/tinyos-2.1.0/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$UartStream$sendDone(uint8_t *buf, uint16_t len, 
error_t error)
#line 105
{
  if (HdlcTranslateC$state.sendEscape) {
      HdlcTranslateC$state.sendEscape = 0;
      HdlcTranslateC$m_data = HdlcTranslateC$txTemp;
      HdlcTranslateC$UartStream$send(&HdlcTranslateC$m_data, 1);
    }
  else {
      HdlcTranslateC$SerialFrameComm$putDone();
    }
}

# 57 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$sendDone(uint8_t * buf, uint16_t len, error_t error){
#line 57
  HdlcTranslateC$UartStream$sendDone(buf, len, error);
#line 57
}
#line 57
# 91 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setTHR(uint32_t val)
#line 91
{
#line 91
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0) = val;
}

# 42 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setTHR(uint32_t val){
#line 42
  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setTHR(val);
#line 42
}
#line 42
# 105 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDSADR(uint8_t chnl, uint32_t val)
#line 105
{
#line 105
  * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40000204 + (uint32_t )(chnl << 4)) = val;
}

# 87 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDSADR(uint32_t val){
#line 87
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDSADR(2, val);
#line 87
}
#line 87
# 107 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline void HplPXA27xDMAM$HplPXA27xDMAChnl$setDTADR(uint8_t chnl, uint32_t val)
#line 107
{
#line 107
  * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40000208 + (uint32_t )(chnl << 4)) = val;
}

# 89 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDTADR(uint32_t val){
#line 89
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDTADR(2, val);
#line 89
}
#line 89
# 50 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfoC.nc"
static inline uint32_t /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getAddr(void )
#line 50
{
  return 1081081856U;
}

# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfo.nc"
inline static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTTxDMAInfo$getAddr(void ){
#line 54
  unsigned int __nesc_result;
#line 54

#line 54
  __nesc_result = /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getAddr();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$RunTx$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(SerialP$RunTx);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 86 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static inline bool SchedulerBasicP$isWaiting(uint8_t id)
{
  return SchedulerBasicP$m_next[id] != SchedulerBasicP$NO_TASK || SchedulerBasicP$m_tail == id;
}

static inline bool SchedulerBasicP$pushTask(uint8_t id)
{
  if (!SchedulerBasicP$isWaiting(id)) 
    {
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_head = id;
          SchedulerBasicP$m_tail = id;
        }
      else 
        {
          SchedulerBasicP$m_next[SchedulerBasicP$m_tail] = id;
          SchedulerBasicP$m_tail = id;
        }
      return TRUE;
    }
  else 
    {
      return FALSE;
    }
}

# 476 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$sendDone(uint8_t *buf, 
uint16_t len, 
uart_status_t status)
#line 478
{
  return (void *)0;
}

# 62 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialPacket.nc"
inline static uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$sendDone(uint8_t *buf, uint16_t len, uart_status_t status){
#line 62
  unsigned char *__nesc_result;
#line 62

#line 62
  __nesc_result = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$sendDone(buf, len, status);
#line 62

#line 62
  return __nesc_result;
#line 62
}
#line 62
# 116 "/opt/tinyos-2.1.0/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$UartStream$receiveDone(uint8_t *buf, uint16_t len, error_t error)
#line 116
{
}

# 99 "/opt/tinyos-2.1.0/tos/interfaces/UartStream.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$receiveDone(uint8_t * buf, uint16_t len, error_t error){
#line 99
  HdlcTranslateC$UartStream$receiveDone(buf, len, error);
#line 99
}
#line 99
# 482 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$receiveDone(uint8_t *buf, 
uint16_t len, 
uart_status_t status)
#line 484
{
  return (void *)0;
}

# 89 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialPacket.nc"
inline static uint8_t */*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$receiveDone(uint8_t *buf, uint16_t len, uart_status_t status){
#line 89
  unsigned char *__nesc_result;
#line 89

#line 89
  __nesc_result = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$default$receiveDone(buf, len, status);
#line 89

#line 89
  return __nesc_result;
#line 89
}
#line 89
# 119 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setFCR(uint32_t val)
#line 119
{
#line 119
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x08) = val;
}

# 55 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setFCR(uint32_t val){
#line 55
  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setFCR(val);
#line 55
}
#line 55
# 50 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfoC.nc"
static inline uint32_t /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getAddr(void )
#line 50
{
  return 1081081856U;
}

# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfo.nc"
inline static uint32_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTRxDMAInfo$getAddr(void ){
#line 54
  unsigned int __nesc_result;
#line 54

#line 54
  __nesc_result = /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getAddr();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 87 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDSADR(uint32_t val){
#line 87
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDSADR(3, val);
#line 87
}
#line 87


inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDTADR(uint32_t val){
#line 89
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDTADR(3, val);
#line 89
}
#line 89
# 257 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$receive(uint8_t *buf, uint16_t len, 
uint16_t timeout)
#line 258
{
  uint32_t rxAddr;
  uint32_t DMAFlags;
  error_t error = SUCCESS;

  /* atomic removed: atomic calls only */
#line 263
  {
    if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf == (void *)0) {
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf = buf;
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentLen = len;
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentIdx = 0;
      }
    else {
        error = FAIL;
      }
  }

  if (error) {
    return error;
    }
  if (len < 8) {
      /* atomic removed: atomic calls only */
      {
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow = (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow & ~(3 << 6)) | (0 << 6);
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setFCR(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow);
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER() | (1 << 0));
      }
    }
  else {

      DMAFlags = ((((1 << 29) | (1 << 16)) | (1 << 14)) | (1 << 21))
       | (len & 0x1fff);

      rxAddr = (uint32_t )buf;
      DMAFlags |= 1 << 30;

      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCSR(1 << 30);
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDTADR(rxAddr);
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDSADR(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTRxDMAInfo$getAddr());
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCMD(DMAFlags);
      /* atomic removed: atomic calls only */
      {
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow = (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow & ~(3 << 6)) | (1 << 6);
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setFCR(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow);
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER((/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER() & ~(1 << 0)) | (1 << 7));
      }

      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDCSR((1 << 31) | (1 << 30));
    }
  return error;
}

# 391 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline bool SerialP$valid_rx_proto(uint8_t proto)
#line 391
{
  switch (proto) {
      case SERIAL_PROTO_PACKET_ACK: 
        return TRUE;
      case SERIAL_PROTO_ACK: 
        case SERIAL_PROTO_PACKET_NOACK: 
          default: 
            return FALSE;
    }
}

# 192 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$lockCurrentBuffer(void )
#line 192
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufOneLocked = 1;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufZeroLocked = 1;
    }
}

#line 188
static inline bool /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$isCurrentBufferLocked(void )
#line 188
{
  return /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which ? /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufZeroLocked : /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufOneLocked;
}

#line 215
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket(void )
#line 215
{
  error_t result = SUCCESS;

  /* atomic removed: atomic calls only */
#line 217
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$isCurrentBufferLocked()) {


        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$lockCurrentBuffer();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_BEGIN;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex = 0;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType = TOS_SERIAL_UNKNOWN_ID;
      }
    else {
        result = EBUSY;
      }
  }
  return result;
}

# 51 "/opt/tinyos-2.1.0/tos/lib/serial/ReceiveBytePacket.nc"
inline static error_t SerialP$ReceiveBytePacket$startPacket(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$startPacket();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 309 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static __inline uint16_t SerialP$rx_current_crc(void )
#line 309
{
  uint16_t crc;
  uint8_t tmp = SerialP$rxBuf.writePtr;

#line 312
  tmp = tmp == 0 ? SerialP$RX_DATA_BUFFER_SIZE : tmp - 1;
  crc = SerialP$rxBuf.buf[tmp] & 0x00ff;
  crc = (crc << 8) & 0xFF00;
  tmp = tmp == 0 ? SerialP$RX_DATA_BUFFER_SIZE : tmp - 1;
  crc |= SerialP$rxBuf.buf[tmp] & 0x00FF;
  return crc;
}

# 69 "/opt/tinyos-2.1.0/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP$ReceiveBytePacket$endPacket(error_t result){
#line 69
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(result);
#line 69
}
#line 69
# 210 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBufferSwap(void )
#line 210
{
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which ? 0 : 1;
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer = (uint8_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messagePtrs[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which];
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 232 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static __inline bool SerialP$ack_queue_is_full(void )
#line 232
{
  uint8_t tmp;
#line 233
  uint8_t tmp2;

  /* atomic removed: atomic calls only */
#line 234
  {
    tmp = SerialP$ackQ.writePtr;
    tmp2 = SerialP$ackQ.readPtr;
  }
  if (++tmp > SerialP$ACK_QUEUE_SIZE) {
#line 238
    tmp = 0;
    }
#line 239
  return tmp == tmp2;
}







static __inline void SerialP$ack_queue_push(uint8_t token)
#line 248
{
  if (!SerialP$ack_queue_is_full()) {
      /* atomic removed: atomic calls only */
#line 250
      {
        SerialP$ackQ.buf[SerialP$ackQ.writePtr] = token;
        if (++ SerialP$ackQ.writePtr > SerialP$ACK_QUEUE_SIZE) {
#line 252
          SerialP$ackQ.writePtr = 0;
          }
      }
#line 254
      SerialP$MaybeScheduleTx();
    }
}

# 40 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$offset(void )
#line 40
{
  return (uint8_t )(sizeof(message_header_t ) - sizeof(serial_header_t ));
}

# 344 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(uart_id_t id)
#line 344
{
  return 0;
}

# 15 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(uart_id_t arg_0x408e5b30){
#line 15
  unsigned char __nesc_result;
#line 15

#line 15
  switch (arg_0x408e5b30) {
#line 15
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 15
      __nesc_result = SerialPacketInfoActiveMessageP$Info$offset();
#line 15
      break;
#line 15
    default:
#line 15
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$offset(arg_0x408e5b30);
#line 15
      break;
#line 15
    }
#line 15

#line 15
  return __nesc_result;
#line 15
}
#line 15
# 233 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(uint8_t b)
#line 233
{
  /* atomic removed: atomic calls only */
#line 234
  {
    switch (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state) {
        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_BEGIN: 
          /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_DATA;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(b);
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType = b;
        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_DATA: 
          if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex < sizeof(message_t )) {
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer[/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex] = b;
              /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex++;
            }
          else {
            }




        break;

        case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_IDLE: 
          default: 
#line 255
            ;
      }
  }
}

# 58 "/opt/tinyos-2.1.0/tos/lib/serial/ReceiveBytePacket.nc"
inline static void SerialP$ReceiveBytePacket$byteReceived(uint8_t data){
#line 58
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$byteReceived(data);
#line 58
}
#line 58
# 299 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static __inline uint8_t SerialP$rx_buffer_top(void )
#line 299
{
  uint8_t tmp = SerialP$rxBuf.buf[SerialP$rxBuf.readPtr];

#line 301
  return tmp;
}

#line 303
static __inline uint8_t SerialP$rx_buffer_pop(void )
#line 303
{
  uint8_t tmp = SerialP$rxBuf.buf[SerialP$rxBuf.readPtr];

#line 305
  if (++ SerialP$rxBuf.readPtr > SerialP$RX_DATA_BUFFER_SIZE) {
#line 305
    SerialP$rxBuf.readPtr = 0;
    }
#line 306
  return tmp;
}

#line 295
static __inline void SerialP$rx_buffer_push(uint8_t data)
#line 295
{
  SerialP$rxBuf.buf[SerialP$rxBuf.writePtr] = data;
  if (++ SerialP$rxBuf.writePtr > SerialP$RX_DATA_BUFFER_SIZE) {
#line 297
    SerialP$rxBuf.writePtr = 0;
    }
}

# 55 "/opt/tinyos-2.1.0/tos/lib/serial/HdlcTranslateC.nc"
static inline void HdlcTranslateC$SerialFrameComm$resetReceive(void )
#line 55
{
  HdlcTranslateC$state.receiveEscape = 0;
}

# 68 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFrameComm.nc"
inline static void SerialP$SerialFrameComm$resetReceive(void ){
#line 68
  HdlcTranslateC$SerialFrameComm$resetReceive();
#line 68
}
#line 68
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$stopDoneTask$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(SerialP$stopDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 228 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$measureCh0Done(error_t error, uint16_t val)
#line 228
{
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 112 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh0Done(error_t error, uint16_t val)
#line 112
{
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READ_BB;
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_error = error;
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_val = val;
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$postTask();
}

# 118 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$measureCh0Done(uint8_t id, error_t error, uint16_t val)
#line 118
{
#line 118
  return;
}

# 44 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$measureCh0Done(uint8_t arg_0x407fc730, error_t error, uint16_t val){
#line 44
  switch (arg_0x407fc730) {
#line 44
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 44
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh0Done(error, val);
#line 44
      break;
#line 44
    default:
#line 44
      Tsl2561InternalP$HplTSL256x$default$measureCh0Done(arg_0x407fc730, error, val);
#line 44
      break;
#line 44
    }
#line 44
}
#line 44
# 88 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$measureCh0Done(error_t result, uint16_t val)
#line 88
{
  Tsl2561InternalP$HplTSL256x$measureCh0Done(Tsl2561InternalP$currentId, result, val);
}

# 44 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh0Done(error_t error, uint16_t val){
#line 44
  Tsl2561InternalP$ToHPLC$measureCh0Done(error, val);
#line 44
  HalTsl2561ControlP$HplTSL256x$measureCh0Done(error, val);
#line 44
}
#line 44
# 229 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$measureCh1Done(error_t error, uint16_t val)
#line 229
{
}

# 119 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh1Done(error_t error, uint16_t val)
#line 119
{
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READ_IR;
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_error = error;
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_val = val;
  /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$postTask();
}

# 119 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$measureCh1Done(uint8_t id, error_t error, uint16_t val)
#line 119
{
#line 119
  return;
}

# 47 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$measureCh1Done(uint8_t arg_0x407fc730, error_t error, uint16_t val){
#line 47
  switch (arg_0x407fc730) {
#line 47
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 47
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh1Done(error, val);
#line 47
      break;
#line 47
    default:
#line 47
      Tsl2561InternalP$HplTSL256x$default$measureCh1Done(arg_0x407fc730, error, val);
#line 47
      break;
#line 47
    }
#line 47
}
#line 47
# 91 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$measureCh1Done(error_t result, uint16_t val)
#line 91
{
  Tsl2561InternalP$HplTSL256x$measureCh1Done(Tsl2561InternalP$currentId, result, val);
}

# 47 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh1Done(error_t error, uint16_t val){
#line 47
  Tsl2561InternalP$ToHPLC$measureCh1Done(error, val);
#line 47
  HalTsl2561ControlP$HplTSL256x$measureCh1Done(error, val);
#line 47
}
#line 47
# 224 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$getIDDone(error_t error, uint8_t idval)
#line 224
{
}

# 131 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$getIDDone(error_t error, uint8_t idval)
#line 131
{
}

# 125 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$getIDDone(uint8_t id, error_t error, uint8_t idval)
#line 125
{
#line 125
  return;
}

# 65 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$getIDDone(uint8_t arg_0x407fc730, error_t error, uint8_t idval){
#line 65
  switch (arg_0x407fc730) {
#line 65
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 65
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$getIDDone(error, idval);
#line 65
      break;
#line 65
    default:
#line 65
      Tsl2561InternalP$HplTSL256x$default$getIDDone(arg_0x407fc730, error, idval);
#line 65
      break;
#line 65
    }
#line 65
}
#line 65
# 109 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$getIDDone(error_t error, uint8_t idval)
#line 109
{
  Tsl2561InternalP$HplTSL256x$getIDDone(Tsl2561InternalP$currentId, error, idval);
}

# 65 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$getIDDone(error_t error, uint8_t idval){
#line 65
  Tsl2561InternalP$ToHPLC$getIDDone(error, idval);
#line 65
  HalTsl2561ControlP$HplTSL256x$getIDDone(error, idval);
#line 65
}
#line 65
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 65 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
inline static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$read(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t * data){
#line 65
  unsigned char __nesc_result;
#line 65

#line 65
  __nesc_result = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$read(flags, addr, length, data);
#line 65

#line 65
  return __nesc_result;
#line 65
}
#line 65
# 227 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$setCONTROLDone(error_t error)
#line 227
{
}

# 126 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setCONTROLDone(error_t error)
#line 126
{
}

# 120 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$setCONTROLDone(uint8_t id, error_t error)
#line 120
{
#line 120
  return;
}

# 50 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$setCONTROLDone(uint8_t arg_0x407fc730, error_t error){
#line 50
  switch (arg_0x407fc730) {
#line 50
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 50
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setCONTROLDone(error);
#line 50
      break;
#line 50
    default:
#line 50
      Tsl2561InternalP$HplTSL256x$default$setCONTROLDone(arg_0x407fc730, error);
#line 50
      break;
#line 50
    }
#line 50
}
#line 50
# 94 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$setCONTROLDone(error_t error)
#line 94
{
  Tsl2561InternalP$HplTSL256x$setCONTROLDone(Tsl2561InternalP$currentId, error);
}

# 50 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setCONTROLDone(error_t error){
#line 50
  Tsl2561InternalP$ToHPLC$setCONTROLDone(error);
#line 50
  HalTsl2561ControlP$HplTSL256x$setCONTROLDone(error);
#line 50
}
#line 50
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t HalTsl2561ControlP$complete_Task$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(HalTsl2561ControlP$complete_Task);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 205 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$setTIMINGDone(error_t error)
#line 205
{
  HalTsl2561ControlP$clientResult = error;
  HalTsl2561ControlP$complete_Task$postTask();
}

# 127 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTIMINGDone(error_t error)
#line 127
{
}

# 121 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$setTIMINGDone(uint8_t id, error_t error)
#line 121
{
#line 121
  return;
}

# 53 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$setTIMINGDone(uint8_t arg_0x407fc730, error_t error){
#line 53
  switch (arg_0x407fc730) {
#line 53
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 53
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTIMINGDone(error);
#line 53
      break;
#line 53
    default:
#line 53
      Tsl2561InternalP$HplTSL256x$default$setTIMINGDone(arg_0x407fc730, error);
#line 53
      break;
#line 53
    }
#line 53
}
#line 53
# 97 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$setTIMINGDone(error_t error)
#line 97
{
  Tsl2561InternalP$HplTSL256x$setTIMINGDone(Tsl2561InternalP$currentId, error);
}

# 53 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTIMINGDone(error_t error){
#line 53
  Tsl2561InternalP$ToHPLC$setTIMINGDone(error);
#line 53
  HalTsl2561ControlP$HplTSL256x$setTIMINGDone(error);
#line 53
}
#line 53
# 209 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$setINTERRUPTDone(error_t error)
#line 209
{
  HalTsl2561ControlP$clientResult = error;
  HalTsl2561ControlP$complete_Task$postTask();
}

# 130 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setINTERRUPTDone(error_t error)
#line 130
{
}

# 124 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$setINTERRUPTDone(uint8_t id, error_t error)
#line 124
{
#line 124
  return;
}

# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$setINTERRUPTDone(uint8_t arg_0x407fc730, error_t error){
#line 62
  switch (arg_0x407fc730) {
#line 62
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 62
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setINTERRUPTDone(error);
#line 62
      break;
#line 62
    default:
#line 62
      Tsl2561InternalP$HplTSL256x$default$setINTERRUPTDone(arg_0x407fc730, error);
#line 62
      break;
#line 62
    }
#line 62
}
#line 62
# 106 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$setINTERRUPTDone(error_t error)
#line 106
{
  Tsl2561InternalP$HplTSL256x$setINTERRUPTDone(Tsl2561InternalP$currentId, error);
}

# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setINTERRUPTDone(error_t error){
#line 62
  Tsl2561InternalP$ToHPLC$setINTERRUPTDone(error);
#line 62
  HalTsl2561ControlP$HplTSL256x$setINTERRUPTDone(error);
#line 62
}
#line 62
# 217 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$setTHRESHHIGHDone(error_t error)
#line 217
{
  HalTsl2561ControlP$clientResult = error;
  HalTsl2561ControlP$complete_Task$postTask();
}

# 129 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHHIGHDone(error_t error)
#line 129
{
}

# 123 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$setTHRESHHIGHDone(uint8_t id, error_t error)
#line 123
{
#line 123
  return;
}

# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$setTHRESHHIGHDone(uint8_t arg_0x407fc730, error_t error){
#line 59
  switch (arg_0x407fc730) {
#line 59
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 59
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHHIGHDone(error);
#line 59
      break;
#line 59
    default:
#line 59
      Tsl2561InternalP$HplTSL256x$default$setTHRESHHIGHDone(arg_0x407fc730, error);
#line 59
      break;
#line 59
    }
#line 59
}
#line 59
# 103 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$setTHRESHHIGHDone(error_t error)
#line 103
{
  Tsl2561InternalP$HplTSL256x$setTHRESHHIGHDone(Tsl2561InternalP$currentId, error);
}

# 59 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTHRESHHIGHDone(error_t error){
#line 59
  Tsl2561InternalP$ToHPLC$setTHRESHHIGHDone(error);
#line 59
  HalTsl2561ControlP$HplTSL256x$setTHRESHHIGHDone(error);
#line 59
}
#line 59
# 213 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$setTHRESHLOWDone(error_t error)
#line 213
{
  HalTsl2561ControlP$clientResult = error;
  HalTsl2561ControlP$complete_Task$postTask();
}

# 128 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHLOWDone(error_t error)
#line 128
{
}

# 122 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$setTHRESHLOWDone(uint8_t id, error_t error)
#line 122
{
#line 122
  return;
}

# 56 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$setTHRESHLOWDone(uint8_t arg_0x407fc730, error_t error){
#line 56
  switch (arg_0x407fc730) {
#line 56
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 56
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$setTHRESHLOWDone(error);
#line 56
      break;
#line 56
    default:
#line 56
      Tsl2561InternalP$HplTSL256x$default$setTHRESHLOWDone(arg_0x407fc730, error);
#line 56
      break;
#line 56
    }
#line 56
}
#line 56
# 100 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$setTHRESHLOWDone(error_t error)
#line 100
{
  Tsl2561InternalP$HplTSL256x$setTHRESHLOWDone(Tsl2561InternalP$currentId, error);
}

# 56 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTHRESHLOWDone(error_t error){
#line 56
  Tsl2561InternalP$ToHPLC$setTHRESHLOWDone(error);
#line 56
  HalTsl2561ControlP$HplTSL256x$setTHRESHLOWDone(error);
#line 56
}
#line 56
# 116 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$InterruptAlert$fired(void )
#line 116
{
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t HalTsl2561ControlP$complete_Alert$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(HalTsl2561ControlP$complete_Alert);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 221 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$HplTSL256x$alertThreshold(void )
#line 221
{
#line 221
  HalTsl2561ControlP$complete_Alert$postTask();
}

# 132 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$alertThreshold(void )
#line 132
{
}

# 126 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$HplTSL256x$default$alertThreshold(uint8_t id)
#line 126
{
#line 126
  return;
}

# 67 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void Tsl2561InternalP$HplTSL256x$alertThreshold(uint8_t arg_0x407fc730){
#line 67
  switch (arg_0x407fc730) {
#line 67
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID:
#line 67
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$alertThreshold();
#line 67
      break;
#line 67
    default:
#line 67
      Tsl2561InternalP$HplTSL256x$default$alertThreshold(arg_0x407fc730);
#line 67
      break;
#line 67
    }
#line 67
}
#line 67
# 112 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline void Tsl2561InternalP$ToHPLC$alertThreshold(void )
#line 112
{
  Tsl2561InternalP$HplTSL256x$alertThreshold(Tsl2561InternalP$currentId);
}

# 67 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$alertThreshold(void ){
#line 67
  Tsl2561InternalP$ToHPLC$alertThreshold();
#line 67
  HalTsl2561ControlP$HplTSL256x$alertThreshold();
#line 67
}
#line 67
# 297 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptAlert$fired(void )
#line 297
{


  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$alertThreshold();






  return;
}

# 162 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$GpioInterrupt$default$fired(uint8_t pin)
#line 162
{
  return;
}

# 57 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
inline static void HalPXA27xGeneralIOM$GpioInterrupt$fired(uint8_t arg_0x40677670){
#line 57
  switch (arg_0x40677670) {
#line 57
    case 99:
#line 57
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptAlert$fired();
#line 57
      Tsl2561InternalP$InterruptAlert$fired();
#line 57
      break;
#line 57
    default:
#line 57
      HalPXA27xGeneralIOM$GpioInterrupt$default$fired(arg_0x40677670);
#line 57
      break;
#line 57
    }
#line 57
}
#line 57
# 158 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(uint8_t pin)
#line 158
{
  return;
}

# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGpioInterrupt.nc"
inline static void HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$fired(uint8_t arg_0x40678c20){
#line 65
    HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$default$fired(arg_0x40678c20);
#line 65
}
#line 65
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline bool HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(uint8_t pin)
{
  bool flag;

#line 153
  flag = (*(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00048 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00148) & (1 << (pin & 0x1f))) != 0;
  *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00048 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00148) = 1 << (pin & 0x1f);
  return flag;
}

# 124 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static bool HalPXA27xGeneralIOM$HplPXA27xGPIOPin$clearGEDRbit(uint8_t arg_0x40675010){
#line 124
  unsigned char __nesc_result;
#line 124

#line 124
  __nesc_result = HplPXA27xGPIOM$HplPXA27xGPIOPin$clearGEDRbit(arg_0x40675010);
#line 124

#line 124
  return __nesc_result;
#line 124
}
#line 124
# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(uint8_t pin)
#line 150
{
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$clearGEDRbit(pin);
  HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$fired(pin);
  HalPXA27xGeneralIOM$GpioInterrupt$fired(pin);
  return;
}

# 56 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/IM2InitSerialP.nc"
static inline void IM2InitSerialP$TXD$interruptGPIOPin(void )
#line 56
{
#line 56
  return;
}

#line 57
static inline void IM2InitSerialP$RXD$interruptGPIOPin(void )
#line 57
{
#line 57
  return;
}

# 315 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$interruptGPIOPin(void )
#line 315
{
}

#line 314
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$interruptGPIOPin(void )
#line 314
{
}

# 186 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline void HplPXA27xOSTimerM$PXA27xWD$enableWatchdog(void )
{
  * (volatile uint32_t *)0x40A00018 = 1 << 0;
}

# 45 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerWatchdog.nc"
inline static void PlatformP$PXA27xWD$enableWatchdog(void ){
#line 45
  HplPXA27xOSTimerM$PXA27xWD$enableWatchdog();
#line 45
}
#line 45
# 64 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t PlatformP$OST0M3$getOSCR(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSCR(3);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64







inline static void PlatformP$OST0M3$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(3, val);
#line 71
}
#line 71
# 117 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/PlatformP.nc"
static inline void PlatformP$PlatformReset$reset(void )
#line 117
{
  PlatformP$OST0M3$setOSMR(PlatformP$OST0M3$getOSCR() + 1000);
  PlatformP$PXA27xWD$enableWatchdog();
  while (1) ;
  return;
}

# 32 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/PlatformReset.nc"
inline static void PMICM$PlatformReset$reset(void ){
#line 32
  PlatformP$PlatformReset$reset();
#line 32
}
#line 32
# 271 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline void PMICM$PMICGPIO$interruptGPIOPin(void )
#line 271
{
  uint8_t events[3];
  bool localGotReset;



  PMICM$readPMIC(0x01, events, 3);

  if (events[0] & 0x1) {
      /* atomic removed: atomic calls only */
#line 280
      {
        localGotReset = PMICM$gotReset;
      }
      if (localGotReset == TRUE) {
          PMICM$PlatformReset$reset();
        }
      else {
          /* atomic removed: atomic calls only */
#line 287
          {
            PMICM$gotReset = TRUE;
          }
        }
    }
  else {
    }
}

# 42 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void PMICM$PI2C$setIDBR(uint32_t val){
#line 42
  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(val);
#line 42
}
#line 42

inline static uint32_t PMICM$PI2C$getIDBR(void ){
#line 43
  unsigned int __nesc_result;
#line 43

#line 43
  __nesc_result = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 112 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool PlatformP$OST0M3$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(3);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112







inline static void PlatformP$OST0M3$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(3, flag);
#line 119
}
#line 119
# 124 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/PlatformP.nc"
static inline void PlatformP$OST0M3$fired(void )
{
  PlatformP$OST0M3$setOIERbit(FALSE);
  PlatformP$OST0M3$clearOSSRbit();
  return;
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 70 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired(void )
{
#line 71
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$postTask();
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$fired();
#line 67
}
#line 67
# 119 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(4, flag);
#line 119
}
#line 119
#line 112
inline static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(4);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 160 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired(void )
#line 160
{
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$clearOSSRbit();
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(FALSE);
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired();
  return;
}

# 104 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow(void )
#line 104
{
  return;
}

# 71 "/opt/tinyos-2.1.0/tos/lib/timer/Counter.nc"
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$overflow(void ){
#line 71
  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$default$overflow();
#line 71
}
#line 71
# 112 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit(void ){
#line 112
  unsigned char __nesc_result;
#line 112

#line 112
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(5);
#line 112

#line 112
  return __nesc_result;
#line 112
}
#line 112
# 91 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired(void )
#line 91
{
  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit();
  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Counter$overflow();
  return;
}

# 113 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static inline void SchedulerBasicP$Scheduler$init(void )
{
  /* atomic removed: atomic calls only */
  {
    memset((void *)SchedulerBasicP$m_next, SchedulerBasicP$NO_TASK, sizeof SchedulerBasicP$m_next);
    SchedulerBasicP$m_head = SchedulerBasicP$NO_TASK;
    SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
  }
}

# 46 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$init(void ){
#line 46
  SchedulerBasicP$Scheduler$init();
#line 46
}
#line 46
# 58 "/opt/tinyos-2.1.0/tos/types/TinyError.h"
static inline  error_t ecombine(error_t r1, error_t r2)




{
  return r1 == r2 ? r1 : FAIL;
}

# 29 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$set(void ){
#line 29
  HalPXA27xGeneralIOM$GeneralIO$set(105);
#line 29
}
#line 29
inline static void LedsP$Led1$set(void ){
#line 29
  HalPXA27xGeneralIOM$GeneralIO$set(104);
#line 29
}
#line 29
inline static void LedsP$Led0$set(void ){
#line 29
  HalPXA27xGeneralIOM$GeneralIO$set(103);
#line 29
}
#line 29
# 52 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(uint8_t arg_0x40675010, bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(arg_0x40675010, dir);
#line 52
}
#line 52
# 94 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$GeneralIO$makeOutput(uint8_t pin)
#line 94
{
  /* atomic removed: atomic calls only */
#line 95
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(pin, TRUE);
  return;
}

# 35 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$makeOutput(void ){
#line 35
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(105);
#line 35
}
#line 35
inline static void LedsP$Led1$makeOutput(void ){
#line 35
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(104);
#line 35
}
#line 35
inline static void LedsP$Led0$makeOutput(void ){
#line 35
  HalPXA27xGeneralIOM$GeneralIO$makeOutput(103);
#line 35
}
#line 35
# 45 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline error_t LedsP$Init$init(void )
#line 45
{
  /* atomic removed: atomic calls only */
#line 46
  {
    ;
    LedsP$Led0$makeOutput();
    LedsP$Led1$makeOutput();
    LedsP$Led2$makeOutput();
    LedsP$Led0$set();
    LedsP$Led1$set();
    LedsP$Led2$set();
  }
  return SUCCESS;
}

# 122 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setMCR(uint32_t val)
#line 122
{
#line 122
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x10) = val;
}

# 60 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setMCR(uint32_t val){
#line 60
  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setMCR(val);
#line 60
}
#line 60
# 120 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setLCR(uint32_t val)
#line 120
{
#line 120
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x0C) = val;
}

# 57 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setLCR(uint32_t val){
#line 57
  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setLCR(val);
#line 57
}
#line 57
# 104 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLH(uint32_t val)
#line 104
{
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x0C) |= 1 << 7;
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x04) = val;
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x0C) &= ~(1 << 7);
}

# 47 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setDLH(uint32_t val){
#line 47
  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLH(val);
#line 47
}
#line 47
# 92 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLL(uint32_t val)
#line 92
{
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x0C) |= 1 << 7;
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0) = val;
  * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x0C) &= ~(1 << 7);
}

# 44 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUART.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setDLL(uint32_t val){
#line 44
  /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UART$setDLL(val);
#line 44
}
#line 44
# 360 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialCntl$configPort(uint32_t baudrate, 
uint8_t databits, 
uart_parity_t parity, 
uint8_t stopbits, 
bool flow_cntl)
#line 364
{
  uint32_t uiDivisor;
  uint32_t valLCR = 0;
  uint32_t valMCR = 1 << 3;

  uiDivisor = 921600 / baudrate;


  if (uiDivisor & 0xFFFF0000 || uiDivisor == 0) {
      return EINVAL;
    }

  if (databits > 8 || databits < 5) {
      return EINVAL;
    }
  valLCR |= (databits - 5) << 0;

  switch (parity) {
      case EVEN: 
        valLCR |= 1 << 4;

      case ODD: 
        valLCR |= 1 << 3;
      break;
      case NONE: 
        break;
      default: 
        return EINVAL;
      break;
    }

  if (stopbits > 2 || stopbits < 1) {
      return EINVAL;
    }
  else {
#line 398
    if (stopbits == 2) {
        valLCR |= 1 << 2;
      }
    }
  if (flow_cntl) {
      valMCR |= 1 << 5;
    }
  /* atomic removed: atomic calls only */
  {
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setDLL(uiDivisor & 0xFF);
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setDLH((uiDivisor >> 8) & 0xFF);
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setLCR(valLCR);
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setMCR(valMCR);
  }

  return SUCCESS;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDALGNbit(bool flag){
#line 78
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDALGNbit(3, flag);
#line 78
}
#line 78
inline static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDALGNbit(bool flag){
#line 78
  HplPXA27xDMAM$HplPXA27xDMAChnl$setDALGNbit(2, flag);
#line 78
}
#line 78
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfoC.nc"
static inline uint8_t /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getMapIndex(void )
#line 54
{
  return 19;
}

# 63 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfo.nc"
inline static uint8_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTRxDMAInfo$getMapIndex(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*PlatformSerialC.DMAInfoRx*/HplPXA27xDMAInfoC$0$HplPXA27xDMAInfo$getMapIndex();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 75 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline error_t HplPXA27xDMAM$HplPXA27xDMAChnl$setMap(uint8_t chnl, uint8_t dev)
#line 75
{
  HplPXA27xDMAM$HplPXA27xDMACntl$setDRCMR(dev, (1 << 7) | (chnl & 0x1f));
  return SUCCESS;
}

# 77 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setMap(uint8_t dev){
#line 77
  unsigned char __nesc_result;
#line 77

#line 77
  __nesc_result = HplPXA27xDMAM$HplPXA27xDMAChnl$setMap(3, dev);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfoC.nc"
static inline uint8_t /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getMapIndex(void )
#line 54
{
  return 20;
}

# 63 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAInfo.nc"
inline static uint8_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTTxDMAInfo$getMapIndex(void ){
#line 63
  unsigned char __nesc_result;
#line 63

#line 63
  __nesc_result = /*PlatformSerialC.DMAInfoTx*/HplPXA27xDMAInfoC$1$HplPXA27xDMAInfo$getMapIndex();
#line 63

#line 63
  return __nesc_result;
#line 63
}
#line 63
# 77 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAChnl.nc"
inline static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setMap(uint8_t dev){
#line 77
  unsigned char __nesc_result;
#line 77

#line 77
  __nesc_result = HplPXA27xDMAM$HplPXA27xDMAChnl$setMap(2, dev);
#line 77

#line 77
  return __nesc_result;
#line 77
}
#line 77
# 193 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline void HplPXA27xInterruptM$PXA27xIrq$enable(uint8_t id)
{
  HplPXA27xInterruptM$enable(id);
  return;
}

# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(20);
#line 65
}
#line 65
# 188 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static inline error_t HplPXA27xInterruptM$PXA27xIrq$allocate(uint8_t id)
{
  return HplPXA27xInterruptM$allocate(id, FALSE, TOSH_IRP_TABLE[id]);
}

# 60 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static error_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(20);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 57 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HplPXA27xUARTP.nc"
static inline error_t /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$Init$init(void )
#line 57
{
  bool isInited;

  /* atomic removed: atomic calls only */
#line 60
  {
    isInited = /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$m_fInit;
    /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$m_fInit = TRUE;
  }

  if (!isInited) {
      switch (1081081856U) {
          case 0x40100000: 
            * (volatile uint32_t *)0x41300004 |= 1 << 6;
          break;
          case 0x40200000: 
            * (volatile uint32_t *)0x41300004 |= 1 << 7;
          break;
          case 0x40700000: 
            * (volatile uint32_t *)0x41300004 |= 1 << 5;
          break;
          default: 
            break;
        }
      /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$allocate();
      /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$UARTIrq$enable();
      * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x0C) |= 1 << 7;
      * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0) = 0x04;
      * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x04) = 0x00;
      * (volatile uint32_t *)((uint32_t )1081081856U + (uint32_t )0x0C) &= ~(1 << 7);
    }

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*HplPXA27xSTUARTC.HplPXA27xUARTP*/HplPXA27xUARTP$0$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 106 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$Init$init(void )
#line 106
{
  error_t error = SUCCESS;

  /* atomic removed: atomic calls only */
#line 109
  {
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTInit$init();
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentBuf = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf = (void *)0;
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamSendIF = FALSE;
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamRcvIF = FALSE;
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbRcvByteEvtEnabled = TRUE;
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow = (1 << 0) | (0 << 6);
  }
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setMap(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTTxDMAInfo$getMapIndex());
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setMap(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTRxDMAInfo$getMapIndex());
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDALGNbit(TRUE);
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$RxDMA$setDALGNbit(TRUE);

  error = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialCntl$configPort(115200, 8, NONE, 1, FALSE);
  /* atomic removed: atomic calls only */
  {
#line 124
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setFCR(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow);
  }
#line 125
  return error;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL3$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$Init$init();
#line 51
  __nesc_result = ecombine(__nesc_result, LedsP$Init$init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 171 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline void PMICM$startLDOs(void )
#line 171
{

  uint8_t oldVal;
#line 173
  uint8_t newVal;



  PMICM$readPMIC(0x17, &oldVal, 1);
  newVal = (oldVal | 0x2) | 0x4;
  PMICM$writePMIC(0x17, newVal);

  PMICM$readPMIC(0x98, &oldVal, 1);
  newVal = (oldVal | 0x4) | 0x8;
  PMICM$writePMIC(0x98, newVal);




  PMICM$readPMIC(0x97, &oldVal, 1);
  newVal = oldVal | 0x20;
  PMICM$writePMIC(0x97, newVal);
}

#line 300
static inline error_t PMICM$PMIC$setCoreVoltage(uint8_t trimValue)
#line 300
{
  PMICM$writePMIC(0x15, (trimValue & 0x1f) | 0x80);
  return SUCCESS;
}

# 101 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void PMICM$PMICGPIO$setGFERbit(bool flag){
#line 101
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(1, flag);
#line 101
}
#line 101
#line 52
inline static void PMICM$PMICGPIO$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(1, dir);
#line 52
}
#line 52
#line 134
inline static void PMICM$PMICGPIO$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(1, func);
#line 134
}
#line 134
# 46 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static uint32_t PMICM$PI2C$getICR(void ){
#line 46
  unsigned int __nesc_result;
#line 46

#line 46
  __nesc_result = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
#line 45
inline static void PMICM$PI2C$setICR(uint32_t val){
#line 45
  /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(val);
#line 45
}
#line 45
# 219 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline error_t PMICM$Init$init(void )
#line 219
{
  uint8_t val[3];

#line 221
  * (volatile uint32_t *)0x40F0001C |= 1 << 6;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | ((1 << 6) | (1 << 5)));
  /* atomic removed: atomic calls only */
#line 223
  {
    PMICM$gotReset = FALSE;
  }

  PMICM$PMICGPIO$setGAFRpin(0);
  PMICM$PMICGPIO$setGPDRbit(FALSE);
  PMICM$PMICGPIO$setGFERbit(TRUE);




  PMICM$writePMIC(0x08, (
  0x80 | 0x8) | 0x4);


  PMICM$writePMIC(0x05, ~0x1);
  PMICM$writePMIC(0x06, 0xFF);
  PMICM$writePMIC(0x07, 0xFF);


  PMICM$readPMIC(0x01, val, 3);




  PMICM$PMIC$setCoreVoltage(0x4);



  PMICM$startLDOs();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2C.nc"
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISAR(uint32_t val){
#line 51
  /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISAR(val);
#line 51
}
#line 51
# 52 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(118, dir);
#line 52
}
#line 52
#line 134
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(118, func);
#line 134
}
#line 134
#line 52
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(117, dir);
#line 52
}
#line 52
#line 134
inline static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(117, func);
#line 134
}
#line 134
# 183 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static inline error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$Init$init(void )
#line 183
{
  /* atomic removed: atomic calls only */
#line 184
  {
    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags = 1 ? (((1 << 15) | (1 << 10)) | (1 << 6)) | (1 << 5) : ((1 << 10) | (1 << 6)) | (1 << 5);

    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$setGAFRpin(1);
    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$setGPDRbit(TRUE);
    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$setGAFRpin(1);
    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$setGPDRbit(TRUE);

    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_IDLE;
    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISAR(0);
    /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR((/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (1 << 8)) | (1 << 9));
  }
  return SUCCESS;
}

# 52 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void IM2InitSerialP$RXD$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(46, dir);
#line 52
}
#line 52
#line 134
inline static void IM2InitSerialP$RXD$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(46, func);
#line 134
}
#line 134
#line 52
inline static void IM2InitSerialP$TXD$setGPDRbit(bool dir){
#line 52
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(47, dir);
#line 52
}
#line 52
#line 134
inline static void IM2InitSerialP$TXD$setGAFRpin(uint8_t func){
#line 134
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(47, func);
#line 134
}
#line 134
# 47 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/IM2InitSerialP.nc"
static inline error_t IM2InitSerialP$Init$init(void )
#line 47
{
  IM2InitSerialP$TXD$setGAFRpin(1);
  IM2InitSerialP$TXD$setGPDRbit(TRUE);
  IM2InitSerialP$RXD$setGAFRpin(2);
  IM2InitSerialP$RXD$setGPDRbit(FALSE);

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL2$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = IM2InitSerialP$Init$init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$Init$init());
#line 51
  __nesc_result = ecombine(__nesc_result, PMICM$Init$init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(6);
#line 65
}
#line 65
#line 60
inline static error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(6);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline error_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init(void )
#line 54
{
  bool isInited;

  /* atomic removed: atomic calls only */
#line 57
  {
    isInited = /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$m_fInit;
    /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$m_fInit = TRUE;
  }

  if (!isInited) {
      switch (1) {
          case 0: 
            * (volatile uint32_t *)0x41300004 |= 1 << 14;
          * (volatile uint32_t *)0x40301690 = 0;
          break;
          case 1: 
            * (volatile uint32_t *)0x41300004 |= 1 << 15;
          * (volatile uint32_t *)0x40F00190 = 0;
          break;
          default: 
            break;
        }
      /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$allocate();
      /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2CIrq$enable();
    }

  return SUCCESS;
}

# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xGPIOM$GPIOIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(10);
#line 65
}
#line 65
inline static void HplPXA27xGPIOM$GPIOIrq1$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(9);
#line 65
}
#line 65
inline static void HplPXA27xGPIOM$GPIOIrq0$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(8);
#line 65
}
#line 65
#line 60
inline static error_t HplPXA27xGPIOM$GPIOIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(10);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xGPIOM$GPIOIrq1$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(9);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xGPIOM$GPIOIrq0$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(8);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 60 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline error_t HplPXA27xGPIOM$Init$init(void )
{
  bool isInited;

  /* atomic removed: atomic calls only */
#line 64
  {
    isInited = HplPXA27xGPIOM$gfInitialized;
    HplPXA27xGPIOM$gfInitialized = TRUE;
  }

  if (!isInited) {
      HplPXA27xGPIOM$GPIOIrq0$allocate();
      HplPXA27xGPIOM$GPIOIrq1$allocate();
      HplPXA27xGPIOM$GPIOIrq$allocate();
      HplPXA27xGPIOM$GPIOIrq0$enable();
      HplPXA27xGPIOM$GPIOIrq1$enable();
      HplPXA27xGPIOM$GPIOIrq$enable();
    }
  return SUCCESS;
}

# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(18);
#line 65
}
#line 65
#line 60
inline static error_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(18);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static inline error_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$Init$init(void )
#line 54
{
  bool isInited;

  /* atomic removed: atomic calls only */
#line 57
  {
    isInited = /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$m_fInit;
    /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$m_fInit = TRUE;
  }

  if (!isInited) {
      switch (0) {
          case 0: 
            * (volatile uint32_t *)0x41300004 |= 1 << 14;
          * (volatile uint32_t *)0x40301690 = 0;
          break;
          case 1: 
            * (volatile uint32_t *)0x41300004 |= 1 << 15;
          * (volatile uint32_t *)0x40F00190 = 0;
          break;
          default: 
            break;
        }
      /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$allocate();
      /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2CIrq$enable();
    }

  return SUCCESS;
}

# 65 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static void HplPXA27xDMAM$DMAIrq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(25);
#line 65
}
#line 65
#line 60
inline static error_t HplPXA27xDMAM$DMAIrq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(25);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 50 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static inline error_t HplPXA27xDMAM$Init$init(void )
#line 50
{
  HplPXA27xDMAM$DMAIrq$allocate();
  HplPXA27xDMAM$DMAIrq$enable();
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL1$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = HplPXA27xDMAM$Init$init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$Init$init());
#line 51
  __nesc_result = ecombine(__nesc_result, HplPXA27xGPIOM$Init$init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$Init$init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 119 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOIERbit(bool flag){
#line 119
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(5, flag);
#line 119
}
#line 119
#line 57
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSCR(uint32_t val){
#line 57
  HplPXA27xOSTimerM$PXA27xOST$setOSCR(5, val);
#line 57
}
#line 57
#line 71
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(5, val);
#line 71
}
#line 71
#line 85
inline static void /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOMCR(uint32_t val){
#line 85
  HplPXA27xOSTimerM$PXA27xOST$setOMCR(5, val);
#line 85
}
#line 85
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = HplPXA27xOSTimerM$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 56 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xCounterM.nc"
static inline error_t /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init(void )
#line 56
{

  /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTInit$init();
  /* atomic removed: atomic calls only */

  {
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOMCR(((1 << 7) | (1 << 6)) | (((2 & 0x8) << 5) | ((2 & 0x7) << 0)));
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSMR(0);
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOSCR(1);
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$clearOSSRbit();
    /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$setOIERbit(TRUE);
  }
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t PlatformP$InitL0$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 243 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/hardware.h"
static inline void TOSH_SET_PIN_DIRECTIONS(void )
{

  * (volatile uint32_t *)0x40F00004 = (1 << 5) | (1 << 4);
}

# 52 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/PlatformP.nc"
static inline error_t PlatformP$Init$init(void )
#line 52
{


  * (volatile uint32_t *)0x41300004 = (((1 << 22) | (1 << 20)) | (1 << 15)) | (1 << 9);

  * (volatile uint32_t *)0x48000048 = (((1 << 24) | ((
  0 & 0xF) << 8)) | ((1 & 0xF) << 4)) | ((4 & 0xF) << 0);


  * (volatile uint32_t *)0x41300008 = 1 << 1;
  while ((* (volatile uint32_t *)0x41300008 & (1 << 0)) == 0) ;

  TOSH_SET_PIN_DIRECTIONS();



   __asm volatile ("mcr p15,0,%0,c15,c1,0\n\t" :  : "r"(0x43));




  * (volatile uint32_t *)0x41300000 = (((1 << 31) | (8 & 0x1f)) | ((2 & 0xf) << 7)) | (1 << 25);
   __asm volatile (
  "mcr p14,0,%0,c6,c0,0\n\t" :  : 

  "r"(1 << 1));
#line 91
  * (volatile uint32_t *)0x48000064 = (1 & 0x3) << 12;
  * (volatile uint32_t *)0x48000008 = ((* (volatile uint32_t *)0x48000008 | (1 << 3)) | (1 << 15)) | ((2 & 0x7) << 0);
  * (volatile uint32_t *)0x4800000C = * (volatile uint32_t *)0x4800000C | (1 << 3);
  * (volatile uint32_t *)0x48000010 = * (volatile uint32_t *)0x48000010 | (1 << 3);
  * (volatile uint32_t *)0x48000014 = 0;

  * (volatile uint32_t *)0x48000000 = ((((((((1 << 27) | (1 << 11)) | ((0x3 & 0x3) << 24)) | (
  1 << 13)) | ((0x3 & 0x3) << 8)) | (1 << 7)) | ((
  0x2 & 0x3) << 5)) | ((0x1 & 0x3) << 3)) | (1 << 2);

  * (volatile uint32_t *)0x48000004 = (* (volatile uint32_t *)0x48000004 & ~((1 << 29) | (1 << 14))) | (1 << 14);

  enableICache();
  initSyncFlash();



  PlatformP$InitL0$init();
  PlatformP$InitL1$init();
  PlatformP$InitL2$init();
  PlatformP$InitL3$init();


  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t RealMainP$PlatformInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = PlatformP$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 60 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterrupt.nc"
inline static error_t HplPXA27xOSTimerM$OST0Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(26);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST1Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(27);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST2Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(28);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST3Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(29);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
inline static error_t HplPXA27xOSTimerM$OST4_11Irq$allocate(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = HplPXA27xInterruptM$PXA27xIrq$allocate(7);
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60





inline static void HplPXA27xOSTimerM$OST0Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(26);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST1Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(27);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST2Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(28);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST3Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(29);
#line 65
}
#line 65
inline static void HplPXA27xOSTimerM$OST4_11Irq$enable(void ){
#line 65
  HplPXA27xInterruptM$PXA27xIrq$enable(7);
#line 65
}
#line 65
# 101 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(uint8_t pin)
{
  *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00018 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00118) = 1 << (pin & 0x1f);
  return;
}

# 66 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPSRbit(uint8_t arg_0x40675010){
#line 66
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPSRbit(arg_0x40675010);
#line 66
}
#line 66
# 54 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static bool RealMainP$Scheduler$runNextTask(void ){
#line 54
  unsigned char __nesc_result;
#line 54

#line 54
  __nesc_result = SchedulerBasicP$Scheduler$runNextTask();
#line 54

#line 54
  return __nesc_result;
#line 54
}
#line 54
# 616 "VMC.nc"
static inline void VMC$SerialAMSend$sendDone(message_t *msg, error_t err)
#line 616
{

  if (&VMC$serial_p == msg) {
    VMC$serial_busy = FALSE;
    }
}

# 99 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
inline static void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$sendDone(message_t * msg, error_t error){
#line 99
  VMC$SerialAMSend$sendDone(msg, error);
#line 99
}
#line 99
# 57 "/opt/tinyos-2.1.0/tos/system/AMQueueEntryP.nc"
static inline void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(message_t *m, error_t err)
#line 57
{
  /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$sendDone(m, err);
}

# 207 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(uint8_t id, message_t *msg, error_t err)
#line 207
{
}

# 89 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(uint8_t arg_0x40a91510, message_t * msg, error_t error){
#line 89
  switch (arg_0x40a91510) {
#line 89
    case 0U:
#line 89
      /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$default$sendDone(arg_0x40a91510, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 118 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask(void )
#line 118
{
  uint8_t i;
#line 119
  uint8_t j;
#line 119
  uint8_t mask;
#line 119
  uint8_t last;
  message_t *msg;

#line 121
  for (i = 0; i < 1 / 8 + 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[i]) {
          for (mask = 1, j = 0; j < 8; j++) {
              if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[i] & mask) {
                  last = i * 8 + j;
                  msg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[last].msg;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[last].msg = (void *)0;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[i] &= ~mask;
                  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(last, msg, ECANCEL);
                }
              mask <<= 1;
            }
        }
    }
}

#line 161
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask(void )
#line 161
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current, /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg, FAIL);
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
inline static error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(am_id_t arg_0x40a91ef8, am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(arg_0x40a91ef8, addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 235 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_ntoh_uint8(const void * source)
#line 235
{
  const uint8_t *base = source;

#line 237
  return base[0];
}

# 49 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline serial_header_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(message_t * msg)
#line 49
{
  return (serial_header_t * )((uint8_t *)msg + (size_t )& ((message_t *)0)->data - sizeof(serial_header_t ));
}

#line 106
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$payloadLength(message_t *msg)
#line 106
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(msg);

#line 108
  return __nesc_ntoh_uint8(header->length.data);
}

# 67 "/opt/tinyos-2.1.0/tos/interfaces/Packet.nc"
inline static uint8_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$payloadLength(message_t * msg){
#line 67
  unsigned char __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$payloadLength(msg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 67 "/opt/tinyos-2.1.0/tos/interfaces/AMPacket.nc"
inline static am_addr_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(message_t * amsg){
#line 67
  unsigned short __nesc_result;
#line 67

#line 67
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$destination(amsg);
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 156 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline am_id_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(message_t *amsg)
#line 156
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(amsg);

#line 158
  return __nesc_ntoh_uint8(header->type.data);
}

# 136 "/opt/tinyos-2.1.0/tos/interfaces/AMPacket.nc"
inline static am_id_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(message_t * amsg){
#line 136
  unsigned char __nesc_result;
#line 136

#line 136
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(amsg);
#line 136

#line 136
  return __nesc_result;
#line 136
}
#line 136
# 57 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$nextPacket(void )
#line 57
{
  uint8_t i;

#line 59
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current + 1) % 1;
  for (i = 0; i < 1; i++) {
      if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg == (void *)0 || 
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$cancelMask[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current / 8] & (1 << /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current % 8)) 
        {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current = (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current + 1) % 1;
        }
      else {
          break;
        }
    }
  if (i >= 1) {
#line 70
    /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current = 1;
    }
}

#line 166
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend(void )
#line 166
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$nextPacket();
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current < 1) {
      error_t nextErr;
      message_t *nextMsg = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg;
      am_id_t nextId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(nextMsg);
      am_addr_t nextDest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(nextMsg);
      uint8_t len = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$payloadLength(nextMsg);

#line 174
      nextErr = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(nextId, nextDest, nextMsg, len);
      if (nextErr != SUCCESS) {
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$postTask();
        }
    }
}

# 264 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_ntoh_uint16(const void * source)
#line 264
{
  const uint8_t *base = source;

#line 266
  return ((uint16_t )base[0] << 8) | base[1];
}

# 522 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline error_t SerialP$SendBytePacket$startSend(uint8_t b)
#line 522
{
  bool not_busy = FALSE;

#line 524
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 524
    {
      if (SerialP$txBuf[SerialP$TX_DATA_INDEX].state == SerialP$BUFFER_AVAILABLE) {
          SerialP$txBuf[SerialP$TX_DATA_INDEX].state = SerialP$BUFFER_FILLING;
          SerialP$txBuf[SerialP$TX_DATA_INDEX].buf = b;
          not_busy = TRUE;
        }
    }
#line 530
    __nesc_atomic_end(__nesc_atomic); }
  if (not_busy) {
      SerialP$MaybeScheduleTx();
      return SUCCESS;
    }
  return EBUSY;
}

# 51 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$startSend(uint8_t first_byte){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = SerialP$SendBytePacket$startSend(first_byte);
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 43 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$dataLinkLength(message_t *msg, uint8_t upperLen)
#line 43
{
  return upperLen + sizeof(serial_header_t );
}

# 347 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(uart_id_t id, message_t *msg, 
uint8_t upperLen)
#line 348
{
  return 0;
}

# 23 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$dataLinkLength(uart_id_t arg_0x408e5b30, message_t *msg, uint8_t upperLen){
#line 23
  unsigned char __nesc_result;
#line 23

#line 23
  switch (arg_0x408e5b30) {
#line 23
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 23
      __nesc_result = SerialPacketInfoActiveMessageP$Info$dataLinkLength(msg, upperLen);
#line 23
      break;
#line 23
    default:
#line 23
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$dataLinkLength(arg_0x408e5b30, msg, upperLen);
#line 23
      break;
#line 23
    }
#line 23

#line 23
  return __nesc_result;
#line 23
}
#line 23
# 100 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(uint8_t id, message_t *msg, uint8_t len)
#line 100
{
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState != /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE) {
      return EBUSY;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 105
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(id);
      if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex > sizeof(message_header_t )) {
          {
            unsigned char __nesc_temp = 
#line 108
            ESIZE;

            {
#line 108
              __nesc_atomic_end(__nesc_atomic); 
#line 108
              return __nesc_temp;
            }
          }
        }
#line 111
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError = SUCCESS;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer = (uint8_t *)msg;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_DATA;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendId = id;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendCancelled = FALSE;






      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendLen = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$dataLinkLength(id, msg, len) + /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendIndex;
    }
#line 123
    __nesc_atomic_end(__nesc_atomic); }
  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$startSend(id) == SUCCESS) {
      return SUCCESS;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE;
      return FAIL;
    }
}

# 64 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
inline static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$send(TOS_SERIAL_ACTIVE_MESSAGE_ID, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 181 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
static inline void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(am_id_t id, message_t *msg, error_t err)
#line 181
{





  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current >= 1) {
      return;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current].msg == msg) {
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current, msg, err);
    }
  else {
      ;
    }
}

# 99 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
inline static void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$sendDone(am_id_t arg_0x40826200, message_t * msg, error_t error){
#line 99
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$sendDone(arg_0x40826200, msg, error);
#line 99
}
#line 99
# 85 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(message_t *msg, error_t result)
#line 85
{
  /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$sendDone(/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(msg), msg, result);
}

# 362 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(uart_id_t idxxx, message_t *msg, error_t error)
#line 362
{
  return;
}

# 89 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
inline static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$sendDone(uart_id_t arg_0x408e5030, message_t * msg, error_t error){
#line 89
  switch (arg_0x408e5030) {
#line 89
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 89
      /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$sendDone(msg, error);
#line 89
      break;
#line 89
    default:
#line 89
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$default$sendDone(arg_0x408e5030, msg, error);
#line 89
      break;
#line 89
    }
#line 89
}
#line 89
# 147 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask(void )
#line 147
{
  error_t error;

  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendState = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SEND_STATE_IDLE;
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
    error = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError;
#line 151
    __nesc_atomic_end(__nesc_atomic); }

  if (/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendCancelled) {
#line 153
    error = ECANCEL;
    }
#line 154
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Send$sendDone(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendId, (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendBuffer, error);
}

#line 201
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$unlockBuffer(uint8_t which)
#line 201
{
  if (which) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufOneLocked = 0;
    }
  else {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.bufZeroLocked = 0;
    }
}

# 153 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(uint8_t num)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num].isrunning = FALSE;
}

# 586 "VMC.nc"
static inline void VMC$Timer$default$stop(uint8_t id)
#line 586
{
  ;
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void VMC$Timer$stop(uint8_t arg_0x405b2258){
#line 67
  switch (arg_0x405b2258) {
#line 67
    case 0:
#line 67
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(1U);
#line 67
      break;
#line 67
    case 1:
#line 67
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(2U);
#line 67
      break;
#line 67
    case 2:
#line 67
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(3U);
#line 67
      break;
#line 67
    default:
#line 67
      VMC$Timer$default$stop(arg_0x405b2258);
#line 67
      break;
#line 67
    }
#line 67
}
#line 67
# 98 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2Off(void )
#line 98
{
  LedsP$Led2$set();
  ;
#line 100
  ;
}

# 83 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void VMC$Leds$led2Off(void ){
#line 83
  LedsP$Leds$led2Off();
#line 83
}
#line 83
# 83 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led1Off(void )
#line 83
{
  LedsP$Led1$set();
  ;
#line 85
  ;
}

# 66 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void VMC$Leds$led1Off(void ){
#line 66
  LedsP$Leds$led1Off();
#line 66
}
#line 66
# 68 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led0Off(void )
#line 68
{
  LedsP$Led0$set();
  ;
#line 70
  ;
}

# 50 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void VMC$Leds$led0Off(void ){
#line 50
  LedsP$Leds$led0Off();
#line 50
}
#line 50
# 91 "VMC.nc"
static inline void VMC$unload_binary(uint8_t appid)
#line 91
{


  uint8_t i;
#line 94
  uint8_t id;

#line 95
  id = 3;
  for (i = 0; i < 3; i++) {

      if (VMC$apps[i].id == appid) {
          id = i;
          break;
        }
    }


  if (id == 3) {
      ;
      return;
    }

  VMC$apps[id].isActive = FALSE;
  VMC$apps[id].handler = 0;
  VMC$apps[id].id = 0;

  switch (id) {

      case 0: 
        VMC$Leds$led0Off();
      break;

      case 1: 
        VMC$Leds$led1Off();
      break;

      case 2: 
        VMC$Leds$led2Off();
      break;
    }



  if (id != 0) {
    VMC$Timer$stop(id);
    }

  if (id == VMC$active_app) {
      for (i = 0; i < 3; i++) {
          VMC$active_app = (VMC$active_app + 1) % 3;
          if (VMC$apps[VMC$active_app].isActive && VMC$apps[VMC$active_app].handler != 0) {
            break;
            }
        }
    }
#line 142
  ;

  if (VMC$active_app == id) {
    VMC$active_app = 3;
    }
}

# 64 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR(void ){
#line 64
  unsigned int __nesc_result;
#line 64

#line 64
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSCR(4);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 152 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow(void )
#line 152
{
  return /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR();
}

# 98 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getNow(void ){
#line 98
  unsigned int __nesc_result;
#line 98

#line 98
  __nesc_result = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getNow();
#line 98

#line 98
  return __nesc_result;
#line 98
}
#line 98
# 85 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline uint32_t /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow(void )
{
#line 86
  return /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getNow();
}

# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow(void ){
#line 125
  unsigned int __nesc_result;
#line 125

#line 125
  __nesc_result = /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$getNow();
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 143 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(uint8_t num, uint32_t dt)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$startTimer(num, /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow(), dt, FALSE);
}

# 582 "VMC.nc"
static inline void VMC$Timer$default$startPeriodic(uint8_t id, uint32_t milli)
#line 582
{
  ;
}

# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void VMC$Timer$startPeriodic(uint8_t arg_0x405b2258, uint32_t dt){
#line 53
  switch (arg_0x405b2258) {
#line 53
    case 0:
#line 53
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(1U, dt);
#line 53
      break;
#line 53
    case 1:
#line 53
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(2U, dt);
#line 53
      break;
#line 53
    case 2:
#line 53
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(3U, dt);
#line 53
      break;
#line 53
    default:
#line 53
      VMC$Timer$default$startPeriodic(arg_0x405b2258, dt);
#line 53
      break;
#line 53
    }
#line 53
}
#line 53
# 150 "VMC.nc"
static inline void VMC$load_binary(uint8_t *bin, uint8_t appid)
#line 150
{

  uint8_t i;
#line 152
  uint8_t id;

#line 153
  id = 3;


  for (i = 0; i < 3; i++) {

      if (VMC$apps[i].id == appid) {
          id = i;
          break;
        }


      if (! VMC$apps[i].isActive && id == 3) {
        id = i;
        }
    }

  if (id == 3) {
      ;
      return;
    }


  for (i = 0; i < bin[1]; i++) 
    VMC$apps[id].init[i] = bin[3 + i];


  for (i = 0; i < bin[2]; i++) 
    VMC$apps[id].timer[i] = bin[3 + bin[1] + i];


  VMC$apps[id].handler = 1;
  VMC$apps[id].isActive = TRUE;
  VMC$apps[id].id = appid;
  VMC$apps[id].waiting_sensor = 6;


  if (bin[2] > 0) {
      VMC$apps[id].next_handler = 2;
      VMC$Timer$startPeriodic(id, 1000);
    }


  for (i = 0; i < 6; i++) 
    VMC$apps[id].regs[i] = 0;
#line 215
  VMC$next_app();
}

#line 623
static inline message_t *VMC$SerialReceive$receive(message_t *msg, void *payload, uint8_t len)
#line 623
{

  test_serial_msg *serial_msg = (test_serial_msg *)payload;

  if (__nesc_ntoh_uint8(serial_msg->type.data) == 11) {
    VMC$load_binary((uint8_t *)serial_msg->binary, __nesc_ntoh_uint8(serial_msg->appid.data));
    }
  else {
#line 629
    if (__nesc_ntoh_uint8(serial_msg->type.data) == 12) {
      VMC$unload_binary(__nesc_ntoh_uint8(serial_msg->appid.data));
      }
    }
#line 632
  return msg;
}

# 93 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(uint8_t id, message_t *msg, void *payload, uint8_t len)
#line 93
{
  return msg;
}

# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
inline static message_t * /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$receive(am_id_t arg_0x40826bc0, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x40826bc0) {
#line 67
    case 6:
#line 67
      __nesc_result = VMC$SerialReceive$receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$default$receive(arg_0x40826bc0, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 97 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline message_t */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(message_t *msg, void *payload, uint8_t len)
#line 97
{
  return /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Receive$receive(/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$type(msg), msg, msg->data, len);
}

# 357 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline message_t */*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(uart_id_t idxxx, message_t *msg, 
void *payload, 
uint8_t len)
#line 359
{
  return msg;
}

# 67 "/opt/tinyos-2.1.0/tos/interfaces/Receive.nc"
inline static message_t * /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$receive(uart_id_t arg_0x408e89d0, message_t * msg, void * payload, uint8_t len){
#line 67
  nx_struct message_t *__nesc_result;
#line 67

#line 67
  switch (arg_0x408e89d0) {
#line 67
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 67
      __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubReceive$receive(msg, payload, len);
#line 67
      break;
#line 67
    default:
#line 67
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$default$receive(arg_0x408e89d0, msg, payload, len);
#line 67
      break;
#line 67
    }
#line 67

#line 67
  return __nesc_result;
#line 67
}
#line 67
# 46 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfoActiveMessageP.nc"
static inline uint8_t SerialPacketInfoActiveMessageP$Info$upperLength(message_t *msg, uint8_t dataLinkLen)
#line 46
{
  return dataLinkLen - sizeof(serial_header_t );
}

# 351 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(uart_id_t id, message_t *msg, 
uint8_t dataLinkLen)
#line 352
{
  return 0;
}

# 31 "/opt/tinyos-2.1.0/tos/lib/serial/SerialPacketInfo.nc"
inline static uint8_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$upperLength(uart_id_t arg_0x408e5b30, message_t *msg, uint8_t dataLinkLen){
#line 31
  unsigned char __nesc_result;
#line 31

#line 31
  switch (arg_0x408e5b30) {
#line 31
    case TOS_SERIAL_ACTIVE_MESSAGE_ID:
#line 31
      __nesc_result = SerialPacketInfoActiveMessageP$Info$upperLength(msg, dataLinkLen);
#line 31
      break;
#line 31
    default:
#line 31
      __nesc_result = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$default$upperLength(arg_0x408e5b30, msg, dataLinkLen);
#line 31
      break;
#line 31
    }
#line 31

#line 31
  return __nesc_result;
#line 31
}
#line 31
# 264 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask(void )
#line 264
{
  uart_id_t myType;
  message_t *myBuf;
  uint8_t mySize;
  uint8_t myWhich;

#line 269
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 269
    {
      myType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskType;
      myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskBuf;
      mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskSize;
      myWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskWhich;
    }
#line 274
    __nesc_atomic_end(__nesc_atomic); }
  mySize -= /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$offset(myType);
  mySize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$PacketInfo$upperLength(myType, myBuf, mySize);
  myBuf = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$Receive$receive(myType, myBuf, myBuf, mySize);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 278
    {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$messagePtrs[myWhich] = myBuf;
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$unlockBuffer(myWhich);
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending = FALSE;
    }
#line 282
    __nesc_atomic_end(__nesc_atomic); }
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
inline static error_t VMC$execute_instructionTask$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(VMC$execute_instructionTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 613 "VMC.nc"
static inline void VMC$SerialAMControl$stopDone(error_t err)
#line 613
{
}

# 117 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void SerialP$SplitControl$stopDone(error_t error){
#line 117
  VMC$SerialAMControl$stopDone(error);
#line 117
}
#line 117
# 135 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$stop(void )
#line 135
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 136
    {
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(0);
    }
#line 138
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 84 "/opt/tinyos-2.1.0/tos/interfaces/StdControl.nc"
inline static error_t SerialP$SerialControl$stop(void ){
#line 84
  unsigned char __nesc_result;
#line 84

#line 84
  __nesc_result = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$stop();
#line 84

#line 84
  return __nesc_result;
#line 84
}
#line 84
# 330 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFlush$flushDone(void )
#line 330
{
  SerialP$SerialControl$stop();
  SerialP$SplitControl$stopDone(SUCCESS);
}

static inline void SerialP$defaultSerialFlushTask$runTask(void )
#line 335
{
  SerialP$SerialFlush$flushDone();
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$defaultSerialFlushTask$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(SerialP$defaultSerialFlushTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 338 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline void SerialP$SerialFlush$default$flush(void )
#line 338
{
  SerialP$defaultSerialFlushTask$postTask();
}

# 38 "/opt/tinyos-2.1.0/tos/lib/serial/SerialFlush.nc"
inline static void SerialP$SerialFlush$flush(void ){
#line 38
  SerialP$SerialFlush$default$flush();
#line 38
}
#line 38
# 326 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline void SerialP$stopDoneTask$runTask(void )
#line 326
{
  SerialP$SerialFlush$flush();
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t SerialP$startDoneTask$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(SerialP$startDoneTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 342 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline error_t SerialP$SplitControl$start(void )
#line 342
{
  SerialP$startDoneTask$postTask();
  return SUCCESS;
}

# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static error_t VMC$SerialAMControl$start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = SerialP$SplitControl$start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 607 "VMC.nc"
static inline void VMC$SerialAMControl$startDone(error_t err)
#line 607
{
  if (err != SUCCESS) {
    VMC$SerialAMControl$start();
    }
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void SerialP$SplitControl$startDone(error_t error){
#line 92
  VMC$SerialAMControl$startDone(error);
#line 92
}
#line 92
# 128 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static inline error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$start(void )
#line 128
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 129
    {
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER((1 << 6) | (1 << 0));
    }
#line 131
    __nesc_atomic_end(__nesc_atomic); }
  return SUCCESS;
}

# 74 "/opt/tinyos-2.1.0/tos/interfaces/StdControl.nc"
inline static error_t SerialP$SerialControl$start(void ){
#line 74
  unsigned char __nesc_result;
#line 74

#line 74
  __nesc_result = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$StdControl$start();
#line 74

#line 74
  return __nesc_result;
#line 74
}
#line 74
# 320 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static inline void SerialP$startDoneTask$runTask(void )
#line 320
{
  SerialP$SerialControl$start();
  SerialP$SplitControl$startDone(SUCCESS);
}

# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 183 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static inline void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error_t error)
#line 183
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 184
    /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$sendError = error;
#line 184
    __nesc_atomic_end(__nesc_atomic); }
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$postTask();
}

# 80 "/opt/tinyos-2.1.0/tos/lib/serial/SendBytePacket.nc"
inline static void SerialP$SendBytePacket$sendCompleted(error_t error){
#line 80
  /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$SendBytePacket$sendCompleted(error);
#line 80
}
#line 80
# 242 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static __inline bool SerialP$ack_queue_is_empty(void )
#line 242
{
  bool ret;

#line 244
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 244
    ret = SerialP$ackQ.writePtr == SerialP$ackQ.readPtr;
#line 244
    __nesc_atomic_end(__nesc_atomic); }
  return ret;
}











static __inline uint8_t SerialP$ack_queue_top(void )
#line 258
{
  uint8_t tmp = 0;

  /* atomic removed: atomic calls only */
#line 260
  {
    if (!SerialP$ack_queue_is_empty()) {
        tmp = SerialP$ackQ.buf[SerialP$ackQ.readPtr];
      }
  }
  return tmp;
}

static inline uint8_t SerialP$ack_queue_pop(void )
#line 268
{
  uint8_t retval = 0;

#line 270
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 270
    {
      if (SerialP$ackQ.writePtr != SerialP$ackQ.readPtr) {
          retval = SerialP$ackQ.buf[SerialP$ackQ.readPtr];
          if (++ SerialP$ackQ.readPtr > SerialP$ACK_QUEUE_SIZE) {
#line 273
            SerialP$ackQ.readPtr = 0;
            }
        }
    }
#line 276
    __nesc_atomic_end(__nesc_atomic); }
#line 276
  return retval;
}

#line 539
static inline void SerialP$RunTx$runTask(void )
#line 539
{
  uint8_t idle;
  uint8_t done;
  uint8_t fail;









  error_t result = SUCCESS;
  bool send_completed = FALSE;
  bool start_it = FALSE;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 556
    {
      SerialP$txPending = 0;
      idle = SerialP$txState == SerialP$TXSTATE_IDLE;
      done = SerialP$txState == SerialP$TXSTATE_FINISH;
      fail = SerialP$txState == SerialP$TXSTATE_ERROR;
      if (done || fail) {
          SerialP$txState = SerialP$TXSTATE_IDLE;
          SerialP$txBuf[SerialP$txIndex].state = SerialP$BUFFER_AVAILABLE;
        }
    }
#line 565
    __nesc_atomic_end(__nesc_atomic); }


  if (done || fail) {
      SerialP$txSeqno++;
      if (SerialP$txProto == SERIAL_PROTO_ACK) {
          SerialP$ack_queue_pop();
        }
      else {
          result = done ? SUCCESS : FAIL;
          send_completed = TRUE;
        }
      idle = TRUE;
    }


  if (idle) {
      bool goInactive;

#line 583
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 583
        goInactive = SerialP$offPending;
#line 583
        __nesc_atomic_end(__nesc_atomic); }
      if (goInactive) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 585
            SerialP$txState = SerialP$TXSTATE_INACTIVE;
#line 585
            __nesc_atomic_end(__nesc_atomic); }
        }
      else {

          uint8_t myAckState;
          uint8_t myDataState;

#line 591
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 591
            {
              myAckState = SerialP$txBuf[SerialP$TX_ACK_INDEX].state;
              myDataState = SerialP$txBuf[SerialP$TX_DATA_INDEX].state;
            }
#line 594
            __nesc_atomic_end(__nesc_atomic); }
          if (!SerialP$ack_queue_is_empty() && myAckState == SerialP$BUFFER_AVAILABLE) {
              { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 596
                {
                  SerialP$txBuf[SerialP$TX_ACK_INDEX].state = SerialP$BUFFER_COMPLETE;
                  SerialP$txBuf[SerialP$TX_ACK_INDEX].buf = SerialP$ack_queue_top();
                }
#line 599
                __nesc_atomic_end(__nesc_atomic); }
              SerialP$txProto = SERIAL_PROTO_ACK;
              SerialP$txIndex = SerialP$TX_ACK_INDEX;
              start_it = TRUE;
            }
          else {
#line 604
            if (myDataState == SerialP$BUFFER_FILLING || myDataState == SerialP$BUFFER_COMPLETE) {
                SerialP$txProto = SERIAL_PROTO_PACKET_NOACK;
                SerialP$txIndex = SerialP$TX_DATA_INDEX;
                start_it = TRUE;
              }
            else {
              }
            }
        }
    }
  else {
    }


  if (send_completed) {
      SerialP$SendBytePacket$sendCompleted(result);
    }

  if (SerialP$txState == SerialP$TXSTATE_INACTIVE) {
      SerialP$testOff();
      return;
    }

  if (start_it) {

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 629
        {
          SerialP$txCRC = 0;
          SerialP$txByteCnt = 0;
          SerialP$txState = SerialP$TXSTATE_PROTO;
        }
#line 633
        __nesc_atomic_end(__nesc_atomic); }
      if (SerialP$SerialFrameComm$putDelimiter() != SUCCESS) {
          { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 635
            SerialP$txState = SerialP$TXSTATE_ERROR;
#line 635
            __nesc_atomic_end(__nesc_atomic); }
          SerialP$MaybeScheduleTx();
        }
    }
}

# 159 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$runTask(void )
#line 159
{
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISAR(0x7F0);
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (1 << 4));
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(1 << 14);
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 164
    {
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_IDLE;
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$readDone(FAIL, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurTargetAddr, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf);
    }
#line 167
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

static inline void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$runTask(void )
#line 171
{
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setISAR(0x7F0);
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (1 << 4));
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(1 << 14);
  /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 176
    {
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_IDLE;
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$writeDone(FAIL, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurTargetAddr, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen, /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf);
    }
#line 179
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
inline static error_t DemoSensorP$Light$read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$read();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 56 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$SensorControl$startDone(error_t t)
#line 56
{
  DemoSensorP$Light$read();
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$startDone(error_t error){
#line 92
  DemoSensorP$SensorControl$startDone(error);
#line 92
}
#line 92
# 144 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone$runTask(void )
#line 144
{
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$startDone(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mSSError);
  return;
}

# 54 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(resource_client_id_t id)
#line 54
{
  return /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[id] != /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY || /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qTail == id;
}

#line 72
static inline error_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(resource_client_id_t id)
#line 72
{
  /* atomic removed: atomic calls only */
#line 73
  {
    if (!/*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEnqueued(id)) {
        if (/*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead == /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
          /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead = id;
          }
        else {
#line 78
          /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[/*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qTail] = id;
          }
#line 79
        /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qTail = id;
        {
          unsigned char __nesc_temp = 
#line 80
          SUCCESS;

#line 80
          return __nesc_temp;
        }
      }
#line 82
    {
      unsigned char __nesc_temp = 
#line 82
      EBUSY;

#line 82
      return __nesc_temp;
    }
  }
}

# 69 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(resource_client_id_t id){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$enqueue(id);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 166 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(uint8_t id)
#line 166
{
}

# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceRequested.nc"
inline static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(uint8_t arg_0x4072b5b0){
#line 43
    /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$default$requested(arg_0x4072b5b0);
#line 43
}
#line 43
# 71 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(uint8_t id)
#line 71
{
  /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceRequested$requested(/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 73
    {
      if (/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$state == /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE) {
          /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
          /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId = id;
          /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
          {
            unsigned char __nesc_temp = 
#line 78
            SUCCESS;

            {
#line 78
              __nesc_atomic_end(__nesc_atomic); 
#line 78
              return __nesc_temp;
            }
          }
        }
#line 80
      {
        unsigned char __nesc_temp = 
#line 80
        /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$enqueue(id);

        {
#line 80
          __nesc_atomic_end(__nesc_atomic); 
#line 80
          return __nesc_temp;
        }
      }
    }
#line 83
    __nesc_atomic_end(__nesc_atomic); }
}

# 78 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$request(void ){
#line 78
  unsigned char __nesc_result;
#line 78

#line 78
  __nesc_result = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$request(/*VMCApp.Sensor.Sensor*/Tsl2561C$0$BB_KEY);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 59 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$SensorControl$stopDone(error_t t)
#line 59
{
}

# 117 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$stopDone(error_t error){
#line 117
  DemoSensorP$SensorControl$stopDone(error);
#line 117
}
#line 117
# 149 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone$runTask(void )
#line 149
{
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$stopDone(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mSSError);
  return;
}

# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
inline static void DemoSensorP$Read$readDone(error_t result, DemoSensorP$Read$val_t val){
#line 63
  VMC$Read$readDone(result, val);
#line 63
}
#line 63
# 61 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$Light$readDone(error_t err, uint16_t value)
#line 61
{
  DemoSensorP$Read$readDone(err, value);
}

# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
inline static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$readDone(error_t result, /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$val_t val){
#line 63
  DemoSensorP$Light$readDone(result, val);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*VMCApp.Sensor.Sensor*/Tsl2561C$0$BB_KEY);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 185 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh0(void )
#line 185
{
  return /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doReadPrep(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READCH0, 0xC);
}

# 43 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static error_t Tsl2561InternalP$ToHPLC$measureCh0(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh0();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 55 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline error_t Tsl2561InternalP$HplTSL256x$measureCh0(uint8_t id)
#line 55
{
  Tsl2561InternalP$currentId = id;
  return Tsl2561InternalP$ToHPLC$measureCh0();
}

# 43 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh0(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = Tsl2561InternalP$HplTSL256x$measureCh0(/*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 94 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$granted(void )
#line 94
{
  error_t result;

#line 96
  result = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh0();
  if (result != SUCCESS) {
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$release();
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$readDone(result, 0);
    }
}

# 66 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$IRLight$readDone(error_t err, uint16_t value)
#line 66
{
}

# 63 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
inline static void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRPhoto$readDone(error_t result, /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRPhoto$val_t val){
#line 63
  DemoSensorP$IRLight$readDone(result, val);
#line 63
}
#line 63
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*VMCApp.Sensor.Sensor*/Tsl2561C$0$IR_KEY);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 189 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh1(void )
#line 189
{
  return /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doReadPrep(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READCH1, 0xE);
}

# 46 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static error_t Tsl2561InternalP$ToHPLC$measureCh1(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh1();
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 59 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline error_t Tsl2561InternalP$HplTSL256x$measureCh1(uint8_t id)
#line 59
{
  Tsl2561InternalP$currentId = id;
  return Tsl2561InternalP$ToHPLC$measureCh1();
}

# 46 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh1(void ){
#line 46
  unsigned char __nesc_result;
#line 46

#line 46
  __nesc_result = Tsl2561InternalP$HplTSL256x$measureCh1(/*VMCApp.Sensor.Sensor*/Tsl2561C$0$READER_ID);
#line 46

#line 46
  return __nesc_result;
#line 46
}
#line 46
# 103 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$granted(void )
#line 103
{
  error_t result;

#line 105
  result = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$HplTSL256x$measureCh1();
  if (result != SUCCESS) {
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$release();
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRPhoto$readDone(result, 0);
    }
}

# 211 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setINTERRUPT(uint8_t val)
#line 211
{
  return /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doWriteReg(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_SETINTERRUPT, (1 << 6) | 0x6, val, 1);
}

# 61 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL256x.nc"
inline static error_t HalTsl2561ControlP$HplTSL256x$setINTERRUPT(uint8_t val){
#line 61
  unsigned char __nesc_result;
#line 61

#line 61
  __nesc_result = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setINTERRUPT(val);
#line 61

#line 61
  return __nesc_result;
#line 61
}
#line 61
# 197 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$Resource$granted(void )
#line 197
{

  if (HalTsl2561ControlP$state == HalTsl2561ControlP$S_ENALERT) {
      HalTsl2561ControlP$HplTSL256x$setINTERRUPT(HalTsl2561ControlP$iControlRegisterShadow);
    }
  return;
}

# 164 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(uint8_t id)
#line 164
{
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(uint8_t arg_0x4072cab0){
#line 92
  switch (arg_0x4072cab0) {
#line 92
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$BB_KEY:
#line 92
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$granted();
#line 92
      break;
#line 92
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$IR_KEY:
#line 92
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$granted();
#line 92
      break;
#line 92
    case /*VMCApp.Sensor.Sensor*/Tsl2561C$0$ADV_KEY:
#line 92
      HalTsl2561ControlP$Resource$granted();
#line 92
      break;
#line 92
    default:
#line 92
      /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$default$granted(arg_0x4072cab0);
#line 92
      break;
#line 92
    }
#line 92
}
#line 92
# 170 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(uint8_t id)
#line 170
{
}

# 49 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
inline static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(uint8_t arg_0x4072a200){
#line 49
    /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$configure(arg_0x4072a200);
#line 49
}
#line 49
# 154 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask(void )
#line 154
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 155
    {
      /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId;
      /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY;
    }
#line 158
    __nesc_atomic_end(__nesc_atomic); }
  /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$configure(/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
  /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$granted(/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$resId);
}

# 217 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static inline error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$write(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t *data)
#line 217
{
  error_t error = SUCCESS;

  error = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$startI2CTransact(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_WRITE, addr, length, data, flags, FALSE);

  return error;
}

# 81 "/opt/tinyos-2.1.0/tos/interfaces/I2CPacket.nc"
inline static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$write(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t * data){
#line 81
  unsigned char __nesc_result;
#line 81

#line 81
  __nesc_result = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$write(flags, addr, length, data);
#line 81

#line 81
  return __nesc_result;
#line 81
}
#line 81
# 50 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline bool /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty(void )
#line 50
{
  return /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead == /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
}

# 43 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static bool /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$isEmpty();
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 58 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline resource_client_id_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue(void )
#line 58
{
  /* atomic removed: atomic calls only */
#line 59
  {
    if (/*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead != /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
        uint8_t id = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead;

#line 62
        /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[/*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead];
        if (/*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qHead == /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY) {
          /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$qTail = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
          }
#line 65
        /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$resQ[id] = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;
        {
          unsigned char __nesc_temp = 
#line 66
          id;

#line 66
          return __nesc_temp;
        }
      }
#line 68
    {
      unsigned char __nesc_temp = 
#line 68
      /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY;

#line 68
      return __nesc_temp;
    }
  }
}

# 60 "/opt/tinyos-2.1.0/tos/interfaces/ResourceQueue.nc"
inline static resource_client_id_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue(void ){
#line 60
  unsigned char __nesc_result;
#line 60

#line 60
  __nesc_result = /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$FcfsQueue$dequeue();
#line 60

#line 60
  return __nesc_result;
#line 60
}
#line 60
# 172 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static inline void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(uint8_t id)
#line 172
{
}

# 55 "/opt/tinyos-2.1.0/tos/interfaces/ResourceConfigure.nc"
inline static void /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(uint8_t arg_0x4072a200){
#line 55
    /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$default$unconfigure(arg_0x4072a200);
#line 55
}
#line 55
# 85 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$HalTsl2561Advanced$enableAlertDone(error_t err)
#line 85
{
}

# 49 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
inline static void HalTsl2561ControlP$HalTsl2561Advanced$enableAlertDone(error_t error){
#line 49
  DemoSensorP$HalTsl2561Advanced$enableAlertDone(error);
#line 49
}
#line 49
# 110 "/opt/tinyos-2.1.0/tos/interfaces/Resource.nc"
inline static error_t HalTsl2561ControlP$Resource$release(void ){
#line 110
  unsigned char __nesc_result;
#line 110

#line 110
  __nesc_result = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(/*VMCApp.Sensor.Sensor*/Tsl2561C$0$ADV_KEY);
#line 110

#line 110
  return __nesc_result;
#line 110
}
#line 110
# 82 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$HalTsl2561Advanced$setTHighDone(error_t err)
#line 82
{
}

# 47 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
inline static void HalTsl2561ControlP$HalTsl2561Advanced$setTHighDone(error_t error){
#line 47
  DemoSensorP$HalTsl2561Advanced$setTHighDone(error);
#line 47
}
#line 47
# 79 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$HalTsl2561Advanced$setTLowDone(error_t err)
#line 79
{
}

# 45 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
inline static void HalTsl2561ControlP$HalTsl2561Advanced$setTLowDone(error_t error){
#line 45
  DemoSensorP$HalTsl2561Advanced$setTLowDone(error);
#line 45
}
#line 45
# 76 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$HalTsl2561Advanced$setPersistenceDone(error_t err)
#line 76
{
}

# 43 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
inline static void HalTsl2561ControlP$HalTsl2561Advanced$setPersistenceDone(error_t error){
#line 43
  DemoSensorP$HalTsl2561Advanced$setPersistenceDone(error);
#line 43
}
#line 43
# 73 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$HalTsl2561Advanced$setIntegrationDone(error_t err)
#line 73
{
}

# 41 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
inline static void HalTsl2561ControlP$HalTsl2561Advanced$setIntegrationDone(error_t error){
#line 41
  DemoSensorP$HalTsl2561Advanced$setIntegrationDone(error);
#line 41
}
#line 41
# 70 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$HalTsl2561Advanced$setGainDone(error_t err)
#line 70
{
}

# 39 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
inline static void HalTsl2561ControlP$HalTsl2561Advanced$setGainDone(error_t error){
#line 39
  DemoSensorP$HalTsl2561Advanced$setGainDone(error);
#line 39
}
#line 39
# 66 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$complete_Task$runTask(void )
#line 66
{
  switch (HalTsl2561ControlP$state) {
      case HalTsl2561ControlP$S_GAIN: 
        HalTsl2561ControlP$state = HalTsl2561ControlP$S_IDLE;
      HalTsl2561ControlP$Resource$release();
      HalTsl2561ControlP$HalTsl2561Advanced$setGainDone(HalTsl2561ControlP$clientResult);
      break;
      case HalTsl2561ControlP$S_INTEG: 
        HalTsl2561ControlP$state = HalTsl2561ControlP$S_IDLE;
      HalTsl2561ControlP$Resource$release();
      HalTsl2561ControlP$HalTsl2561Advanced$setIntegrationDone(HalTsl2561ControlP$clientResult);
      break;
      case HalTsl2561ControlP$S_PERSIST: 
        HalTsl2561ControlP$state = HalTsl2561ControlP$S_IDLE;
      HalTsl2561ControlP$Resource$release();
      HalTsl2561ControlP$HalTsl2561Advanced$setPersistenceDone(HalTsl2561ControlP$clientResult);
      break;
      case HalTsl2561ControlP$S_TLOW: 
        HalTsl2561ControlP$state = HalTsl2561ControlP$S_IDLE;
      HalTsl2561ControlP$Resource$release();
      HalTsl2561ControlP$HalTsl2561Advanced$setTLowDone(HalTsl2561ControlP$clientResult);
      break;
      case HalTsl2561ControlP$S_THIGH: 
        HalTsl2561ControlP$state = HalTsl2561ControlP$S_IDLE;
      HalTsl2561ControlP$Resource$release();
      HalTsl2561ControlP$HalTsl2561Advanced$setTHighDone(HalTsl2561ControlP$clientResult);
      break;
      case HalTsl2561ControlP$S_ENALERT: 
        HalTsl2561ControlP$state = HalTsl2561ControlP$S_IDLE;
      HalTsl2561ControlP$Resource$release();
      HalTsl2561ControlP$HalTsl2561Advanced$enableAlertDone(HalTsl2561ControlP$clientResult);
      break;
      default: 
        break;
    }
}

# 88 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline void DemoSensorP$HalTsl2561Advanced$alertThreshold(void )
#line 88
{
}

# 50 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561Advanced.nc"
inline static void HalTsl2561ControlP$HalTsl2561Advanced$alertThreshold(void ){
#line 50
  DemoSensorP$HalTsl2561Advanced$alertThreshold();
#line 50
}
#line 50
# 62 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ControlP.nc"
static inline void HalTsl2561ControlP$complete_Alert$runTask(void )
#line 62
{
  HalTsl2561ControlP$HalTsl2561Advanced$alertThreshold();
}

# 60 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static inline void /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$runTask(void )
#line 60
{
  switch (/*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state) {
      case /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READ_BB: 
        /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READY;
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$release();
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$readDone(/*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_error, /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_val);
      break;
      case /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READ_IR: 
        /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READY;
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRResource$release();
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$IRPhoto$readDone(/*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_error, /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_val);
      break;
      default: 
        /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READY;
      break;
    }
}

# 162 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$start(void )
#line 162
{
  error_t error = SUCCESS;

#line 164
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 164
    {
      if (/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState == /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STOPPED) {
          /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STARTING;
        }
      else {
          error = EBUSY;
        }
    }
#line 171
    __nesc_atomic_end(__nesc_atomic); }

  if (error) {
    return error;
    }
  return /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doWriteReg(/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STARTING, (1 << 6) | 0x0, 
  0x3, 1);
}

# 83 "/opt/tinyos-2.1.0/tos/interfaces/SplitControl.nc"
inline static error_t DemoSensorP$SensorControl$start(void ){
#line 83
  unsigned char __nesc_result;
#line 83

#line 83
  __nesc_result = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$SplitControl$start();
#line 83

#line 83
  return __nesc_result;
#line 83
}
#line 83
# 46 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/DemoSensorP.nc"
static inline error_t DemoSensorP$Read$read(void )
#line 46
{

  if (DemoSensorP$status == 1) {
      DemoSensorP$SensorControl$start();
      DemoSensorP$status = 0;
    }
  else 
#line 51
    {
      DemoSensorP$Light$read();
    }
  return SUCCESS;
}

# 55 "/opt/tinyos-2.1.0/tos/interfaces/Read.nc"
inline static error_t VMC$Read$read(void ){
#line 55
  unsigned char __nesc_result;
#line 55

#line 55
  __nesc_result = DemoSensorP$Read$read();
#line 55

#line 55
  return __nesc_result;
#line 55
}
#line 55
# 178 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline uint32_t /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$getNow(uint8_t num)
{
  return /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow();
}

# 590 "VMC.nc"
static inline uint32_t VMC$Timer$default$getNow(uint8_t id)
#line 590
{
  ;
  return 0;
}

# 125 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static uint32_t VMC$Timer$getNow(uint8_t arg_0x405b2258){
#line 125
  unsigned int __nesc_result;
#line 125

#line 125
  switch (arg_0x405b2258) {
#line 125
    case 0:
#line 125
      __nesc_result = /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$getNow(1U);
#line 125
      break;
#line 125
    case 1:
#line 125
      __nesc_result = /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$getNow(2U);
#line 125
      break;
#line 125
    case 2:
#line 125
      __nesc_result = /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$getNow(3U);
#line 125
      break;
#line 125
    default:
#line 125
      __nesc_result = VMC$Timer$default$getNow(arg_0x405b2258);
#line 125
      break;
#line 125
    }
#line 125

#line 125
  return __nesc_result;
#line 125
}
#line 125
# 73 "VMC.nc"
static inline void VMC$read_sensor(int8_t reg)
#line 73
{

  uint32_t time_now = VMC$Timer$getNow(0);



  VMC$apps[VMC$active_app].regs[reg] = 100;
  ;
  return;




  VMC$apps[VMC$active_app].waiting_sensor = reg;
  VMC$Read$read();
}

# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led2$clr(void ){
#line 30
  HalPXA27xGeneralIOM$GeneralIO$clr(105);
#line 30
}
#line 30
# 93 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led2On(void )
#line 93
{
  LedsP$Led2$clr();
  ;
#line 95
  ;
}

# 78 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void VMC$Leds$led2On(void ){
#line 78
  LedsP$Leds$led2On();
#line 78
}
#line 78
# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led1$clr(void ){
#line 30
  HalPXA27xGeneralIOM$GeneralIO$clr(104);
#line 30
}
#line 30
# 78 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led1On(void )
#line 78
{
  LedsP$Led1$clr();
  ;
#line 80
  ;
}

# 61 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void VMC$Leds$led1On(void ){
#line 61
  LedsP$Leds$led1On();
#line 61
}
#line 61
# 30 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void LedsP$Led0$clr(void ){
#line 30
  HalPXA27xGeneralIOM$GeneralIO$clr(103);
#line 30
}
#line 30
# 63 "/opt/tinyos-2.1.0/tos/system/LedsP.nc"
static inline void LedsP$Leds$led0On(void )
#line 63
{
  LedsP$Led0$clr();
  ;
#line 65
  ;
}

# 45 "/opt/tinyos-2.1.0/tos/interfaces/Leds.nc"
inline static void VMC$Leds$led0On(void ){
#line 45
  LedsP$Leds$led0On();
#line 45
}
#line 45
# 218 "VMC.nc"
static inline void VMC$execute_instructionTask$runTask(void )
#line 218
{

  uint8_t *instr;
  int8_t r;
  app *p = &VMC$apps[VMC$active_app];


  if (VMC$active_app == 3) {
    return;
    }

  if (p->handler == 1) {
    instr = p->init + p->PC;
    }
  else {
#line 232
    instr = p->timer + p->PC;
    }
  if (VMC$apps[VMC$active_app].waiting_sensor != 6) {
    VMC$next_app();
    }


  switch (instr[0] & 0xF0) {


      case 0x00: 

        ;
      p->handler = p->next_handler;
      p->next_handler = 0;
      p->PC = 0;

      break;


      case 0x10: 

        r = instr[0] & 0x0F;
      ;
      p->PC += 2;


      if (r <= 0 || r > 6) {
        break;
        }
      p->regs[r - 1] = (int8_t )instr[1];

      break;


      case 0x20: 

        r = instr[0] & 0x0F;

      ;
      p->PC += 2;


      if (((r <= 0 || r > 6) || instr[1] <= 0) || instr[1] > 6) {
        break;
        }
      p->regs[r - 1] = p->regs[instr[1] - 1];

      break;


      case 0x30: 

        r = instr[0] & 0x0F;

      ;
      p->PC += 2;


      if (((r <= 0 || r > 6) || instr[1] <= 0) || instr[1] > 6) {
        break;
        }
      p->regs[r - 1] += p->regs[instr[1] - 1];

      break;


      case 0x40: 

        r = instr[0] & 0x0F;

      ;
      p->PC += 2;


      if (((r <= 0 || r > 6) || instr[1] <= 0) || instr[1] > 6) {
        break;
        }
      p->regs[r - 1] -= p->regs[instr[1] - 1];

      break;


      case 0x50: 

        r = instr[0] & 0x0F;

      ;
      p->PC += 1;


      if (r <= 0 || r > 6) {
        break;
        }
      p->regs[r - 1]++;

      break;


      case 0x60: 

        r = instr[0] & 0x0F;

      ;
      p->PC += 1;


      if (r <= 0 || r > 6) {
        break;
        }
      p->regs[r - 1]--;

      break;


      case 0x70: 

        r = instr[0] & 0x0F;

      ;
      p->PC += 2;


      if (((r <= 0 || r > 6) || instr[1] <= 0) || instr[1] > 6) {
        break;
        }
      if (p->regs[r - 1] < p->regs[instr[1] - 1]) {
        p->regs[r - 1] = p->regs[instr[1] - 1];
        }
      break;


      case 0x80: 

        r = instr[0] & 0x0F;

      ;
      p->PC += 2;


      if (((r <= 0 || r > 6) || instr[1] <= 0) || instr[1] > 6) {
        break;
        }
      if (p->regs[r - 1] > p->regs[instr[1] - 1]) {
        p->regs[r - 1] = p->regs[instr[1] - 1];
        }
      break;


      case 0x90: 

        r = instr[0] & 0x0F;

      ;
      p->PC++;


      if (r <= 0 || r > 6) {
          p->PC++;
          break;
        }

      if (p->regs[r - 1] > 0) {
        p->PC += (int8_t )instr[1];
        }
      else {
#line 397
        p->PC++;
        }
      break;


      case 0xA0: 

        r = instr[0] & 0x0F;

      ;
      p->PC++;


      if (r <= 0 || r > 6) {
          p->PC++;
          break;
        }

      if (p->regs[r - 1] == 0) {
        p->PC += (int8_t )instr[1];
        }
      else {
#line 418
        p->PC++;
        }
      break;


      case 0xB0: 

        ;
      p->PC += 1 + (int8_t )instr[1];

      break;


      case 0xC0: 

        r = instr[0] & 0x0F;

      ;

      switch (VMC$active_app) {

          case 0: 

            if (r) {
              VMC$Leds$led0On();
              }
            else {
#line 444
              VMC$Leds$led0Off();
              }
          break;

          case 1: 

            if (r) {
              VMC$Leds$led1On();
              }
            else {
#line 453
              VMC$Leds$led1Off();
              }
          break;

          case 2: 

            if (r) {
              VMC$Leds$led2On();
              }
            else {
#line 462
              VMC$Leds$led2Off();
              }
          break;
        }


      p->PC += 1;

      break;


      case 0xD0: 

        r = instr[0] & 0x0F;

      p->PC += 1;


      if (r <= 0 || r > 6) {
        break;
        }
      VMC$read_sensor(r);
      ;

      break;


      case 0xE0: 

        ;

      if (instr[1] == 0) {
        VMC$Timer$stop(VMC$active_app);
        }
      else {
#line 496
        VMC$Timer$startPeriodic(VMC$active_app, instr[1] * 1000);
        }
      p->PC += 2;

      break;
    }



  VMC$next_app();
}

# 107 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static inline void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(uint8_t pin)
{
  *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00024 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00124) = 1 << (pin & 0x1f);
  return;
}

# 72 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPCRbit(uint8_t arg_0x40675010){
#line 72
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPCRbit(arg_0x40675010);
#line 72
}
#line 72
# 56 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask(void )
#line 56
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 57
    {
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$fired();
    }
#line 60
    __nesc_atomic_end(__nesc_atomic); }
}

# 128 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired(void )
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow());
}

# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$fired(void ){
#line 72
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$fired();
#line 72
}
#line 72
# 114 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSMR(uint8_t chnl_id)
{
  uint32_t val;

#line 117
  val = *(chnl_id < 4 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00000 + (uint32_t )(chnl_id << 2)) : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00080 + (uint32_t )((chnl_id - 4) << 2)));
  return val;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSMR(void ){
#line 78
  unsigned int __nesc_result;
#line 78

#line 78
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSMR(4);
#line 78

#line 78
  return __nesc_result;
#line 78
}
#line 78
# 156 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline uint32_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm(void )
#line 156
{
  return /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSMR();
}

# 105 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getAlarm(void ){
#line 105
  unsigned int __nesc_result;
#line 105

#line 105
  __nesc_result = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$getAlarm();
#line 105

#line 105
  return __nesc_result;
#line 105
}
#line 105
#line 92
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$startAt(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type t0, /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$size_type dt){
#line 92
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(t0, dt);
#line 92
}
#line 92
# 47 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(uint32_t t0, uint32_t dt, bool oneshot)
{
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_dt = dt;
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_oneshot = oneshot;
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$startAt(t0, dt);
}










static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask(void )
{
  if (/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_oneshot == FALSE) {
    /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(/*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$getAlarm(), /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$m_dt, FALSE);
    }
#line 67
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$fired();
}

# 71 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSMR(uint32_t val){
#line 71
  HplPXA27xOSTimerM$PXA27xOST$setOSMR(4, val);
#line 71
}
#line 71
# 138 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static inline bool HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(uint8_t chnl_id)
{
  bool bFlag = FALSE;

  if ((* (volatile uint32_t *)0x40A00014 & (1 << chnl_id)) != 0) {
      bFlag = TRUE;
    }

  return bFlag;
}

# 103 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static bool /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSSRbit(void ){
#line 103
  unsigned char __nesc_result;
#line 103

#line 103
  __nesc_result = HplPXA27xOSTimerM$PXA27xOST$getOSSRbit(4);
#line 103

#line 103
  return __nesc_result;
#line 103
}
#line 103
# 56 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
inline static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$postTask(void ){
#line 56
  unsigned char __nesc_result;
#line 56

#line 56
  __nesc_result = SchedulerBasicP$TaskBasic$postTask(/*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm);
#line 56

#line 56
  return __nesc_result;
#line 56
}
#line 56
# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void PMICM$chargeMonitorTimer$stop(void ){
#line 67
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$stop(0U);
#line 67
}
#line 67
# 355 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline error_t PMICM$PMIC$getBatteryVoltage(uint8_t *val)
#line 355
{

  return PMICM$getPMICADCVal(0, val);
}

# 53 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void PMICM$chargeMonitorTimer$startPeriodic(uint32_t dt){
#line 53
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$startPeriodic(0U, dt);
#line 53
}
#line 53
# 372 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static inline error_t PMICM$PMIC$enableManualCharging(bool enable)
#line 372
{

  uint8_t val;

  if (enable) {

      PMICM$getPMICADCVal(2, &val);

      if (val > 75) {


          PMICM$writePMIC(0x2A, 8);

          PMICM$writePMIC(0x28, ((1 << 7) | ((1 & 0xF) << 3)) | (7 & 0x7));

          PMICM$writePMIC(0x20, 0x80);

          PMICM$chargeMonitorTimer$startPeriodic(300000);
        }
      else {
        }
    }
  else 
    {

      PMICM$PMIC$getBatteryVoltage(&val);


      PMICM$writePMIC(0x2A, 0);
      PMICM$writePMIC(0x28, 0);
      PMICM$writePMIC(0x20, 0x00);
    }
  return SUCCESS;
}

static inline void PMICM$chargeMonitorTimer$fired(void )
#line 407
{
  uint8_t val;

#line 409
  PMICM$PMIC$getBatteryVoltage(&val);

  if (val > 130) {
      PMICM$PMIC$enableManualCharging(FALSE);
      PMICM$chargeMonitorTimer$stop();
    }
  return;
}

# 193 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(uint8_t num)
{
}

# 72 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$fired(uint8_t arg_0x403d9ce0){
#line 72
  switch (arg_0x403d9ce0) {
#line 72
    case 0U:
#line 72
      PMICM$chargeMonitorTimer$fired();
#line 72
      break;
#line 72
    case 1U:
#line 72
      VMC$Timer$fired(0);
#line 72
      break;
#line 72
    case 2U:
#line 72
      VMC$Timer$fired(1);
#line 72
      break;
#line 72
    case 3U:
#line 72
      VMC$Timer$fired(2);
#line 72
      break;
#line 72
    default:
#line 72
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$default$fired(arg_0x403d9ce0);
#line 72
      break;
#line 72
    }
#line 72
}
#line 72
# 115 "/opt/tinyos-2.1.0/tos/interfaces/Packet.nc"
inline static void * VMC$SerialPacket$getPayload(message_t * msg, uint8_t len){
#line 115
  void *__nesc_result;
#line 115

#line 115
  __nesc_result = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$getPayload(msg, len);
#line 115

#line 115
  return __nesc_result;
#line 115
}
#line 115
# 115 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline uint8_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$maxPayloadLength(void )
#line 115
{
  return 100;
}

# 240 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint8_t __nesc_hton_uint8(void * target, uint8_t value)
#line 240
{
  uint8_t *base = target;

#line 242
  base[0] = value;
  return value;
}

# 111 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$setPayloadLength(message_t *msg, uint8_t len)
#line 111
{
  __nesc_hton_uint8(/*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(msg)->length.data, len);
}

# 83 "/opt/tinyos-2.1.0/tos/interfaces/Packet.nc"
inline static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$setPayloadLength(message_t * msg, uint8_t len){
#line 83
  /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$setPayloadLength(msg, len);
#line 83
}
#line 83
# 82 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
static inline error_t /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(uint8_t clientId, message_t *msg, 
uint8_t len)
#line 83
{
  if (clientId >= 1) {
      return FAIL;
    }
  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[clientId].msg != (void *)0) {
      return EBUSY;
    }
  ;

  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[clientId].msg = msg;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Packet$setPayloadLength(msg, len);

  if (/*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current >= 1) {
      error_t err;
      am_id_t amId = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$type(msg);
      am_addr_t dest = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMPacket$destination(msg);

      ;
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current = clientId;

      err = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$AMSend$send(amId, dest, msg, len);
      if (err != SUCCESS) {
          ;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$current = 1;
          /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[clientId].msg = (void *)0;
        }

      return err;
    }
  else {
      ;
    }
  return SUCCESS;
}

# 64 "/opt/tinyos-2.1.0/tos/interfaces/Send.nc"
inline static error_t /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$send(message_t * msg, uint8_t len){
#line 64
  unsigned char __nesc_result;
#line 64

#line 64
  __nesc_result = /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$send(0U, msg, len);
#line 64

#line 64
  return __nesc_result;
#line 64
}
#line 64
# 161 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setType(message_t *amsg, am_id_t type)
#line 161
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(amsg);

#line 163
  __nesc_hton_uint8(header->type.data, type);
}

# 151 "/opt/tinyos-2.1.0/tos/interfaces/AMPacket.nc"
inline static void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setType(message_t * amsg, am_id_t t){
#line 151
  /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setType(amsg, t);
#line 151
}
#line 151
# 269 "/usr/lib/ncc/nesc_nx.h"
static __inline  uint16_t __nesc_hton_uint16(void * target, uint16_t value)
#line 269
{
  uint8_t *base = target;

#line 271
  base[1] = value;
  base[0] = value >> 8;
  return value;
}

# 142 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static inline void /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setDestination(message_t *amsg, am_addr_t addr)
#line 142
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(amsg);

#line 144
  __nesc_hton_uint16(header->dest.data, addr);
}

# 92 "/opt/tinyos-2.1.0/tos/interfaces/AMPacket.nc"
inline static void /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setDestination(message_t * amsg, am_addr_t addr){
#line 92
  /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$setDestination(amsg, addr);
#line 92
}
#line 92
# 45 "/opt/tinyos-2.1.0/tos/system/AMQueueEntryP.nc"
static inline error_t /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 47
{
  /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setDestination(msg, dest);
  /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMPacket$setType(msg, 6);
  return /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$Send$send(msg, len);
}

# 69 "/opt/tinyos-2.1.0/tos/interfaces/AMSend.nc"
inline static error_t VMC$SerialAMSend$send(am_addr_t addr, message_t * msg, uint8_t len){
#line 69
  unsigned char __nesc_result;
#line 69

#line 69
  __nesc_result = /*VMCApp.SerialAMSenderC.AMQueueEntryP*/AMQueueEntryP$0$AMSend$send(addr, msg, len);
#line 69

#line 69
  return __nesc_result;
#line 69
}
#line 69
# 82 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(uint32_t t0, uint32_t dt)
{
#line 83
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$start(t0, dt, TRUE);
}

# 118 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(uint32_t t0, uint32_t dt){
#line 118
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$startOneShotAt(t0, dt);
#line 118
}
#line 118
# 117 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop(void )
#line 117
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 118
    {
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(FALSE);
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
    }
#line 121
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/Alarm.nc"
inline static void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$stop(void ){
#line 62
  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$stop();
#line 62
}
#line 62
# 60 "/opt/tinyos-2.1.0/tos/lib/timer/AlarmToTimerC.nc"
static inline void /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop(void )
{
#line 61
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Alarm$stop();
}

# 67 "/opt/tinyos-2.1.0/tos/lib/timer/Timer.nc"
inline static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$stop(void ){
#line 67
  /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$Timer$stop();
#line 67
}
#line 67
# 89 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static inline void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask(void )
{




  uint32_t now = /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$getNow();
  int32_t min_remaining = (1UL << 31) - 1;
  bool min_remaining_isset = FALSE;
  uint8_t num;

  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$stop();

  for (num = 0; num < /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;
          int32_t remaining = timer->dt - elapsed;

          if (remaining < min_remaining) 
            {
              min_remaining = remaining;
              min_remaining_isset = TRUE;
            }
        }
    }

  if (min_remaining_isset) 
    {
      if (min_remaining <= 0) {
        /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(now);
        }
      else {
#line 124
        /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$TimerFrom$startOneShotAt(now, min_remaining);
        }
    }
}

# 57 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSCR(uint32_t val){
#line 57
  HplPXA27xOSTimerM$PXA27xOST$setOSCR(4, val);
#line 57
}
#line 57
#line 85
inline static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOMCR(uint32_t val){
#line 85
  HplPXA27xOSTimerM$PXA27xOST$setOMCR(4, val);
#line 85
}
#line 85
# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = HplPXA27xOSTimerM$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 63 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static inline error_t /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init(void )
#line 63
{

  /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTInit$init();
  /* atomic removed: atomic calls only */
  {
    /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = FALSE;
    switch (2) {
        case 1: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 10;
        break;
        case 2: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 1;
        break;
        case 3: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 1;
        break;
        case 4: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 300;
        break;
        default: 
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mMinDeltaT = 0;
        break;
      }
    /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOMCR(((1 << 7) | (1 << 6)) | (((2 & 0x8) << 5) | ((2 & 0x7) << 0)));
    /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSCR(0);
  }
  return SUCCESS;
}

# 101 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(uint8_t arg_0x40675010, bool flag){
#line 101
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(arg_0x40675010, flag);
#line 101
}
#line 101
#line 83
inline static void HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(uint8_t arg_0x40675010, bool flag){
#line 83
  HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(arg_0x40675010, flag);
#line 83
}
#line 83
# 105 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(uint8_t pin)
#line 105
{
  /* atomic removed: atomic calls only */
#line 106
  {
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(pin, TRUE);
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(pin, FALSE);
  }
  return SUCCESS;
}

#line 138
static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(uint8_t pin)
#line 138
{
  return HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableRisingEdge(pin);
}

# 42 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
inline static error_t Tsl2561InternalP$InterruptAlert$enableRisingEdge(void ){
#line 42
  unsigned char __nesc_result;
#line 42

#line 42
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$enableRisingEdge(99);
#line 42

#line 42
  return __nesc_result;
#line 42
}
#line 42
# 113 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline error_t HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(uint8_t pin)
#line 113
{
  /* atomic removed: atomic calls only */
#line 114
  {
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGRERbit(pin, FALSE);
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGFERbit(pin, TRUE);
  }
  return SUCCESS;
}

#line 142
static inline error_t HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(uint8_t pin)
#line 142
{
  return HalPXA27xGeneralIOM$HalPXA27xGpioInterrupt$enableFallingEdge(pin);
}

# 43 "/opt/tinyos-2.1.0/tos/interfaces/GpioInterrupt.nc"
inline static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptAlert$enableFallingEdge(void ){
#line 43
  unsigned char __nesc_result;
#line 43

#line 43
  __nesc_result = HalPXA27xGeneralIOM$GpioInterrupt$enableFallingEdge(99);
#line 43

#line 43
  return __nesc_result;
#line 43
}
#line 43
# 83 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static inline void HalPXA27xGeneralIOM$GeneralIO$makeInput(uint8_t pin)
#line 83
{
  /* atomic removed: atomic calls only */
#line 84
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPDRbit(pin, FALSE);
  return;
}

# 33 "/opt/tinyos-2.1.0/tos/interfaces/GeneralIO.nc"
inline static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptPin$makeInput(void ){
#line 33
  HalPXA27xGeneralIOM$GeneralIO$makeInput(99);
#line 33
}
#line 33
# 154 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static inline error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$Init$init(void )
#line 154
{
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptPin$makeInput();
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$InterruptAlert$enableFallingEdge();
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STOPPED;
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$interruptBit = FALSE;
  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t Tsl2561InternalP$SubInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$Init$init();
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 47 "/opt/tinyos-2.1.0/tos/sensorboards/im2sb/Tsl2561InternalP.nc"
static inline error_t Tsl2561InternalP$Init$init(void )
#line 47
{
  Tsl2561InternalP$SubInit$init();


  Tsl2561InternalP$InterruptAlert$enableRisingEdge();
  return SUCCESS;
}

# 45 "/opt/tinyos-2.1.0/tos/system/FcfsResourceQueueC.nc"
static inline error_t /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init(void )
#line 45
{
  memset(/*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$resQ, /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$NO_ENTRY, sizeof /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$resQ);
  return SUCCESS;
}

# 214 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static __inline void SerialP$ackInit(void )
#line 214
{
  SerialP$ackQ.writePtr = SerialP$ackQ.readPtr = 0;
}

#line 205
static __inline void SerialP$rxInit(void )
#line 205
{
  SerialP$rxBuf.writePtr = SerialP$rxBuf.readPtr = 0;
  SerialP$rxState = SerialP$RXSTATE_NOSYNC;
  SerialP$rxByteCnt = 0;
  SerialP$rxProto = 0;
  SerialP$rxSeqno = 0;
  SerialP$rxCRC = 0;
}

#line 193
static __inline void SerialP$txInit(void )
#line 193
{
  uint8_t i;

  /* atomic removed: atomic calls only */
#line 195
  for (i = 0; i < SerialP$TX_BUFFER_COUNT; i++) SerialP$txBuf[i].state = SerialP$BUFFER_AVAILABLE;
  SerialP$txState = SerialP$TXSTATE_IDLE;
  SerialP$txByteCnt = 0;
  SerialP$txProto = 0;
  SerialP$txSeqno = 0;
  SerialP$txCRC = 0;
  SerialP$txPending = FALSE;
  SerialP$txIndex = 0;
}

#line 218
static inline error_t SerialP$Init$init(void )
#line 218
{

  SerialP$txInit();
  SerialP$rxInit();
  SerialP$ackInit();

  return SUCCESS;
}

# 51 "/opt/tinyos-2.1.0/tos/interfaces/Init.nc"
inline static error_t RealMainP$SoftwareInit$init(void ){
#line 51
  unsigned char __nesc_result;
#line 51

#line 51
  __nesc_result = SerialP$Init$init();
#line 51
  __nesc_result = ecombine(__nesc_result, /*Tsl2561InternalC.Arbiter.Queue*/FcfsResourceQueueC$0$Init$init());
#line 51
  __nesc_result = ecombine(__nesc_result, Tsl2561InternalP$Init$init());
#line 51
  __nesc_result = ecombine(__nesc_result, /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Init$init());
#line 51

#line 51
  return __nesc_result;
#line 51
}
#line 51
# 40 "VMC.nc"
static inline void VMC$system_init(void )
#line 40
{

  uint8_t i;

#line 43
  for (i = 0; i < 3; i++) {

      VMC$apps[i].isActive = FALSE;
      VMC$apps[i].handler = 0;
      VMC$apps[i].id = 0;
    }
}

#line 510
static inline void VMC$Boot$booted(void )
#line 510
{

  VMC$system_init();
  VMC$SerialAMControl$start();

  VMC$Timer$startPeriodic(0, 1000);
}

# 49 "/opt/tinyos-2.1.0/tos/interfaces/Boot.nc"
inline static void RealMainP$Boot$booted(void ){
#line 49
  VMC$Boot$booted();
#line 49
}
#line 49
# 143 "/opt/tinyos-2.1.0/tos/chips/pxa27x/pxa27xhardware.h"
static __inline void __nesc_disable_interrupt()
#line 143
{

  uint32_t statusReg = 0;

   __asm volatile (
  "mrs %0,CPSR\n\t"
  "orr %0,%1,#0xc0\n\t"
  "msr CPSR_c,%1\n\t" : 
  "=r"(statusReg) : 
  "0"(statusReg));

  return;
}

#line 129
static __inline void __nesc_enable_interrupt()
#line 129
{

  uint32_t statusReg = 0;

   __asm volatile (
  "mrs %0,CPSR\n\t"
  "bic %0,%1,#0xc0\n\t"
  "msr CPSR_c, %1" : 
  "=r"(statusReg) : 
  "0"(statusReg));

  return;
}

# 53 "/opt/tinyos-2.1.0/tos/chips/pxa27x/McuSleepC.nc"
static inline void McuSleepC$McuSleep$sleep(void )
#line 53
{

   __asm volatile (
  "mcr p14,0,%0,c7,c0,0" :  : 

  "r"(1));

  __nesc_enable_interrupt();

   __asm volatile ("" :  :  : "memory");
  __nesc_disable_interrupt();
  return;
}

# 59 "/opt/tinyos-2.1.0/tos/interfaces/McuSleep.nc"
inline static void SchedulerBasicP$McuSleep$sleep(void ){
#line 59
  McuSleepC$McuSleep$sleep();
#line 59
}
#line 59
# 67 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static __inline uint8_t SchedulerBasicP$popTask(void )
{
  if (SchedulerBasicP$m_head != SchedulerBasicP$NO_TASK) 
    {
      uint8_t id = SchedulerBasicP$m_head;

#line 72
      SchedulerBasicP$m_head = SchedulerBasicP$m_next[SchedulerBasicP$m_head];
      if (SchedulerBasicP$m_head == SchedulerBasicP$NO_TASK) 
        {
          SchedulerBasicP$m_tail = SchedulerBasicP$NO_TASK;
        }
      SchedulerBasicP$m_next[id] = SchedulerBasicP$NO_TASK;
      return id;
    }
  else 
    {
      return SchedulerBasicP$NO_TASK;
    }
}

#line 138
static inline void SchedulerBasicP$Scheduler$taskLoop(void )
{
  for (; ; ) 
    {
      uint8_t nextTask;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
        {
          while ((nextTask = SchedulerBasicP$popTask()) == SchedulerBasicP$NO_TASK) 
            {
              SchedulerBasicP$McuSleep$sleep();
            }
        }
#line 150
        __nesc_atomic_end(__nesc_atomic); }
      SchedulerBasicP$TaskBasic$runTask(nextTask);
    }
}

# 61 "/opt/tinyos-2.1.0/tos/interfaces/Scheduler.nc"
inline static void RealMainP$Scheduler$taskLoop(void ){
#line 61
  SchedulerBasicP$Scheduler$taskLoop();
#line 61
}
#line 61
# 68 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
__attribute((interrupt("IRQ")))   void hplarmv_irq(void )
#line 68
{

  uint32_t IRQPending;

  IRQPending = HplPXA27xInterruptM$getICHP();
  IRQPending >>= 16;

  while (IRQPending & (1 << 15)) {
      uint8_t PeripheralID = IRQPending & 0x3f;

#line 77
      HplPXA27xInterruptM$PXA27xIrq$fired(PeripheralID);
      IRQPending = HplPXA27xInterruptM$getICHP();
      IRQPending >>= 16;
    }

  return;
}

# 326 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamSendSignal(void )
#line 326
{
  uint8_t *pBuf = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentBuf;
  uint16_t len = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentLen;

#line 329
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentBuf = (void *)0;
  if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamSendIF) {
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamSendIF = FALSE;
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$sendDone(pBuf, len, SUCCESS);
    }
  else {
      pBuf = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$sendDone(pBuf, len, SUCCESS);
      if (pBuf) {
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$send(pBuf, len);
        }
    }
  return;
}

#line 161
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$send(uint8_t *buf, uint16_t len)
#line 161
{
  error_t error;

#line 163
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 163
    /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamSendIF = TRUE;
#line 163
    __nesc_atomic_end(__nesc_atomic); }
  error = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$send(buf, len);
  if (error) {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 166
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamSendIF = FALSE;
#line 166
        __nesc_atomic_end(__nesc_atomic); }
    }
  return error;
}

#line 204
static error_t /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$send(uint8_t *buf, uint16_t len)
#line 204
{
  uint32_t txAddr;
  uint32_t DMAFlags;
  error_t error = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 209
    {
      if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentBuf == (void *)0) {
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentBuf = buf;
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$txCurrentLen = len;
        }
      else {
          error = FAIL;
        }
    }
#line 217
    __nesc_atomic_end(__nesc_atomic); }

  if (error) {
    return error;
    }
  if (len < 8) {
      uint16_t i;

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 225
        {
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow |= 1 << 3;
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setFCR(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gulFCRShadow);
        }
#line 228
        __nesc_atomic_end(__nesc_atomic); }
      for (i = 0; i < len; i++) {
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setTHR(buf[i]);
        }
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 232
        /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER() | (1 << 1));
#line 232
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {

      DMAFlags = ((((1 << 28) | (1 << 16)) | (1 << 14)) | (1 << 21))
       | (len & 0x1fff);

      txAddr = (uint32_t )buf;
      DMAFlags |= 1 << 31;

      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCSR(1 << 30);
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDSADR(txAddr);
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDTADR(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UARTTxDMAInfo$getAddr());
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCMD(DMAFlags);

      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 247
        {
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$setIER(/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UART$getIER() | (1 << 7));
        }
#line 249
        __nesc_atomic_end(__nesc_atomic); }

      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$TxDMA$setDCSR((1 << 31) | (1 << 30));
    }
  return error;
}

# 92 "/opt/tinyos-2.1.0/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC$SerialFrameComm$putData(uint8_t data)
#line 92
{
  if (data == HDLC_CTLESC_BYTE || data == HDLC_FLAG_BYTE) {
      HdlcTranslateC$state.sendEscape = 1;
      HdlcTranslateC$txTemp = data ^ 0x20;
      HdlcTranslateC$m_data = HDLC_CTLESC_BYTE;
    }
  else {
      HdlcTranslateC$m_data = data;
    }
  return HdlcTranslateC$UartStream$send(&HdlcTranslateC$m_data, 1);
}

# 80 "/opt/tinyos-2.1.0/tos/system/crc.h"
static uint16_t crcByte(uint16_t crc, uint8_t b)
#line 80
{
  crc = (uint8_t )(crc >> 8) | (crc << 8);
  crc ^= b;
  crc ^= (uint8_t )(crc & 0xff) >> 4;
  crc ^= crc << 12;
  crc ^= (crc & 0xff) << 5;
  return crc;
}

# 86 "/opt/tinyos-2.1.0/tos/lib/serial/HdlcTranslateC.nc"
static error_t HdlcTranslateC$SerialFrameComm$putDelimiter(void )
#line 86
{
  HdlcTranslateC$state.sendEscape = 0;
  HdlcTranslateC$m_data = HDLC_FLAG_BYTE;
  return HdlcTranslateC$UartStream$send(&HdlcTranslateC$m_data, 1);
}

# 502 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static void SerialP$MaybeScheduleTx(void )
#line 502
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 503
    {
      if (SerialP$txPending == 0) {
          if (SerialP$RunTx$postTask() == SUCCESS) {
              SerialP$txPending = 1;
            }
        }
    }
#line 509
    __nesc_atomic_end(__nesc_atomic); }
}

# 159 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static error_t SchedulerBasicP$TaskBasic$postTask(uint8_t id)
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 161
    {
#line 161
      {
        unsigned char __nesc_temp = 
#line 161
        SchedulerBasicP$pushTask(id) ? SUCCESS : EBUSY;

        {
#line 161
          __nesc_atomic_end(__nesc_atomic); 
#line 161
          return __nesc_temp;
        }
      }
    }
#line 164
    __nesc_atomic_end(__nesc_atomic); }
}

# 309 "/opt/tinyos-2.1.0/tos/chips/pxa27x/uart/HalPXA27xSerialP.nc"
static void /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$DispatchStreamRcvSignal(void )
#line 309
{
  uint8_t *pBuf = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf;
  uint16_t len = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentLen;

#line 312
  /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$rxCurrentBuf = (void *)0;
  if (/*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamRcvIF) {
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$gbUsingUartStreamRcvIF = FALSE;
      /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$UartStream$receiveDone(pBuf, len, SUCCESS);
    }
  else {
      pBuf = /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$receiveDone(pBuf, len, SUCCESS);
      if (pBuf) {
          /*PlatformSerialC.HalPXA27xSerialP*/HalPXA27xSerialP$0$HalPXA27xSerialPacket$receive(pBuf, len, 0);
        }
    }
  return;
}

# 402 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static void SerialP$rx_state_machine(bool isDelimeter, uint8_t data)
#line 402
{

  switch (SerialP$rxState) {

      case SerialP$RXSTATE_NOSYNC: 
        if (isDelimeter) {
            SerialP$rxInit();
            SerialP$rxState = SerialP$RXSTATE_PROTO;
          }
      break;

      case SerialP$RXSTATE_PROTO: 
        if (!isDelimeter) {
            SerialP$rxCRC = crcByte(SerialP$rxCRC, data);
            SerialP$rxState = SerialP$RXSTATE_TOKEN;
            SerialP$rxProto = data;
            if (!SerialP$valid_rx_proto(SerialP$rxProto)) {
              goto nosync;
              }
            if (SerialP$rxProto != SERIAL_PROTO_PACKET_ACK) {
                goto nosync;
              }
            if (SerialP$ReceiveBytePacket$startPacket() != SUCCESS) {
                goto nosync;
              }
          }
      break;

      case SerialP$RXSTATE_TOKEN: 
        if (isDelimeter) {
            goto nosync;
          }
        else {
            SerialP$rxSeqno = data;
            SerialP$rxCRC = crcByte(SerialP$rxCRC, SerialP$rxSeqno);
            SerialP$rxState = SerialP$RXSTATE_INFO;
          }
      break;

      case SerialP$RXSTATE_INFO: 
        if (SerialP$rxByteCnt < SerialP$SERIAL_MTU) {
            if (isDelimeter) {
                if (SerialP$rxByteCnt >= 2) {
                    if (SerialP$rx_current_crc() == SerialP$rxCRC) {
                        SerialP$ReceiveBytePacket$endPacket(SUCCESS);
                        SerialP$ack_queue_push(SerialP$rxSeqno);
                        goto nosync;
                      }
                    else {
                        goto nosync;
                      }
                  }
                else {
                    goto nosync;
                  }
              }
            else {
                if (SerialP$rxByteCnt >= 2) {
                    SerialP$ReceiveBytePacket$byteReceived(SerialP$rx_buffer_top());
                    SerialP$rxCRC = crcByte(SerialP$rxCRC, SerialP$rx_buffer_pop());
                  }
                SerialP$rx_buffer_push(data);
                SerialP$rxByteCnt++;
              }
          }
        else 

          {
            goto nosync;
          }
      break;

      default: 
        goto nosync;
    }
  goto done;

  nosync: 

    SerialP$rxInit();
  SerialP$SerialFrameComm$resetReceive();
  SerialP$ReceiveBytePacket$endPacket(FAIL);
  if (SerialP$offPending) {
      SerialP$rxState = SerialP$RXSTATE_INACTIVE;
      SerialP$testOff();
    }
  else {
    if (isDelimeter) {
        SerialP$rxState = SerialP$RXSTATE_PROTO;
      }
    }
  done: ;
}

# 285 "/opt/tinyos-2.1.0/tos/lib/serial/SerialDispatcherP.nc"
static void /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$ReceiveBytePacket$endPacket(error_t result)
#line 285
{
  uint8_t postsignalreceive = FALSE;

  /* atomic removed: atomic calls only */
#line 287
  {
    if (!/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending && result == SUCCESS) {
        postsignalreceive = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskPending = TRUE;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskType = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvType;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskWhich = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.which;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskBuf = (message_t *)/*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBuffer;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTaskSize = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$recvIndex;
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveBufferSwap();
        /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveState.state = /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$RECV_STATE_IDLE;
      }
  }
  if (postsignalreceive) {
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$postTask();
    }
}

# 347 "/opt/tinyos-2.1.0/tos/lib/serial/SerialP.nc"
static void SerialP$testOff(void )
#line 347
{
  bool turnOff = FALSE;

#line 349
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 349
    {
      if (SerialP$txState == SerialP$TXSTATE_INACTIVE && 
      SerialP$rxState == SerialP$RXSTATE_INACTIVE) {
          turnOff = TRUE;
        }
    }
#line 354
    __nesc_atomic_end(__nesc_atomic); }
  if (turnOff) {
      SerialP$stopDoneTask$postTask();
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 357
        SerialP$offPending = FALSE;
#line 357
        __nesc_atomic_end(__nesc_atomic); }
    }
  else {
      { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 360
        SerialP$offPending = TRUE;
#line 360
        __nesc_atomic_end(__nesc_atomic); }
    }
}

# 73 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$readNextByte(void )
#line 73
{
  if (/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex >= /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen - 1) {
      /* atomic removed: atomic calls only */
#line 75
      {
#line 75
        /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_READEND;
      }
#line 76
      if (/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurFlags & I2C_STOP) {
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (((((1 << 12) | (1 << 9)) | (1 << 2)) | (1 << 3)) | (1 << 1)));
        }
      else {
#line 79
        if (/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurFlags & I2C_ACK_END) {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (((1 << 12) | (1 << 9)) | (1 << 3)));
          }
        else {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | ((((1 << 12) | (1 << 9)) | (1 << 2)) | (1 << 3)));
          }
        }
    }
  else 
#line 86
    {
      /* atomic removed: atomic calls only */
#line 87
      {
#line 87
        /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_READ;
      }
#line 88
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (((1 << 12) | (1 << 9)) | (1 << 3)));
    }
  return;
}

# 107 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setICR(uint32_t val)
#line 107
{
  switch (0) {
      case 0: * (volatile uint32_t *)0x40301690 = val;
#line 109
      break;
      case 1: * (volatile uint32_t *)0x40F00190 = val;
#line 110
      break;
      default: break;
    }
  return;
}

#line 99
static uint32_t /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$getIDBR(void )
#line 99
{
  switch (0) {
      case 0: return * (volatile uint32_t *)0x40301688;
#line 101
      break;
      case 1: return * (volatile uint32_t *)0x40F00188;
#line 102
      break;
      default: return 0;
    }
}

# 219 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$readDone(error_t i2c_error, uint16_t chipAddr, uint8_t len, uint8_t *buf)
#line 219
{
  uint16_t tempVal;
  error_t error = i2c_error;

  switch (/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState) {
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READCH0: 
        tempVal = buf[1];
      tempVal = (tempVal << 8) | buf[0];
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh0Done(error, tempVal);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READCH1: 
        tempVal = buf[1];
      tempVal = (tempVal << 8) | buf[0];
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$measureCh1Done(error, tempVal);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READID: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$getIDDone(error, buf[0]);
      break;
      default: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      break;
    }
  return;
}

# 90 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setIDBR(uint32_t val)
#line 90
{
  switch (0) {
      case 0: * (volatile uint32_t *)0x40301688 = val;
#line 92
      break;
      case 1: * (volatile uint32_t *)0x40F00188 = val;
#line 93
      break;
      default: break;
    }
  return;
}

# 93 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static void /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$writeNextByte(void )
#line 93
{
  if (/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex >= /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen) {
      /* atomic removed: atomic calls only */
#line 95
      {
#line 95
        /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_WRITEEND;
      }
      if (/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurFlags & I2C_STOP) {
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | ((((1 << 12) | (1 << 3)) | (1 << 8)) | (1 << 1)));
        }
      else 
        {
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (((1 << 12) | (1 << 8)) | (1 << 3)));
        }
    }
  else 
    {
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (((1 << 12) | (1 << 8)) | (1 << 3)));
    }
  return;
}

# 247 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static void /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$writeDone(error_t i2c_error, uint16_t chipAddr, uint8_t len, uint8_t *buf)
#line 247
{
  error_t error = i2c_error;

  switch (/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState) {
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STARTING: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mSSError = error;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone$postTask();
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STOPPING: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mSSError = error;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STOPPED;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone$postTask();
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READCH0: 
        error = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$read(I2C_START | I2C_STOP, 73U, 2, /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READCH1: 
        error = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$read(I2C_START | I2C_STOP, 73U, 2, /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_SETCONTROL: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setCONTROLDone(error);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_SETTIMING: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTIMINGDone(error);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_SETINTERRUPT: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setINTERRUPTDone(error);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_SETHIGH: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTHRESHHIGHDone(error);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_SETLOW: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$HplTSL256x$setTHRESHLOWDone(error);
      break;
      case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_READID: 
        error = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$read(I2C_STOP, 73U, 1, /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer);
      break;
      default: 
        /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
      break;
    }
  return;
}

# 199 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HalPXA27xI2CMasterP.nc"
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CPacket$read(i2c_flags_t flags, uint16_t addr, uint8_t length, uint8_t *data)
#line 199
{
  error_t error = SUCCESS;

  if (flags & I2C_ACK_END && flags & I2C_STOP) {
      error = EINVAL;
      return error;
    }

  if (flags & I2C_START) {
      error = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$startI2CTransact(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_READSTART, addr, length, data, flags, TRUE);
    }
  else {
      error = /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$startI2CTransact(/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_READ, addr, length, data, flags, TRUE);
    }

  return error;
}

#line 112
static error_t /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$startI2CTransact(uint8_t nextState, uint16_t addr, uint8_t length, uint8_t *data, 
i2c_flags_t flags, bool bRnW)
#line 113
{
  error_t error = SUCCESS;
  uint8_t tmpAddr;

  if (data == (void *)0 || length == 0) {
      return EINVAL;
    }

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 121
    {
      if (/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState == /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C_STATE_IDLE) {
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mI2CState = nextState;
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurTargetAddr = addr;
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBuf = data;
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufLen = length;
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurBufIndex = 0;
          /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mCurFlags = flags;
        }
      else {
          error = EBUSY;
        }
    }
#line 133
    __nesc_atomic_end(__nesc_atomic); }
  if (error) {
      return error;
    }

  if (flags & I2C_START) {

      tmpAddr = bRnW ? 0x1 : 0x0;
      tmpAddr |= (addr << 1) & 0xFE;
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setIDBR(tmpAddr);
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2C$setICR(((/*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$mBaseICRFlags | (1 << 8)) | (1 << 3)) | (1 << 0));
    }
  else {
#line 145
    if (bRnW) {
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 146
          {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$readNextByte();
          }
#line 148
          __nesc_atomic_end(__nesc_atomic); }
      }
    else {
        { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 151
          {
            /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$writeNextByte();
          }
#line 153
          __nesc_atomic_end(__nesc_atomic); }
      }
    }
#line 155
  return error;
}

# 150 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOPin.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$interruptGPIOPin(uint8_t arg_0x4050cc50){
#line 150
  HalPXA27xGeneralIOM$HplPXA27xGPIOPin$interruptGPIOPin(arg_0x4050cc50);
#line 150
  switch (arg_0x4050cc50) {
#line 150
    case 1:
#line 150
      PMICM$PMICGPIO$interruptGPIOPin();
#line 150
      break;
#line 150
    case 46:
#line 150
      IM2InitSerialP$RXD$interruptGPIOPin();
#line 150
      break;
#line 150
    case 47:
#line 150
      IM2InitSerialP$TXD$interruptGPIOPin();
#line 150
      break;
#line 150
    case 117:
#line 150
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSCL$interruptGPIOPin();
#line 150
      break;
#line 150
    case 118:
#line 150
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$I2CSDA$interruptGPIOPin();
#line 150
      break;
#line 150
  }
#line 150
}
#line 150
# 104 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static error_t PMICM$readPMIC(uint8_t address, uint8_t *value, uint8_t numBytes)
#line 104
{

  if (numBytes > 0) {
      PMICM$PI2C$setIDBR(0x49 << 1);
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 0));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;


      PMICM$PI2C$setIDBR(address);
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 0));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 1));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 1));


      PMICM$PI2C$setIDBR((0x49 << 1) | 1);
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 0));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 0));


      while (numBytes > 1) {
          PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
          while (PMICM$PI2C$getICR() & (1 << 3)) ;
          *value = PMICM$PI2C$getIDBR();
          value++;
          numBytes--;
        }

      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 1));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 2));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
      while (PMICM$PI2C$getICR() & (1 << 3)) ;
      *value = PMICM$PI2C$getIDBR();
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 1));
      PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 2));

      return SUCCESS;
    }
  else {
      return FAIL;
    }
}

# 90 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setIDBR(uint32_t val)
#line 90
{
  switch (1) {
      case 0: * (volatile uint32_t *)0x40301688 = val;
#line 92
      break;
      case 1: * (volatile uint32_t *)0x40F00188 = val;
#line 93
      break;
      default: break;
    }
  return;
}









static void /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$setICR(uint32_t val)
#line 107
{
  switch (1) {
      case 0: * (volatile uint32_t *)0x40301690 = val;
#line 109
      break;
      case 1: * (volatile uint32_t *)0x40F00190 = val;
#line 110
      break;
      default: break;
    }
  return;
}

static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getICR(void )
#line 116
{
  switch (1) {
      case 0: return * (volatile uint32_t *)0x40301690;
#line 118
      break;
      case 1: return * (volatile uint32_t *)0x40F00190;
#line 119
      break;
      default: return 0;
    }
}

#line 99
static uint32_t /*HplPXA27xPI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$0$I2C$getIDBR(void )
#line 99
{
  switch (1) {
      case 0: return * (volatile uint32_t *)0x40301688;
#line 101
      break;
      case 1: return * (volatile uint32_t *)0x40F00188;
#line 102
      break;
      default: return 0;
    }
}

# 108 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOSMR(uint8_t chnl_id, uint32_t val)
{
  *(chnl_id < 4 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00000 + (uint32_t )(chnl_id << 2)) : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00080 + (uint32_t )((chnl_id - 4) << 2))) = val;
  return;
}

#line 97
static uint32_t HplPXA27xOSTimerM$PXA27xOST$getOSCR(uint8_t chnl_id)
{
  uint8_t remap_id;
  uint32_t val;

  remap_id = chnl_id < 4 ? 0 : chnl_id;
  val = *(remap_id == 0 ? & * (volatile uint32_t *)0x40A00010 : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00040 + (uint32_t )((remap_id - 4) << 2)));

  return val;
}

#line 233
static void HplPXA27xOSTimerM$PXA27xOST$default$fired(uint8_t chnl_id)
{
  HplPXA27xOSTimerM$PXA27xOST$setOIERbit(chnl_id, FALSE);
  HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(chnl_id);
  return;
}

# 139 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimer.nc"
static void HplPXA27xOSTimerM$PXA27xOST$fired(uint8_t arg_0x4044d138){
#line 139
  switch (arg_0x4044d138) {
#line 139
    case 3:
#line 139
      PlatformP$OST0M3$fired();
#line 139
      break;
#line 139
    case 4:
#line 139
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$fired();
#line 139
      break;
#line 139
    case 5:
#line 139
      /*CounterMilliC.PhysCounterMilli32*/HalPXA27xCounterM$0$OSTChnl$fired();
#line 139
      break;
#line 139
    default:
#line 139
      HplPXA27xOSTimerM$PXA27xOST$default$fired(arg_0x4044d138);
#line 139
      break;
#line 139
    }
#line 139
}
#line 139
# 163 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOIERbit(uint8_t chnl_id, bool flag)
{
  if (flag == TRUE) {
      * (volatile uint32_t *)0x40A0001C |= 1 << chnl_id;
    }
  else {
      * (volatile uint32_t *)0x40A0001C &= ~(1 << chnl_id);
    }
  return;
}

#line 149
static bool HplPXA27xOSTimerM$PXA27xOST$clearOSSRbit(uint8_t chnl_id)
{
  bool bFlag = FALSE;

  if ((* (volatile uint32_t *)0x40A00014 & (1 << chnl_id)) != 0) {
      bFlag = TRUE;
    }


  * (volatile uint32_t *)0x40A00014 = 1 << chnl_id;

  return bFlag;
}

# 85 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
__attribute((interrupt("FIQ")))   void hplarmv_fiq(void )
#line 85
{
}

# 52 "/opt/tinyos-2.1.0/tos/system/RealMainP.nc"
  int main(void )
#line 52
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
    {





      {
      }
#line 60
      ;

      RealMainP$Scheduler$init();





      RealMainP$PlatformInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;





      RealMainP$SoftwareInit$init();
      while (RealMainP$Scheduler$runNextTask()) ;
    }
#line 77
    __nesc_atomic_end(__nesc_atomic); }


  __nesc_enable_interrupt();

  RealMainP$Boot$booted();


  RealMainP$Scheduler$taskLoop();




  return -1;
}

# 61 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static error_t HplPXA27xOSTimerM$Init$init(void )
{
  bool initflag;

  /* atomic removed: atomic calls only */
#line 64
  {
    initflag = HplPXA27xOSTimerM$gfInitialized;
    HplPXA27xOSTimerM$gfInitialized = TRUE;
  }

  if (!initflag) {
      * (volatile uint32_t *)0x40A0001C = 0x0UL;
      * (volatile uint32_t *)0x40A00014 = 0xFFFFFFFF;
      HplPXA27xOSTimerM$OST0Irq$allocate();
      HplPXA27xOSTimerM$OST1Irq$allocate();
      HplPXA27xOSTimerM$OST2Irq$allocate();
      HplPXA27xOSTimerM$OST3Irq$allocate();
      HplPXA27xOSTimerM$OST4_11Irq$allocate();
      HplPXA27xOSTimerM$OST0Irq$enable();
      HplPXA27xOSTimerM$OST1Irq$enable();
      HplPXA27xOSTimerM$OST2Irq$enable();
      HplPXA27xOSTimerM$OST3Irq$enable();
      HplPXA27xOSTimerM$OST4_11Irq$enable();
    }

  return SUCCESS;
}

# 94 "/opt/tinyos-2.1.0/tos/chips/pxa27x/HplPXA27xInterruptM.nc"
static error_t HplPXA27xInterruptM$allocate(uint8_t id, bool level, uint8_t priority)
{
  uint32_t tmp;
  error_t error = FAIL;

  /* atomic removed: atomic calls only */
#line 99
  {
    uint8_t i;

#line 101
    if (HplPXA27xInterruptM$usedPriorities == 0) {
        uint8_t PriorityTable[40];
#line 102
        uint8_t DuplicateTable[40];

#line 103
        for (i = 0; i < 40; i++) {
            DuplicateTable[i] = PriorityTable[i] = 0xFF;
          }

        for (i = 0; i < 40; i++) 
          if (TOSH_IRP_TABLE[i] != 0xff) {
              if (PriorityTable[TOSH_IRP_TABLE[i]] != 0xFF) {


                DuplicateTable[i] = PriorityTable[TOSH_IRP_TABLE[i]];
                }
              else {
#line 114
                PriorityTable[TOSH_IRP_TABLE[i]] = i;
                }
            }

        for (i = 0; i < 40; i++) {
            if (PriorityTable[i] != 0xff) {
                PriorityTable[HplPXA27xInterruptM$usedPriorities] = PriorityTable[i];
                if (i != HplPXA27xInterruptM$usedPriorities) {
                  PriorityTable[i] = 0xFF;
                  }
#line 123
                HplPXA27xInterruptM$usedPriorities++;
              }
          }

        for (i = 0; i < 40; i++) 
          if (DuplicateTable[i] != 0xFF) {
              uint8_t j;
#line 129
              uint8_t ExtraTable[40];

#line 130
              for (j = 0; DuplicateTable[i] != PriorityTable[j]; j++) ;
              memcpy(ExtraTable + j + 1, PriorityTable + j, HplPXA27xInterruptM$usedPriorities - j);
              memcpy(PriorityTable + j + 1, ExtraTable + j + 1, 
              HplPXA27xInterruptM$usedPriorities - j);
              PriorityTable[j] = i;
              HplPXA27xInterruptM$usedPriorities++;
            }

        for (i = 0; i < HplPXA27xInterruptM$usedPriorities; i++) {
            * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((i & 0x1F) << 2)) = (1 << 31) | PriorityTable[i];
            tmp = * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((i & 0x1F) << 2));
          }
      }

    if (id < 34) {
        if (priority == 0xff) {
            priority = HplPXA27xInterruptM$usedPriorities;
            HplPXA27xInterruptM$usedPriorities++;
            * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((priority & 0x1F) << 2)) = (1 << 31) | id;
            tmp = * (volatile uint32_t *)((uint32_t )0x40D0001C + (uint32_t )((priority & 0x1F) << 2));
          }
        if (level) {
            *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A4 : & * (volatile uint32_t *)0x40D00008) |= 1 << (id & 0x1f);
            tmp = *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A4 : & * (volatile uint32_t *)0x40D00008);
          }

        error = SUCCESS;
      }
  }
  return error;
}

static void HplPXA27xInterruptM$enable(uint8_t id)
{
  uint32_t tmp;

  /* atomic removed: atomic calls only */
#line 165
  {
    if (id < 34) {
        *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A0 : & * (volatile uint32_t *)0x40D00004) |= 1 << (id & 0x1f);
        tmp = *(id & 0x20 ? & * (volatile uint32_t *)0x40D000A0 : & * (volatile uint32_t *)0x40D00004);
      }
  }
  return;
}

# 121 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HplPXA27xOSTimerM.nc"
static void HplPXA27xOSTimerM$PXA27xOST$setOMCR(uint8_t chnl_id, uint32_t val)
{
  if (chnl_id > 3) {
      * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A000C0 + (uint32_t )((chnl_id - 4) << 2)) = val;
    }
  return;
}

#line 87
static void HplPXA27xOSTimerM$PXA27xOST$setOSCR(uint8_t chnl_id, uint32_t val)
{
  uint8_t remap_id;

  remap_id = chnl_id < 4 ? 0 : chnl_id;
  *(remap_id == 0 ? & * (volatile uint32_t *)0x40A00010 : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40A00040 + (uint32_t )((remap_id - 4) << 2))) = val;

  return;
}

# 158 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGAFRpin(uint8_t pin, uint8_t func)
{
  func &= 0x3;
  * (volatile uint32_t *)((uint32_t )0x40E00054 + (uint32_t )((pin & 0x70) >> 2)) = (* (volatile uint32_t *)((uint32_t )0x40E00054 + (uint32_t )((pin & 0x70) >> 2)) & ~(3 << ((pin & 0x0f) << 1))) | (func << ((pin & 0x0f) << 1));
  return;
}

#line 85
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGPDRbit(uint8_t pin, bool dir)
{
  if (dir) {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0000C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0010C) |= 1 << (pin & 0x1f);
    }
  else {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0000C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0010C) &= ~(1 << (pin & 0x1f));
    }
  return;
}

# 140 "/opt/tinyos-2.1.0/tos/chips/pxa27x/i2c/HplPXA27xI2CP.nc"
static void /*HplPXA27xI2CC.HplPXA27xI2CP*/HplPXA27xI2CP$1$I2C$setISAR(uint32_t val)
#line 140
{
  switch (0) {
      case 0: * (volatile uint32_t *)0x403016A0 = val;
#line 142
      break;
      case 1: * (volatile uint32_t *)0x40F001A0 = val;
#line 143
      break;
      default: break;
    }
  return;
}

# 129 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGFERbit(uint8_t pin, bool flag)
{
  if (flag) {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0003C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0013C) |= 1 << (pin & 0x1f);
    }
  else {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E0003C + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E0013C) &= ~(1 << (pin & 0x1f));
    }
  return;
}

# 151 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static error_t PMICM$writePMIC(uint8_t address, uint8_t value)
#line 151
{
  PMICM$PI2C$setIDBR(0x49 << 1);
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 0));
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
  while (PMICM$PI2C$getICR() & (1 << 3)) ;

  * (volatile uint32_t *)0x40F00188 = address;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 0));
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
  while (PMICM$PI2C$getICR() & (1 << 3)) ;

  * (volatile uint32_t *)0x40F00188 = value;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 1));
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() | (1 << 3));
  while (PMICM$PI2C$getICR() & (1 << 3)) ;
  PMICM$PI2C$setICR(PMICM$PI2C$getICR() & ~(1 << 1));
#line 166
  * (volatile uint32_t *)0x40F00190 &= ~(1 << 1);

  return SUCCESS;
}

# 56 "/opt/tinyos-2.1.0/tos/chips/pxa27x/dma/HplPXA27xDMAM.nc"
static void HplPXA27xDMAM$HplPXA27xDMACntl$setDRCMR(uint8_t peripheral, uint8_t val)
#line 56
{
  *(peripheral < 63 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40000100 + (uint32_t )((peripheral & 0x3f) << 2)) : & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40001100 + (uint32_t )((peripheral & 0x3f) << 2))) = val;
}

#line 79
static void HplPXA27xDMAM$HplPXA27xDMAChnl$setDALGNbit(uint8_t chnl, bool flag)
#line 79
{
  if (flag) {
      * (volatile uint32_t *)0x400000A0 |= 1 << chnl;
    }
  else {
      * (volatile uint32_t *)0x400000A0 &= ~(1 << chnl);
    }
  return;
}

# 54 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static void HalPXA27xGeneralIOM$GeneralIO$set(uint8_t pin)
#line 54
{

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 56
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPSRbit(pin);
#line 56
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 123 "/opt/tinyos-2.1.0/tos/system/SchedulerBasicP.nc"
static bool SchedulerBasicP$Scheduler$runNextTask(void )
{
  uint8_t nextTask;

  /* atomic removed: atomic calls only */
#line 127
  {
    nextTask = SchedulerBasicP$popTask();
    if (nextTask == SchedulerBasicP$NO_TASK) 
      {
        {
          unsigned char __nesc_temp = 
#line 131
          FALSE;

#line 131
          return __nesc_temp;
        }
      }
  }
#line 134
  SchedulerBasicP$TaskBasic$runTask(nextTask);
  return TRUE;
}

#line 164
static void SchedulerBasicP$TaskBasic$default$runTask(uint8_t id)
{
}

# 64 "/opt/tinyos-2.1.0/tos/interfaces/TaskBasic.nc"
static void SchedulerBasicP$TaskBasic$runTask(uint8_t arg_0x40333758){
#line 64
  switch (arg_0x40333758) {
#line 64
    case /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer:
#line 64
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired:
#line 64
      /*HilTimerMilliC.AlarmToTimerMilli32*/AlarmToTimerC$0$fired$runTask();
#line 64
      break;
#line 64
    case /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm:
#line 64
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$runTask();
#line 64
      break;
#line 64
    case VMC$execute_instructionTask:
#line 64
      VMC$execute_instructionTask$runTask();
#line 64
      break;
#line 64
    case /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task:
#line 64
      /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$signalDone_task$runTask();
#line 64
      break;
#line 64
    case HalTsl2561ControlP$complete_Alert:
#line 64
      HalTsl2561ControlP$complete_Alert$runTask();
#line 64
      break;
#line 64
    case HalTsl2561ControlP$complete_Task:
#line 64
      HalTsl2561ControlP$complete_Task$runTask();
#line 64
      break;
#line 64
    case /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask:
#line 64
      /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$runTask();
#line 64
      break;
#line 64
    case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone:
#line 64
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StartDone$runTask();
#line 64
      break;
#line 64
    case /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone:
#line 64
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$StopDone$runTask();
#line 64
      break;
#line 64
    case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError:
#line 64
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleReadError$runTask();
#line 64
      break;
#line 64
    case /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError:
#line 64
      /*Tsl2561InternalC.I2CC.HalPXA27xI2CMasterP*/HalPXA27xI2CMasterP$0$handleWriteError$runTask();
#line 64
      break;
#line 64
    case SerialP$RunTx:
#line 64
      SerialP$RunTx$runTask();
#line 64
      break;
#line 64
    case SerialP$startDoneTask:
#line 64
      SerialP$startDoneTask$runTask();
#line 64
      break;
#line 64
    case SerialP$stopDoneTask:
#line 64
      SerialP$stopDoneTask$runTask();
#line 64
      break;
#line 64
    case SerialP$defaultSerialFlushTask:
#line 64
      SerialP$defaultSerialFlushTask$runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$signalSendDone$runTask();
#line 64
      break;
#line 64
    case /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask:
#line 64
      /*SerialDispatcherC.SerialDispatcherP*/SerialDispatcherP$0$receiveTask$runTask();
#line 64
      break;
#line 64
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask:
#line 64
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$CancelTask$runTask();
#line 64
      break;
#line 64
    case /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask:
#line 64
      /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$errorTask$runTask();
#line 64
      break;
#line 64
    default:
#line 64
      SchedulerBasicP$TaskBasic$default$runTask(arg_0x40333758);
#line 64
      break;
#line 64
    }
#line 64
}
#line 64
# 155 "/opt/tinyos-2.1.0/tos/system/AMQueueImplP.nc"
static void /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$sendDone(uint8_t last, message_t * msg, error_t err)
#line 155
{
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$queue[last].msg = (void *)0;
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$tryToSend();
  /*SerialAMQueueP.AMQueueImplP*/AMQueueImplP$0$Send$sendDone(last, msg, err);
}

# 132 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static am_addr_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMPacket$destination(message_t *amsg)
#line 132
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(amsg);

#line 134
  return __nesc_ntoh_uint16(header->dest.data);
}

#line 57
static error_t /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$AMSend$send(am_id_t id, am_addr_t dest, 
message_t *msg, 
uint8_t len)
#line 59
{
  serial_header_t *header = /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$getHeader(msg);

#line 61
  __nesc_hton_uint16(header->dest.data, dest);





  __nesc_hton_uint8(header->type.data, id);
  __nesc_hton_uint8(header->length.data, len);

  return /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$SubSend$send(msg, len);
}

# 133 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$startTimer(uint8_t num, uint32_t t0, uint32_t dt, bool isoneshot)
{
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num];

#line 136
  timer->t0 = t0;
  timer->dt = dt;
  timer->isoneshot = isoneshot;
  timer->isrunning = TRUE;
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 55 "VMC.nc"
static void VMC$next_app(void )
#line 55
{

  uint8_t i;

  for (i = 0; i < 3; i++) {
      VMC$active_app = (VMC$active_app + 1) % 3;
      if (VMC$apps[VMC$active_app].isActive && VMC$apps[VMC$active_app].handler != 0) {
          VMC$execute_instructionTask$postTask();
          return;
        }
    }


  VMC$active_app = 3;
}

# 78 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HalTsl2561ReaderP.nc"
static error_t /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandPhoto$read(void )
#line 78
{
  error_t status;

#line 80
  if (/*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$m_state != /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$S_READY) {
    return FAIL;
    }
#line 82
  status = /*VMCApp.Sensor.Sensor.HalTsl2561ReaderP*/HalTsl2561ReaderP$0$BroadbandResource$request();
  return status;
}

# 83 "/opt/tinyos-2.1.0/tos/chips/tsl2561/HplTSL2561LogicP.nc"
static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doWriteReg(uint8_t nextState, uint8_t reg, uint16_t val, uint8_t size)
#line 83
{
  error_t error = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 86
    {
      if (/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState == /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE || /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState == /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_STARTING) {
          /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = nextState;
        }
      else {
          error = EBUSY;
        }
    }
#line 93
    __nesc_atomic_end(__nesc_atomic); }
  if (error) {
    return error;
    }
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer[0] = (1 << 7) | reg;
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer[1] = (uint8_t )(val & 0xFF);
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer[2] = (uint8_t )((val >> 8) & 0xFF);

  error = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$write(I2C_START | I2C_STOP, 73U, size + 1, /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer);

  if (error) {
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 104
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
#line 104
      __nesc_atomic_end(__nesc_atomic); }
    }
  return error;
}

static error_t /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$doReadPrep(uint8_t nextState, uint8_t reg)
#line 109
{
  error_t error = SUCCESS;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 112
    {
      if (/*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState == /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE) {
          /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = nextState;
        }
      else {
          error = EBUSY;
        }
    }
#line 119
    __nesc_atomic_end(__nesc_atomic); }
  if (error) {
    return error;
    }
  /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer[0] = (1 << 7) | reg;

  error = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$I2CPacket$write(I2C_START, 73U, 1, /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mI2CBuffer);

  if (error) {
    { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 128
      /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$mState = /*Tsl2561InternalC.Logic*/HplTSL2561LogicP$0$STATE_IDLE;
#line 128
      __nesc_atomic_end(__nesc_atomic); }
    }
  return error;
}

# 97 "/opt/tinyos-2.1.0/tos/system/SimpleArbiterP.nc"
static error_t /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Resource$release(uint8_t id)
#line 97
{
  bool released = FALSE;

#line 99
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 99
    {
      if (/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$state == /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$RES_BUSY && /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$resId == id) {
          if (/*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$isEmpty() == FALSE) {
              /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$reqResId = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$Queue$dequeue();
              /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$RES_GRANTING;
              /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$grantedTask$postTask();
            }
          else {
              /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$resId = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$NO_RES;
              /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$state = /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$RES_IDLE;
            }
          released = TRUE;
        }
    }
#line 112
    __nesc_atomic_end(__nesc_atomic); }
  if (released == TRUE) {
      /*Tsl2561InternalC.Arbiter.Arbiter*/SimpleArbiterP$0$ResourceConfigure$unconfigure(id);
      return SUCCESS;
    }
  return FAIL;
}

# 551 "VMC.nc"
static void VMC$Read$readDone(error_t result, uint16_t val)
#line 551
{

  uint8_t i;


  if (val > 255) {
    VMC$cache_value = 255;
    }
  else {
#line 559
    VMC$cache_value = (uint8_t )val;
    }
#line 560
  VMC$cache_time = VMC$Timer$getNow(0);


  for (i = 0; i < 3; i++) {
      if (VMC$apps[i].isActive && VMC$apps[i].waiting_sensor != 6) {
          int8_t r = VMC$apps[i].waiting_sensor;

          if (val > 255) {
            VMC$apps[i].regs[r] = 255;
            }
          else {
#line 570
            VMC$apps[i].regs[r] = (uint8_t )val;
            }
#line 571
          VMC$apps[i].waiting_sensor = 6;

          ;
        }
    }
}

# 60 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HalPXA27xGeneralIOM.nc"
static void HalPXA27xGeneralIOM$GeneralIO$clr(uint8_t pin)
#line 60
{
  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 61
    HalPXA27xGeneralIOM$HplPXA27xGPIOPin$setGPCRbit(pin);
#line 61
    __nesc_atomic_end(__nesc_atomic); }
  return;
}

# 132 "/opt/tinyos-2.1.0/tos/chips/pxa27x/timer/HalPXA27xAlarmM.nc"
static void /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$Alarm$startAt(uint32_t t0, uint32_t dt)
#line 132
{
  uint32_t tf;
#line 133
  uint32_t t1;
  bool bPending;

#line 135
  tf = t0 + dt;

  { __nesc_atomic_t __nesc_atomic = __nesc_atomic_start();
#line 137
    {
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(TRUE);
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOSMR(tf);
      /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$mfRunning = TRUE;
      t1 = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSCR();
      bPending = /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$getOSSRbit();
      if (dt <= t1 - t0 && !bPending) {
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$OSTChnl$setOIERbit(FALSE);
          /*HilTimerMilliC.PhysAlarmMilli32*/HalPXA27xAlarmM$0$lateAlarm$postTask();
        }
    }
#line 147
    __nesc_atomic_end(__nesc_atomic); }

  return;
}

# 62 "/opt/tinyos-2.1.0/tos/lib/timer/VirtualizeTimerC.nc"
static void /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$fireTimers(uint32_t now)
{
  uint8_t num;

  for (num = 0; num < /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$NUM_TIMERS; num++) 
    {
      /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer_t *timer = &/*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$m_timers[num];

      if (timer->isrunning) 
        {
          uint32_t elapsed = now - timer->t0;

          if (elapsed >= timer->dt) 
            {
              if (timer->isoneshot) {
                timer->isrunning = FALSE;
                }
              else {
#line 79
                timer->t0 += timer->dt;
                }
              /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$Timer$fired(num);
              break;
            }
        }
    }
  /*HilTimerMilliC.VirtTimersMilli32*/VirtualizeTimerC$0$updateFromTimer$postTask();
}

# 519 "VMC.nc"
static void VMC$Timer$fired(uint8_t id)
#line 519
{

  serial_response *returnmes = (serial_response *)VMC$SerialPacket$getPayload(&VMC$serial_p, sizeof(serial_response ));

#line 522
  __nesc_hton_uint8(returnmes->instr.data, 1);
  __nesc_hton_uint8(returnmes->pc.data, 1);
  __nesc_hton_uint8(returnmes->appid.data, 1);
  if (VMC$SerialAMSend$send(AM_BROADCAST_ADDR, &VMC$serial_p, sizeof(serial_response )) == SUCCESS) {
    VMC$serial_busy = TRUE;
    }
  if (VMC$apps[id].isActive == 0 || VMC$apps[id].PC != 0) {
    return;
    }
  ;



  if (VMC$apps[id].handler == 0) {
    VMC$apps[id].handler = 2;
    }
  else {
#line 538
    VMC$apps[id].next_handler = 2;
    }


  if (VMC$active_app == 3) {

      VMC$active_app = id;
      VMC$execute_instructionTask$postTask();
    }
}

# 119 "/opt/tinyos-2.1.0/tos/lib/serial/SerialActiveMessageP.nc"
static void */*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$getPayload(message_t *msg, uint8_t len)
#line 119
{
  if (len > /*SerialActiveMessageC.AM*/SerialActiveMessageP$0$Packet$maxPayloadLength()) {
      return (void *)0;
    }
  else {
      return (void * )msg->data;
    }
}

# 334 "/opt/tinyos-2.1.0/tos/platforms/intelmote2/chips/da9030/PMICM.nc"
static error_t PMICM$getPMICADCVal(uint8_t channel, uint8_t *val)
#line 334
{
  uint8_t oldval;
  error_t rval;


  rval = PMICM$readPMIC(0x30, &oldval, 1);
  if (rval == SUCCESS) {
      rval = PMICM$writePMIC(0x30, ((channel & 0x7) | (
      1 << 3)) | (1 << 4));
    }
  if (rval == SUCCESS) {
      rval = PMICM$readPMIC(0x40, val, 1);
    }
  if (rval == SUCCESS) {

      rval = PMICM$writePMIC(0x30, oldval);
    }

  return rval;
}

# 113 "/opt/tinyos-2.1.0/tos/chips/pxa27x/gpio/HplPXA27xGPIOM.nc"
static void HplPXA27xGPIOM$HplPXA27xGPIOPin$setGRERbit(uint8_t pin, bool flag)
{
  if (flag) {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00030 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00130) |= 1 << (pin & 0x1f);
    }
  else {
      *(pin < 96 ? & * (volatile uint32_t *)((uint32_t )& * (volatile uint32_t *)0x40E00030 + (uint32_t )((pin & 0x60) >> 3)) : & * (volatile uint32_t *)0x40E00130) &= ~(1 << (pin & 0x1f));
    }
  return;
}

