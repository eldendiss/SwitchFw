// comm/i2c_proto.cpp
#include <Wire.h>
#include "i2c_proto.h"

static AppConfig* g_cfg=nullptr;
static ControlState* g_state=nullptr;
static uint8_t reg_ptr=0;

void i2c_bind(AppConfig* cfg, ControlState* state){ g_cfg=cfg; g_state=state; }

static uint8_t* cfg_bytes(){ return (uint8_t*)&g_cfg->ctrl; }

static void on_receive(int n){
  if (n<=0) return;
  reg_ptr = Wire.read(); n--;
  // writes: small subset for phase 1
  while(n>0){
    uint8_t b=Wire.read(); n--;
    // Write into ctrl struct up to its size
    uint8_t* base = cfg_bytes();
    uint16_t sz = sizeof(ControlParams);
    if (reg_ptr < sz){ base[reg_ptr]=b; }
    reg_ptr++;
  }
}
static void on_request(){
  // Return status or cfg bytes depending on ptr
  uint8_t* base = cfg_bytes();
  uint16_t sz = sizeof(ControlParams);

  if (reg_ptr==0x40){ // command ack region
    uint8_t ack=0; Wire.write(&ack,1); return;
  }
  else if (reg_ptr==0x50 && g_state){
    // status block
    uint8_t buf[6];
    buf[0]=0; // run_state filled by main
    buf[1]=0; // fault_code filled by main
    uint16_t fb = (uint16_t)g_state->fb_ema;
    int16_t dcounts = 0; // main can cache last derivative if needed
    buf[2]=(uint8_t)(fb&0xFF); buf[3]=(uint8_t)(fb>>8);
    buf[4]=(uint8_t)(dcounts&0xFF); buf[5]=(uint8_t)(dcounts>>8);
    Wire.write(buf,6); return;
  }
  else {
    if (reg_ptr<sz){
      uint8_t chunk = (uint8_t)min<int>(sz - reg_ptr, 16);
      Wire.write(base + reg_ptr, chunk);
      reg_ptr += chunk;
      return;
    }
  }
  uint8_t zero=0; Wire.write(&zero,1);
}

void i2c_init(uint8_t addr){
  Wire.begin(addr); // slave
  Wire.onReceive(on_receive);
  Wire.onRequest(on_request);
}
