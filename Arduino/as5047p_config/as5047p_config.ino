#define MOSI 51
#define MISO 50
#define SCK  52
#define ENCODER_SS 53


void setup() {
  Serial.begin(115200);
  Serial.print("/************** Init ****************/\n");
  init_spi();
  delay(100);
  //-- bit 7 enable PWM
  //-- bit 5 binary mode (moves to from 4000 to 4096 cpr)
  write_reg(0x18,(1<<7) | (1<<5));
  uint16_t res = read_reg(0x18);
  Serial.println(res,HEX);
  if(res == (1<<7) | (1<<5)){
    Serial.print("Config set correctly, starting flash procedure\n");
    //-- set the prog enable bit (0)  in the prog reg (0x03):
    write_reg(0x03,(1<<0));
    //-- set the prog start bit (3)  in the prog reg (0x03):
    write_reg(0x03,(1<<3));
    //read the prog reg until it reads 1:
    while(((res = read_reg(0x03))&0x3FF)!= 1){
      Serial.println(res&0x3FF);
    }
    
  }
}

void loop() {
  //--Period -> 1828us, 
  /*uint32_t pos = read_reg(0x3FFF)&0x3FFF; 
  //float pos_estimate = (deltaT - 7.10075261) / 1817.348871;
  //deltaT = pos_estimate * 16384.0;
  //int16_t error =   pos - deltaT;
  
  Serial.print(pos); Serial.print("\t");    
  //Serial.print(deltaT>>2); Serial.print("\t");  
  //Serial.print(error>>2);
  Serial.print("\n");
  delay(100);*/
}

uint16_t read_reg(uint16_t addr){
  //-- Set the read bit
  uint16_t res, cmd = 0x4000;
  
  cmd |= addr & 0x3FFF;
  cmd |= parity(cmd) << 15;

  digitalWrite(ENCODER_SS, LOW);

  SPDR = cmd >> 8;  
  while (!(SPSR & (1 << SPIF)));  
  
  res = SPDR << 8;
  SPDR = cmd & 0xFF;  
  while (!(SPSR & (1 << SPIF)));
  res |= SPDR;
  
  digitalWrite(ENCODER_SS, HIGH);
  
  if(addr != 0)
    res = read_reg(0);
  
  return res;
}
uint16_t write_reg(uint16_t addr, uint16_t val){
  
  uint16_t res, cmd = 0x0000;
  
  cmd |= addr & 0x3FFF;
  cmd |= parity(cmd) << 15;

  //Serial.print(cmd, HEX); Serial.print("\t");  
  //Serial.print(val, HEX); Serial.print("\t");  
  
  digitalWrite(ENCODER_SS, LOW);
  SPDR = cmd >> 8;  
  while (!(SPSR & (1 << SPIF)));  
  res = SPDR << 8;
  SPDR = cmd & 0xFF;  
  while (!(SPSR & (1 << SPIF)));
  res |= SPDR;    
  digitalWrite(ENCODER_SS, HIGH);
  delay(1);
  val &= 0x3FFF;
  val |= parity(val) << 15;
  digitalWrite(ENCODER_SS, LOW);
  SPDR = val >> 8;  
  while (!(SPSR & (1 << SPIF)));  
  res = SPDR << 8;
  SPDR = val & 0xFF;  
  while (!(SPSR & (1 << SPIF)));
  res |= SPDR;    
  digitalWrite(ENCODER_SS, HIGH);
  
  return res;
}


/*******************Encoder Input Code     *******************/
void init_spi() {
  // Set MOSI and SCK output
  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);
  pinMode(ENCODER_SS, OUTPUT);
  digitalWrite(ENCODER_SS, HIGH);

  // Enable SPI, Master, set clock rate fck/2, mode 1
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPI2X) | (1 << CPHA);
}

uint8_t parity(uint16_t v)
{
    v ^= v >> 8;
    v ^= v >> 4;
    v ^= v >> 2;
    v ^= v >> 1;
    return v & 1;
}
