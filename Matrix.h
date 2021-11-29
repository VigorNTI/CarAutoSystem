//data display from right to left, from bottom to top, HIGH level display. 
#define IIC_SCL  A5
#define IIC_SDA  A4

class Matrix {
public:
  int mode;
  int cycleCounter;
  int cycleDelay;
  int cycleSelect;
  Matrix(){ mode = 0; cycleCounter = 0; cycleSelect = 2; cycleDelay = 50; }
  // function to call
  void set_mode(int mode_in, int cycleDelay_in){
    this->mode = mode_in;
    this->cycleDelay = cycleDelay_in;
  }
  void set_mode(int mode_in){
    this->mode = mode_in;
  }
  void update() {
    if (this->mode == 0 || this->mode == 1) {
      this->write(mode);
    }else{
      this->cycleCounter++;
      if (cycleCounter >= cycleDelay) {
        if (cycleSelect == 2)
          cycleSelect = 3;
        else
          cycleSelect = 2;
        cycleCounter = 0;
      }
      this->write(cycleSelect);
    }
  }
  void setup() 
  {
    pinMode(IIC_SCL, OUTPUT);
    pinMode(IIC_SDA, OUTPUT);
    digitalWrite(IIC_SCL, LOW);
    digitalWrite(IIC_SDA, LOW);
  }
  unsigned char data_display1 = 0;
  unsigned char data_display2 = 0;
  unsigned char data_display3 = 0;
  unsigned char data_display4 = 0;
  unsigned char data_display5 = 0;
  unsigned char data_display6 = 0;
  unsigned char data_display7 = 0;
  unsigned char data_display8 = 0;
  unsigned char data_display9 = 0;
  unsigned char data_display10 = 0;
  unsigned char data_display11 = 0;
  unsigned char data_display12 = 0;
  unsigned char data_display13 = 0;
  unsigned char data_display14 = 0;
  unsigned char data_display15 = 0;
  unsigned char data_display16 = 0;
  //unsigned char table[] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01}; 
  unsigned char table[4][16] = {
    {0x00,0x00,0x0E,0x10,0x0E,0x60,0x80,0x40,0x40,0x80,0x60,0x0E,0x10,0x0E,0x00,0x00},
    {0x00,0x00,0x0A,0x04,0x0A,0x60,0x80,0x40,0x40,0x80,0x60,0x0A,0x04,0x0A,0x00,0x00},
    {0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x18,0x18,0x10,0x10,0x10,0x10,0x10,0x10,0x00},
    {0x00,0x10,0x10,0x10,0x08,0x04,0x02,0x1A,0x1A,0x02,0x04,0x08,0x10,0x10,0x10,0x00}}; 

private:
  /*----------------------------------------------------------------*/
  bool write(int buff_val) 
  {
      /**************set the address plus 1***************/
      IIC_start();
      IIC_send(0x40);// set the address plus 1 automatically
      IIC_end();
      /************end the process of address plus 1 *****************/
      /************set the data display*****************/ 
      IIC_start();
      IIC_send(0xc0);// set the initial address as 0
      for(char i = 0;i < 16;i++)
      {
         IIC_send(table[buff_val][i]);// send the display data 
      }
  
      IIC_end();
      /************end the data display*****************/
      /*************set the brightness display***************/ 
      IIC_start();
      IIC_send(0x8A);// set the brightness display
      IIC_end(); 
      /*************end the brightness display***************/ 
      return true;
  }
  /*----------------------------------------------------------------*/
  void IIC_start()
  {
    digitalWrite(IIC_SCL,LOW);
    delayMicroseconds(3);
    digitalWrite(IIC_SDA,HIGH);
    delayMicroseconds(3);
    digitalWrite(IIC_SCL,HIGH);
    delayMicroseconds(3);
    digitalWrite(IIC_SDA,LOW);
    delayMicroseconds(3);
  }
  void IIC_send(unsigned char send_data)
  {
    for(char i = 0;i < 8;i++)
    {
        digitalWrite(IIC_SCL,LOW);
        delayMicroseconds(3); 
        if(send_data & 0x01)
        {
          digitalWrite(IIC_SDA,HIGH);
        }
        else
        {
          digitalWrite(IIC_SDA,LOW);
        }
        delayMicroseconds(3);
        digitalWrite(IIC_SCL,HIGH); 
        delayMicroseconds(3);
        send_data = send_data >> 1;
    }
  }
  void IIC_end()
  {
    digitalWrite(IIC_SCL,LOW);
    delayMicroseconds(3);
    digitalWrite(IIC_SDA,LOW);
    delayMicroseconds(3);
    digitalWrite(IIC_SCL,HIGH);
    delayMicroseconds(3);
    digitalWrite(IIC_SDA,HIGH);
    delayMicroseconds(3);
  }
};
