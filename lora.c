/* Tyler Eaves
   teaves@eng.ucsd.edu
   UCSD CubeSat
   lora.c
 
   Last Update: 08/27/18 
                5:45 PM
   
   This file is designed to establish SPI control over
   the SX1278 LoRa transceiver module via a Raspberry Pi. It can
   be compiled with something like:

   $ cc lora.c -o lora -lbcm2835

   The bcm2835  library must be flagged for compilation to succeed.
   The RPI uses its many functions to facilitate the SPI. It can be
   downloaded at:

   www.airspayce.com/mikem/bcm2835/

   This file uses version 1.56.  

   Concerning theory of SPI interaction with the device: Don't rely
   completely on the SX1278 datasheet for a guide to accessing registers.
   A fundamental property of the bcm2835 library is that its byte transfer
   functions write and read MOSI/MISO simultaneously.  This poses a problem
   in that the SX1278 cannot respond to an address byte with the 
   corresponding data value until that address byte is completely clocked in
   (otherwise it wouldn't yet know what the address was).  

   I've discovered experimentally that a read operation actually requires two 
   bytes MOSI, the first being the address and the second being garbage the 
   SX1278 will ignore because the first byte flagged a read operation.  This 
   is necessary because the bcm2835 library's multi-byte transfer function 
   requires transmit and receive buffers of equal size. While the address byte 
   is being transmitted from the first element of the transmit buffer the 
   first element of the receive buffer will clock garbage (0x00).  Then as 
   garbage is clocked out of the second element of the transmit buffer the 
   valuable data from the desired address will be clocked into the second 
   element of the receive buffer. Bytes are transmitted and received starting 
   at a buffer's first element and incrementing. 

   As an example take these two inital arrays and transfer command:

   char tbuf[] = {0x07, 0x00};
   char rbuf[] = {0x00, 0x00}; 
   bcm2835_spi_transfernb(tbuf, rbuf, sizeof(tbuf));

   After the transfer the arrays will exist as follows:

   tbuf --> {0x07, 0x00}
   rbuf --> {0x00, 0x80}

   In this example the byte 0x07 refers to the address of the register
   holding the MSB's of the RF carrier frequency.  The default/reset value
   is 0x80, which is what we've read.  Again, note that the data byte was
   clocked into the second element of the receive buffer, only after the 
   address byte had been completely clocked out of the first element of the
   transmit buffer.  

   The following example details a successful write operation.  In the same 
   manner as before, two buffer arrays were setup and the ..._transfernb()
   command used. A write requires only two bytes both ways, just like the 
   read.  The intitial interaction is:

   Sent --> {0x81, 0x08}
   Reci <-- {0x00, 0x0F}

   The first byte sent, 0x81, is the address.  This actually refers to 
   SX1278 register address 0x01, but the write operation flag is a high
   MSB followed by seven bits of address.  Were we reading this register we
   would have sent 0x01.  The second transmitted byte, 0x08, is the data we
   wish to write to the register.  The first received byte, 0x00, is gargbage
   as we expect.  The second received byte, 0x0F, is the last value the 
   register held before we wrote to it. I verified with a prior read
   operation that register 0x01 actually held value 0x0F, so this feedback
   is correct. 

   Now with another read operation we can check the results of the write 
   we just performed.  The read interaction is:

   Sent --> {0x01, 0x00}
   Reci <-- {0x00, 0x08}

   Here the first byte sent, 0x01, is the address we wish to read.  MSB low
   indicates the read operation (and conveniently this is also the name of
   the register in hardware so it's more intuitive than the write case). The
   second byte sent, 0x00, is garbage as we already know.  Similarly the 
   first received byte, 0x00, is also garbage.  Finally we see that the 
   second received byte is 0x08.  The value of register 0x01 has not only 
   changed, but has been updated to exactly the desired value.  We can
   successfully write to a register. At this point we can theoretically
   access all functions of the LoRa module. An appropriate
   test before we begin long term software development is to verify the 
   module can actually transmit and receive data. 

   Notes on Tx:
   I believe I have successfully transmitted a packet.  The SX1278 is designed
   to exhibit two behaviors in the event of a transmission.  First the chip
   will flag a "TxDone" event by flagging bit[3] high in register 0x12.    
   Secondly the device will automatically return to LoRa standby mode which is 
   indicated by value 0x89 in the operating mode register, 0x01.  When I
   write data to the FIFO and enter Tx mode I observe both of these events 
   take place. Without access to RF test equipment there is no way to reliably
   observe the transmitter's output spectrum. The only way to absolutely 
   verify transmission is to setup another LoRa module as a receiver and 
   detect the packet. 

   As a disclaimer, one must be aware of the RPI's clock speed relative to that
   of the SPI connection.  The RPI2 is running 700 MHz default whereas the 
   SPI clock signal will only be reliable up to roughly 15 MHz.  This means
   the RPI can process several instructions before the SPI connection transfers
   even a single bit!  If you wish to OBSERVE the "TxDone" flag and the 
   operating transition to standby mode you must allow a significant delay 
   between the time you initiate Tx mode and attempt to read the diagnostic
   registers.  Do not simply cascade commands as such: 

   .
   .
   .
   write_reg(REG_OP_MODE, LORA_TX);
   read_reg(REG_IRQ_FLAGS);
   read_reg(REG_OP_MODE);
   .
   .
   .     
   
   The registers will not be updated with the expected values by the time they
   are read.  

   ---------------------------------------------------------------------------------------------*/

//------------------------------header files and label definitions------------------------------

#include <bcm2835.h>
#include <stdio.h>

//SX1278 device modes
#define FSK_SLEEP      0b00001000  //0x08  must enter before switching to lora
#define FSK_CAD        0b00001111  //0x0F  seems to be default startup op mode
#define LORA_SLEEP     0b10001000  //0x88
#define LORA_STANDBY   0b10001001  //0x89
#define LORA_TX        0b10001011  //0x8B
#define LORA_RX_CONT   0b10001101  //0x8D
#define LORA_RX_SINGLE 0b10001110  //0x8E
#define LORA_CAD       0b10001111  //0x8F

//register addresses
#define REG_FIFO                 0b00000000  //0x00
#define REG_OP_MODE              0b00000001  //0x01
#define REG_RF_FREQ_MSB_MSB      0b00000110  //0x06
#define REG_RF_FREQ_MSB          0b00000111  //0x07
#define REG_RF_FREQ_LSB          0b00001000  //0x08
#define REG_PA_CONFIG            0b00001001  //0x09
#define REG_PA_RAMP              0b00001010  //0x0A
#define REG_OCP                  0b00001011  //0x0B
#define REG_LNA                  0b00001100  //0x0C
#define REG_FIFO_ADDR_PTR        0b00001101  //0x0D
#define REG_FIFO_TX_BASE_ADDR    0b00001110  //0x0E
#define REG_FIFO_RX_BASE_ADDR    0b00001111  //0x0F
#define REG_FIFO_RX_CURRENT_ADDR 0b00010000  //0x10
#define REG_IRQ_FLAGS_MASK       0b00010001  //0x11
#define REG_IRQ_FLAGS            0b00010010  //0x12
#define REG_RX_NUM_BYTES         0b00010011  //0x13
#define REG_RX_PACKET_COUNT_MSB  0b00010110  //0x16
#define REG_RX_PACKET_COUNT_LSB  0b00010111  //0x17
#define REG_MODEM_STAT           0b00011000  //0x18
#define REG_PACKET_SNR           0b00011001  //0x19
#define REG_PACKET_RSSI          0b00011010  //0x1A
#define REG_CURRENT_RSSI         0b00011011  //0x1B
#define REG_HOP_CHANNEL          0b00011100  //0x1C
#define REG_MODEM_CONFIG1        0b00011101  //0x1D
#define REG_MODEM_CONFIG2        0b00011110  //0x1E
#define REG_PREAMBLE_LEN_MSB     0b00100000  //0x20
#define REG_PREAMBLE_LEN_LSB     0b00100001  //0x21
#define REG_PAYLOAD_LEN          0b00100010  //0x22
#define REG_MAX_PAYLOAD_LEN      0b00100011  //0x23
#define REG_HOP_PERIOD           0b00100100  //0x24
#define REG_MODEM_CONFIG3        0b00100110  //0x26
#define REG_DETECT_OPTIMIZE      0b00110001  //0x31
#define REG_DETECT_THRESH        0b00110111  //0x37
#define REG_SYNC_WORD            0b00111001  //0x39

//helpful values
#define FIFO_TX_BASE_ADDR 0b10000000  //0x80

//-----------------------------------helper function prototypes----------------------------------

uint8_t read_reg(uint8_t addr);

uint8_t write_reg(uint8_t addr, uint8_t data);

void diagnose();

//-----------------------------------------function main-----------------------------------------

int main(int argc, char **argv){

  //test the library initialization functions
  if(!bcm2835_init()){
    printf("bcm2835_init failed.  Must run as root.\n");
    return 1;
  }  
  if(!bcm2835_spi_begin()){
    printf("bcm2835_spi_begin failed.  Must run as root.\n");
    return 1;
  }

  //The following five functions define the SPI communication operating
  //parameters.  Some are default values but redundancy never hurt anyone.  

  //Sets bit order. The SX1278 is expecting to receive MSB first and will
  //also be transmitting back MSB first.  This is a default.  
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);

  //Sets the clock polarity and phase. The LoRa module asks for CPOL = 0
  //and CPHA = 0.  This also happens to be the default.  
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);

  //Sets the clock divider and therefore the clock speed.  The default is
  //65536 which results in a clock speed of 3.8 kHz. I'm using 2048 which
  //corresponds to 122 kHz.  Troubleshooting SPI connection.  
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_65536);

  //Specifies which chip select pins will be asserted when an SPI transfer
  //is made.  The RPI2 has two but we'll just be using RPI pin #24 also
  //known as "SPI_CE0_N". This is a default setting defined below.
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);

  //Configured the polarity under which the chip select is considered active.
  //The SX1278 looks for a chip select active low.  This is a default setting
  //given below.
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);


  //set a GPIO pin high to stabilize reset pin on sx1278
  //configure RPI pin 11 tp gpio output functionality
  //then set that pin high
  bcm2835_gpio_fsel(RPI_V2_GPIO_P1_11, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_set(RPI_V2_GPIO_P1_11); 

  //Checks device boot mode and enters LoRa standby regardless 
  uint8_t bootmode = read_reg(REG_OP_MODE);
  if((bootmode & 0x80) == 0X00 ){
    write_reg(REG_OP_MODE, FSK_SLEEP);
  }
  write_reg(REG_OP_MODE, LORA_SLEEP);
  write_reg(REG_OP_MODE, LORA_STANDBY);
  
  if(read_reg(REG_OP_MODE) != LORA_STANDBY){
    printf("There was a problem entering LORA_STANDBY.\n");
    return 1;
  }else{
    printf("Device has entered LORA_STANDBY.\n");
  }
  
  //reading relevant LoRa registers for diagnostics
  diagnose();

  //test tx
  //read fifo pointer to verify it initalizes to the Rx base 0x00
  read_reg(REG_FIFO_ADDR_PTR);
  //set SPI access to fifo to the Tx base reg
  write_reg(REG_FIFO_ADDR_PTR, FIFO_TX_BASE_ADDR);
  //enter some arbitrary payload and check that its there
  write_reg(REG_FIFO, 0xCC);
  //commence the Tx
  write_reg(REG_OP_MODE, LORA_TX);
  int i = 0;
  while(i<5000000){
    i++;
  }
  //confirm Tx
  read_reg(REG_IRQ_FLAGS);
  read_reg(REG_OP_MODE);

  
  bcm2835_spi_end();
  bcm2835_close();
  return 0;
  
}

//--------------------------------helper function implementations---------------------------------

//wrapper function that reads from a register
//argument is byte address of register to be read
//return value is data held in register
uint8_t read_reg(uint8_t addr){
  char tbuf[] = {addr, 0x00};
  char rbuf[] = {0x00, 0x00};
  bcm2835_spi_transfernb(tbuf, rbuf, sizeof(tbuf));
  printf("Read value 0x%02X from register 0x%02X.\n", rbuf[1], addr);
  return rbuf[1];
}

//wrapper function that writes to a register
//arguments are address to be written to and data to write
//returns last data held in register before the write
uint8_t write_reg(uint8_t addr, uint8_t data){
  char tbuf[] = {addr | 0x80, data};   //flag addr MSB high to indicate write op
  char rbuf[] = {0x00, 0x00};
  bcm2835_spi_transfernb(tbuf, rbuf, sizeof(tbuf));
  printf("Wrote value 0x%02X to register 0x%02X.\n", data, addr);
  return rbuf[1];
}

//diagnostic function that reads every #defined register except FIFO
//(because that would inadvertently increment the address pointer)
//prints outputs because it's just calling the read_reg() function
void diagnose(void){
  read_reg(REG_OP_MODE);
  read_reg(REG_RF_FREQ_MSB_MSB);
  read_reg(REG_RF_FREQ_MSB);
  read_reg(REG_RF_FREQ_LSB);
  read_reg(REG_PA_CONFIG);
  read_reg(REG_PA_RAMP);
  read_reg(REG_OCP);
  read_reg(REG_LNA);
  read_reg(REG_FIFO_ADDR_PTR);
  read_reg(REG_FIFO_TX_BASE_ADDR);
  read_reg(REG_FIFO_RX_BASE_ADDR);
  read_reg(REG_FIFO_RX_CURRENT_ADDR);
  read_reg(REG_IRQ_FLAGS_MASK);  
  read_reg(REG_IRQ_FLAGS);
  read_reg(REG_RX_NUM_BYTES);
  read_reg(REG_RX_PACKET_COUNT_MSB);
  read_reg(REG_RX_PACKET_COUNT_LSB);
  read_reg(REG_MODEM_STAT);
  read_reg(REG_PACKET_SNR);
  read_reg(REG_PACKET_RSSI);
  read_reg(REG_CURRENT_RSSI);
  read_reg(REG_HOP_CHANNEL);
  read_reg(REG_MODEM_CONFIG1);
  read_reg(REG_MODEM_CONFIG2);
  read_reg(REG_PREAMBLE_LEN_MSB);
  read_reg(REG_PREAMBLE_LEN_LSB);
  read_reg(REG_PAYLOAD_LEN);
  read_reg(REG_MAX_PAYLOAD_LEN);
  read_reg(REG_HOP_PERIOD);
  read_reg(REG_MODEM_CONFIG3);
  read_reg(REG_DETECT_OPTIMIZE);
  read_reg(REG_DETECT_THRESH);
  read_reg(REG_SYNC_WORD);
}
