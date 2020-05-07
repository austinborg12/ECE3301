#include <p18f4620.h>
#include <stdio.h>
#include <math.h>
#include <usart.h>


#pragma config OSC = INTIO67
#pragma config WDT = OFF
#pragma config LVP = OFF
#pragma config BOREN = OFF
#pragma config CCP2MX = PORTBE

#define SEC_LED PORTEbits.RE2
#define ACCESS_CFG 0xAC
#define START_CONV 0xEE
#define READ_TEMP 0xAA
#define CONT_CONV 0x02
#define ACK     1
#define NAK     0
#define _XTAL_FREQ 4000000

void interrupt high_priority chkisr();
void Init_timer_1();
void Init_ADC();
void do_init();
void Wait_Half_Sec();
void T0_ISR();
void INT0_ISR();
void INT1_ISR();
void INT2_ISR();
void init_UART();
void putchar(char);
void do_update_pwm(char);
void DS1621_Init();
char DS1621_Read_Temp();
void DS3231_Read_Time();

void I2C_Init(const unsigned long c)
{
  SSPCON1 = 0x28;             // SSP Module as Master
  SSPCON2 = 0;
  SSPADD = (_XTAL_FREQ/(4*c))-1;    // Setting Clock Speed
  SSPSTAT = 0;
  TRISC3 = 1;                       // Setting as input as given in datasheet
  TRISC4 = 1;                       // Setting as input as given in datasheet
}

void I2C_Wait() //gets hung here
{
  while ((SSPSTAT & 0x04) || (SSPCON2 & 0x1F)); //Transmit is in progress
}

void I2C_Start()
{
  I2C_Wait();    
  SEN = 1;                          // Initiate start condition
}

void I2C_ReStart()
{
  I2C_Wait();
  RSEN = 1;                         // Initiate repeated start condition
}

void I2C_Stop()
{
  I2C_Wait();
  PEN = 1;                          // Initiate stop condition
}

void I2C_Write(unsigned char data_out)
{
  I2C_Wait();
  SSPBUF = data_out;                // Write data to SSPBUF
}

unsigned char I2C_Read(unsigned char a)
{
  unsigned char temp;
  I2C_Wait();
  RCEN = 1;
  I2C_Wait();
  temp = SSPBUF;                    // Read data from SSPBUF
  I2C_Wait();
  ACKDT = (a)?0:1;                  // Acknowledge bit
  ACKEN = 1;                        // Acknowledge sequence
  return temp;
}


void I2C_Write_Address_Write_One_Byte(char Device, char Address, char Data)
{
  I2C_Start();                      // Start I2C protocol
  I2C_Write((Device << 1) | 0);     // Device address Write mode
  I2C_Write(Address);               // Send register address
  I2C_Write(Data);                  // Initialize data read
  I2C_Stop(); 
}

void I2C_Write_Cmd_Only(char Device, char Cmd)
{
  I2C_Start();                      // Start I2C protocol
  I2C_Write((Device << 1) | 0);     // Device address Write mode
  I2C_Write(Cmd);                   // Send Command
  I2C_Stop(); 
}

void I2C_Write_Cmd_Write_Data(char Device, char Cmd, char Data)
{
  I2C_Start();                      // Start I2C protocol
  I2C_Write((Device << 1) | 0);     // Device address Write mode
  I2C_Write(Cmd);                   // Send Command
  I2C_Write(Data);                  // Send Data
  I2C_Stop(); 
}

char I2C_Write_Cmd_Read_One_Byte(char Device, char Cmd)
{
char Data_Ret;    
  I2C_Start();                      // Start I2C protocol
  I2C_Write((Device << 1) | 0);     // Device address
  I2C_Write(Cmd);                   // Send register address
  I2C_ReStart();                    // Restart I2C
  I2C_Write((Device << 1) | 1);     // Initialize data read
  Data_Ret = I2C_Read(NAK);         //
  I2C_Stop(); 
  return Data_Ret;
}

char I2C_Write_Address_Read_One_Byte(char Device, char Address)
{
char Data_Ret;    
  I2C_Start();                      // Start I2C protocol
  I2C_Write((Device << 1) | 0);     // Device address
  I2C_Write(Address);               // Send register address
  I2C_ReStart();                    // Restart I2C
  I2C_Write((Device << 1) | 1);     // Initialize data read
  Data_Ret = I2C_Read(NAK);         //
  I2C_Stop(); 
  return Data_Ret;
}


int half_sec_cnt = 0;
int old_sec = -1;
int secs_passed = 0;
int INTflag0, INTflag1, INTflag2 ,T0_Flag = 0;
int T0flag, tach_cnt = 0;
unsigned char second, minute, hour, dow, day, month, year;

void main()
{
    Init_ADC();
    I2C_Init(1);
    do_init();
    init_UART();
    SEC_LED = 0;
    PORTCbits.RC2 = 1;
    
    int PWM_val = 0;
    do_update_pwm(PWM_val);
    
    while(1)
    {
        if (T0_Flag == 1)
        {
            T0_Flag = 0;
            printf("RPM = %i ", tach_cnt*60);
            printf("Duty Cycle = %i\r\n", PWM_val);
            DS1621_Init();
            int temp = DS1621_Read_Temp();
            printf("Temperature = %d ",temp);
            DS3231_Read_Time();
            printf ("%02x:%02x:%02x  %02x/%02x/%02x \r\n",hour,minute,second,month,day,year);
            
        }
        if(INTflag0 == 1)
        {
            INTflag0 = 0;
            PWM_val += 5;
            if(PWM_val == 105) PWM_val = 5;
            do_update_pwm(PWM_val);
        }
        if(INTflag1 == 1)
        {
            INTflag1 = 0;
            printf("Interupt on Pin 1! \r\n");
            if(PWM_val == 0) PWM_val = 100;
            else PWM_val -= 5;
            do_update_pwm(PWM_val);
        }
        if(INTflag2 == 1)
        {
            INTflag2 = 0;
            printf("Interupt on Pin 2! \r\n");
        }
        
    }

}

void interrupt high_priority chkisr()
{
    if(INTCONbits.TMR0IF == 1) T0_ISR();
    if(INTCONbits.INT0IF == 1) INT0_ISR();
    if(INTCON3bits.INT1IF == 1) INT1_ISR();
    if(INTCON3bits.INT2IF ==1) INT2_ISR();
    
}

void Init_timer_1()
{
    TMR1L = 0x00;
    T1CON = 0x03;
}

void Init_ADC()
{
    TRISB = 0x07;
    TRISE = 0x00;
    ADCON1 = 0x0F;
    INTCONbits.TMR0IE = 1;
    INTCONbits.TMR0IF = 0;
    INTCONbits.GIE = 1;
}

void do_init()
{
    OSCCON = 0x70;
    T0CON = 0x03;
    TMR0H = 0x0B;
    TMR0L = 0xDB;
    INTCONbits.TMR0IE = 1;
    INTCONbits.INT0IE = 1;
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT2E = 1;
    
    INTCONbits.INT0IF = 0;
    INTCON3bits.INT1IF = 0;    
    INTCON3bits.INT2IF = 0;
    
    INTCON2bits.INTEDG0 = 0;
    INTCON2bits.INTEDG1 = 0;
    INTCON2bits.INTEDG2 = 1;
            
    INTCONbits.GIE = 1;
    INTCON2bits.RBPU = 0;
    
    T0CONbits.TMR0ON = 1;
            
    Init_timer_1();
}

void Wait_Half_Sec()
{
    T0CON = 0x02;        // Timer 0, 16-bit mode, prescaler 1:8
    TMR0H = 0x3D;       // set the upper byte of TMR   
    TMR0L = 0x09;       // set the lower byte of TMR   
    INTCONbits.TMR0IF = 0;     // clear the Timer 0 flag   
    T0CONbits.TMR0ON = 1;     // Turn on the Timer 0  
    while (INTCONbits.TMR0IF == 0);   // wait for the Timer Flag to be 1 for done  
    T0CONbits.TMR0ON = 0;
    half_sec_cnt++;
}

void T0_ISR()
{
    INTCONbits.TMR0IF = 0;
    T0CONbits.TMR0ON = 0;
    TMR0H = 0x0B;
    TMR0L = 0xDB;
    SEC_LED =~ SEC_LED;
    tach_cnt = TMR1L;
    TMR1L = 0;
    T0CONbits.TMR0ON = 1;
    T0_Flag = 1;
}

void INT0_ISR()
{
    INTCONbits.INT0IF = 0;
    INTflag0 = 1;
    for (int j=0; j<20000;j++);
}

void INT1_ISR()
{
    INTCON3bits.INT1F = 0;
    INTflag1 = 1;
    for (int j=0; j<20000;j++);
}

void INT2_ISR()
{
    INTCON3bits.INT2IF = 0;
    INTflag2 = 1;
    for (int j=0; j<20000;j++);
}

void init_UART()                                                //Enables UART for serial communication
{
    OpenUSART (USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT & USART_CONT_RX & USART_BRGH_HIGH, 25);
}

void putchar(char c)
{
    while(!TRMT);
    TXREG = c;
}

void do_update_pwm(char duty_cycle)
{
    float dc_f;
    int dc_I;
    PR2 = 0b00000100 ;    // set the frequency for 25 Khz  
    T2CON = 0b00000111 ;  // 
    dc_f = ( 4.0 * duty_cycle / 20.0) ;  // calculate factor of duty cycle versus a 25 Khz  
    dc_I = (int) dc_f;  // get the integer part
    if (dc_I > duty_cycle) dc_I++;  // round up function
    CCP1CON = ((dc_I & 0x03) << 4) | 0b00001100;
    CCPR1L = (dc_I) >> 2;  
}

void DS1621_Init()
{
    char Device = 0x48;
    I2C_Write_Cmd_Write_Data (Device, ACCESS_CFG, CONT_CONV);
    I2C_Write_Cmd_Only(Device, START_CONV);
}

char DS1621_Read_Temp()
{
char Data_Ret;  
char Device = 0x48;
char Cmd = 0xAA;
  I2C_Start();                      // Start I2C protocol
  I2C_Write((Device << 1) | 0);     // Device address
  I2C_Write(Cmd);                   // Send register address
  I2C_ReStart();                    // Restart I2C
  I2C_Write((Device << 1) | 1);     // Initialize data read
  Data_Ret = I2C_Read(NAK);         //
  I2C_Stop(); 
  return Data_Ret;
}

void DS3231_Init()
{
    char Device = 0x68;
    I2C_Write_Cmd_Write_Data (Device, ACCESS_CFG, CONT_CONV);
    I2C_Write_Cmd_Only(Device, START_CONV);
}

void DS3231_Read_Time()
{  
  char Device = 0x68;
  char Address = 0x00;
  
  I2C_Start();                      // Start I2C protocol
  I2C_Write((Device << 1) | 0);     // Device address
  I2C_Write(Address);               // Send register address
  I2C_ReStart();                    // Restart I2C
  I2C_Write((Device << 1) | 1);     // Initialize data read
  second = I2C_Read(ACK);
  I2C_Stop(); 
  
}