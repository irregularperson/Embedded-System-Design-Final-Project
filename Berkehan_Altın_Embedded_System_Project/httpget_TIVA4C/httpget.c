#include <string.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Idle.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/drivers/GPIO.h>
#include <ti/net/http/httpcli.h>
#include <ti/drivers/I2C.h>
#include "Board.h"
#include <sys/socket.h>

extern Event_Handle event0;

#define SOCKETTEST_IP     "192.168.0.11"
#define TASKSTACKSIZE      4096
#define OUTGOING_PORT      37


extern  Semaphore_Handle semaphore0;// posted by Timer_ISR task and pended by getNTPTimeTask
extern  Semaphore_Handle semaphore1;// posted by serverSocketTask, getTempTask and pended by getTempTask
extern  Semaphore_Handle semaphore2;// posted by Timer_ISR task, serverSocketTask and pended by updateDateTimeEverySecondTask
extern  Semaphore_Handle semaphore_clock;
extern  Swi_Handle swi0;

int     calculatedTime;// raw1900Time is calculated
int     updatedNTPTime;// calculatedTime is updated in every second
int     ctr;
char    raw1900Time[3];
char    temp_sensor[20];
char    pres_sensor[20];
char    socketDataSensor[120];


uint8_t txBuffer[4];
uint8_t rxBuffer[30];
uint8_t rxBufferLong[22];
I2C_Handle i2c;
I2C_Params i2cParams;
I2C_Transaction i2cTransaction;


/*---------------------------------------------------------------------------------------------*/
void printError(char *errString, int code)
{
    System_printf("Error! code = %d, desc = %s\n", code, errString);
    BIOS_exit(code);
}
/*---------------------------------------------------------------------------------------------*/


Void Timer_ISR(UArg arg1)  //Timer that posts semaphores.
{
    Semaphore_post(semaphore2);// activate updateDateTimeEverySecondTask
   // Semaphore_post(semaphore0);// activate getNTPTimeTask

}
/*---------------------------------------------------------------------------------------------*/


Void updateDateTimeEverySecondTask(UArg arg1)
{
    while(1){
        // wait for the semaphore that Timer_ISR() will signal
        Semaphore_pend(semaphore2, BIOS_WAIT_FOREVER);

        calculatedTime  = raw1900Time[0]*16777216 +
                          raw1900Time[1]*65536 +
                          raw1900Time[2]*256 +
                          raw1900Time[3];

        updatedNTPTime = calculatedTime + 10800 + ctr++;

        Mailbox_post(mailbox1, &updatedNTPTime, BIOS_NO_WAIT);

        System_printf("Date: %s", ctime(&updatedNTPTime));// printed on console
        System_flush();
    }
}

/*---------------------------------------------------------------------------------------------*/


/*Void getNTPTimeTask(UArg arg0, UArg arg1)
{
        // wait for the semaphore that Timer_ISR() will signal
        Semaphore_pend(semaphore0, BIOS_WAIT_FOREVER);
        sendData2Server(SOCKETTEST_IP, OUTGOING_PORT, calculatedTime, strlen(calculatedTime));
}*/

/*---------------------------------------------------------------------------------------------*/


short AC1, AC2, AC3, B1, B2, MB, MC, MD;                           // calibration variables
unsigned short AC4, AC5, AC6;                                      // calibration variables
long UT, UP;                                                       // uncompensated temperature and pressure
float B3, B4, B6, B7, X1t, X1p, X2t, X2p, X3p, B5t, B5p, Altitude;
/*---------------------------------------------------------------------------------------------*/


void BMP180_getPressureCalibration(void)
{

    txBuffer[0] = 0xAA;
    i2cTransaction.slaveAddress = 0x77;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 22;
    if (I2C_transfer(i2c, &i2cTransaction)) {

    //System_printf("Calibration data acquired\n");
    AC1 = rxBuffer[0]<<8 | rxBuffer[1];
    AC2 = rxBuffer[2]<<8 | rxBuffer[3];
    AC3 = rxBuffer[4]<<8 | rxBuffer[5];
    AC4 = rxBuffer[6]<<8 | rxBuffer[7];
    AC5 = rxBuffer[8]<<8 | rxBuffer[9];
    AC6 = rxBuffer[10]<<8 | rxBuffer[11];
    B1 = rxBuffer[12]<<8 | rxBuffer[13];
    B2 = rxBuffer[14]<<8 | rxBuffer[15];
    MB = rxBuffer[16]<<8 | rxBuffer[17];
    MC = rxBuffer[18]<<8 | rxBuffer[19];
    MD = rxBuffer[20]<<8 | rxBuffer[21];

    }
}
/*---------------------------------------------------------------------------------------------*/


void BMP180_startTemperatureAcquisition(void)
{
    txBuffer[0] = 0xf4; // control register
    txBuffer[1] = 0x2e; // conversion command
    i2cTransaction.slaveAddress = 0x77; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 2; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 0; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
    //System_printf("Temperature acquisition initiated\n");
    }
}
/*---------------------------------------------------------------------------------------------*/


float BMP180_getTemperature(void)
{
    float temp;
    txBuffer[0] = 0xf6; // temp register
    i2cTransaction.slaveAddress = 0x77; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 1; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 2; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
    //System_printf("Temperature value acquired\n");
    }
    UT = rxBuffer[0]<<8 | rxBuffer[1]; //UT = raw temperature data
    //System_printf("Uncompansated Temperature : %d\n", UT);
    //compute temperature
    X1t = ((UT - AC6) * AC5) >> 15;
    X2t = (MC << 11) / (X1t + MD);
    B5t = X1t + X2t;
    temp = ((B5t + 8) / 16) / 10;
    return temp;
}
/*---------------------------------------------------------------------------------------------*/


void BMP180_startPressureAcquisition(void)
{
    txBuffer[0] = 0xf4; // control register
    txBuffer[1] = 0x34; // conversion command
    i2cTransaction.slaveAddress = 0x77; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 2; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 0; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
    //System_printf("Pressure acquisition initiated\n");
    }
}
/*---------------------------------------------------------------------------------------------*/


float BMP180_getPressure(void)
{
    float pressure;
    txBuffer[0] = 0xf6; // temp register
    i2cTransaction.slaveAddress = 0x77; // 0x77
    i2cTransaction.writeBuf = txBuffer; // transmit buffer
    i2cTransaction.writeCount = 1; // two bytes
    i2cTransaction.readBuf = rxBuffer; // receive buffer
    i2cTransaction.readCount = 2; // 2 bytes
    if (I2C_transfer(i2c, &i2cTransaction)) {
        //System_printf("Pressure value acquired\n");
    }
    UP = rxBuffer[0]<<8 | rxBuffer[1]; //UT = raw pressure data
    //System_printf("Uncompansated Pressure : %d\n", UP);
    B6 = B5t - 4000;
    X1p = (B2 * (B6 * B6 / 4096)) / 2048;
    X2p = AC2 * B6 / 2048;
    X3p = X1p = X2p;
    B3 = ((((long)AC1 * 4 + X3p)) + 2) / 4;
    X1p = AC3 * B6 / 8192;
    X2p = (B1 * (B6 * B6 / 4096)) / 65536;
    X3p = ((X1p + X2p) + 2) / 4;
    B4 = AC4 * (unsigned long)(X3p + 32768) / 32768;
    B7 = ((unsigned long)UP - B3) * (50000);
    if (B7 < 0x80000000) {
        pressure = (B7 * 2) / B4;
    }
    else {
        pressure = (B7 / B4) * 2;
    }
    X1p = (pressure / 256) * (pressure / 256);
    X1p = (X1p * 3038) / 65536;
    X2p = (-7357 * pressure) / 65536;
    pressure = pressure + (X1p + X2p + 3791) / 16;
    return pressure;
}
/*---------------------------------------------------------------------------------------------*/


float BMP180_calculateAltitude(float pressure)
{
    float alt;
    alt = 44330.0f * (1.0f - powf(pressure / 101325.0f, 1 / 5.255f)); //bmp180 datasheette yer alýyordu.
    return alt;
}
/*---------------------------------------------------------------------------------------------*/


short AC1, AC2, AC3, B1, B2, MB, MC, MD; // calibration variables
unsigned short AC4, AC5, AC6; // calibration variables

/*---------------------------------------------------------------------------------------------*/
void getPressureCalibration(void)
{
    txBuffer[0] = 0xAA;
    i2cTransaction.slaveAddress = 0x77; // 0x77
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 22;
    I2C_transfer(i2c, &i2cTransaction);
    AC1 = rxBufferLong[0]<<8 | rxBufferLong[1];
    AC2 = rxBufferLong[2]<<8 | rxBufferLong[3];
    AC3 = rxBufferLong[4]<<8 | rxBufferLong[5];
    AC4 = rxBufferLong[6]<<8 | rxBufferLong[7];
    AC5 = rxBufferLong[8]<<8 | rxBufferLong[9];
    AC6 = rxBufferLong[10]<<8 | rxBufferLong[11];
    B1 = rxBufferLong[12]<<8 | rxBufferLong[13];
    B2 = rxBufferLong[14]<<8 | rxBufferLong[15];
    MB = rxBufferLong[16]<<8 | rxBufferLong[17];
    MC = rxBufferLong[18]<<8 | rxBufferLong[19];
    MD = rxBufferLong[20]<<8 | rxBufferLong[21];
}
/*---------------------------------------------------------------------------------------------*/


void initializeI2C()
{
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_100kHz;         // It can be I2C_400kHz orI2C_100kHz
    i2c = I2C_open(Board_I2C0, &i2cParams); // actually I2C7
}

void closeI2C(void)
{
    I2C_close(i2c);
}
/*---------------------------------------------------------------------------------------------*/


void sendData2Server(char *serverIP, int serverPort, char *data, int size)
{
    int sockfd;
    struct sockaddr_in serverAddr;

    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockfd == -1)
    {
        System_printf("Socket not created");
        BIOS_exit(-1);
    }

    memset(&serverAddr, 0, sizeof(serverAddr));  /* clear serverAddr structure */
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(serverPort);     /* convert port # to network order */
    inet_pton(AF_INET, serverIP, &(serverAddr.sin_addr));

    int connStat = connect(sockfd, (struct sockaddr *)&serverAddr, /* connecting….*/
                  sizeof(serverAddr));
    if(connStat < 0) {
        System_printf("Error while connecting to server\n");
        if (sockfd > 0)
            close(sockfd);
        BIOS_exit(-1);
    }

    int numSend = send(sockfd, data, size, 0);       /* send data to the server*/
    if(numSend < 0) {
        System_printf("Error while sending data to server\n");
        if (sockfd > 0) close(sockfd);
        BIOS_exit(-1);
    }

    if (sockfd > 0) close(sockfd);
}
/*---------------------------------------------------------------------------------------------*/


Void ClockFunc()
{
    Semaphore_post(semaphore_clock);
}
/*---------------------------------------------------------------------------------------------*/


Void sensorTask() //tmp006 da burada yer alýyor, semaphore_clock post edildiði vakit çalýþýyor bu task
{
    while(1)
    {
            Semaphore_pend(semaphore_clock, BIOS_WAIT_FOREVER);
            unsigned int i;
            uint16_t temperature;
            initializeI2C();

            txBuffer[0] = 0x01; // register=0x01 (temperature)
            i2cTransaction.slaveAddress = Board_TMP006_ADDR; // 0x41
            i2cTransaction.writeBuf = txBuffer;
            i2cTransaction.writeCount = 1;
            i2cTransaction.readBuf = rxBuffer;
            i2cTransaction.readCount = 2;
            if (I2C_transfer(i2c, &i2cTransaction)) {
            temperature = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);
            if (rxBuffer[0] & 0x80)
            {
                temperature |= 0xF000;
            }
            temperature /= 32;
            sprintf(temp_sensor, sizeof(temp_sensor), "%d", temperature);
            }
            else
            {
                System_printf("I2C Bus fault\n");
            }
            System_flush();

            float temp, press, alt;
            BMP180_getPressureCalibration();
            BMP180_startTemperatureAcquisition();
            System_flush();
            Task_sleep(5);
            temp = BMP180_getTemperature();
            BMP180_startPressureAcquisition();
            System_flush();
            Task_sleep(5);
            press = BMP180_getPressure();
            alt = BMP180_calculateAltitude(press);

            closeI2C();
            press = (int)press / 100;
            sprintf(pres_sensor, sizeof(pres_sensor), "%d", (int)press);

            //Event_post(event0, Event_Id_00); // post Event_Id_00
        }
}
/*---------------------------------------------------------------------------------------------*/


Void socketTask(UArg arg0, UArg arg1)
{
    while(1)
    {
        //Event_pend(event0,Event_Id_00 + Event_Id_01 , Event_Id_NONE,  BIOS_WAIT_FOREVER);


        strcpy(socketDataSensor, temp_sensor); //temp_sensoru socketDataSensore kopyalýyor
        strcat(socketDataSensor, "C, ");       //concatanate ediyor
        strcat(socketDataSensor, pres_sensor);
        strcat(socketDataSensor, "hPa         ");

        sendData2Server(SOCKETTEST_IP, 5011, socketDataSensor, strlen(socketDataSensor));

        Task_sleep(1000);
    }
}
/*---------------------------------------------------------------------------------------------*/


void netIPAddrHook(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
       static Task_Handle taskHandle1, taskHandle2, taskHandle3;
       Task_Params taskParams;
       Error_Block eb;
       Error_init(&eb);

       Task_Params_init(&taskParams);
       taskParams.stackSize = TASKSTACKSIZE;
       taskParams.priority = 1;
       taskHandle2 = Task_create((Task_FuncPtr)socketTask, &taskParams, &eb);
   }
/*---------------------------------------------------------------------------------------------*/


int main(void)
{
    Board_initGeneral();
    Board_initGPIO();
    Board_initEMAC();
    Board_initI2C();

    GPIO_write(Board_LED0, Board_LED_ON);

    semaphore_clock=Semaphore_create(0,NULL,NULL);

    System_printf("Example is running...");
    System_flush();
    BIOS_start();

    return (0);
}
