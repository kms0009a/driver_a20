
#ifndef A20_H_
#define A20_H_

/**
  ******************************************************************************
  * @file    Olinuxino_A20/I2C/a20.h 
  * @author  KOUMAD Salim, Mraichi Rached, Sabbani Ahmed
  * @version V0.0.1
  * @date    25-02-2015
  * @brief   Main program body
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define _4K_DIV_BASE_	0x01C20000	
#define PIO_BASE_ADDR   0x800                   // Base addr of PIO	
#define PIO_END_ADDR 	_4K_DIV_BASE_ + 0x1000  // Size of PIO is 1k octets
#define PIO_SIZE        0x1000                  // 4K octets
#define PBCFG2          0x2C                    // PBCFG Offset
#define PBDATA_REG      0x34                    // PBDATA Offset            


#define MOD_IO_ADDRESS_DEFAULT  0x58                    // Default slave address of the mod_io
#define MOD_IO_ADDRESS          MOD_IO_ADDRESS_DEFAULT  // Address to use in communications with MOD_IO
#define MAX_ANALOG_VALUE        1023                    // (1111111111) = 3.3 Volts


/* DEVICE PART */

#define MODIO_CLASS      "modio_class"  // To use in udev to create class
#define NB_MODIO_DEVICES 12             // Number of devices (4 Outputs, 4 Inputs, 4 Optoc)
#define FIRST_MINOR      0              // First Minor to use when creating devices with udev

/* PATH TO FILES IN /DEV/ , USED BY UDEV */

#define OUTPUT1_FILE "output1"     
#define OUTPUT2_FILE "output2"
#define OUTPUT3_FILE "output3"
#define OUTPUT4_FILE "output4"

#define OPTO1_FILE  "opto1"
#define OPTO2_FILE  "opto2"
#define OPTO3_FILE  "opto3"
#define OPTO4_FILE  "opto4"

#define ANINPUT1_FILE  "aninput1"
#define ANINPUT2_FILE  "aninput2"
#define ANINPUT3_FILE  "aninput3"
#define ANINPUT4_FILE  "aninput4"

/* Private typedef -----------------------------------------------------------*/

/**
 *  @name rw_mode_t type
 *  @brief I2C Command mode , read (request a value from the slave) or
 *         write to make an action on the slave.
 */ 
typedef enum _rw_mode
{
    READ_MODE  = 1, // 8th bit of the first octet (wich containts slave address) 
    WRITE_MODE = 0  // 8th bit of the first octet (wich containts slave address)
    
}rw_mode_t;

/**
 * @name bit_status_t type
 * @brief Represent state of a bit : 1 = HIGH, 0 = LOW
 */
typedef enum _bit_state
{
    HIGH = 1, // High level +V >> 0
    LOW  = 0  // Low level +v ~~ 0
    
}bit_status_t;


/**
 *  @name pin_config_t type
 *  @brief Position of SDA and SCL in 'data' and 'config' registers
 *  @details SDAx_BIT is the position of the SDA bit in the data register to set/read value
 *           SDAx_PIN is the position of the SDA bit in the cfg register, to set Input/Output mode.
 */
typedef enum _pin_config
{
	SDA2_BIT = 21,  // SDA2_BIT shift in data register
	SCL2_BIT = 20,  // SCL2_BIT shift in data register
	SDA2_PIN = 20,  // SDA2_PIN shift in cfg register
	SCL2_PIN = 16   // SDA2_PIN shift in cfg register
	
}pin_config_t;

/** 
 *  @name ret_t type
 * 	@brief return status { OK, ERR, ACK, NOACK }
 */

typedef enum _return_values
{
    OK    = 0,
    ERR   = 1,
    ACK   = 2,
    NOACK = 3
    
}ret_t;

/**
 *  @name dev_minor_t
 * 	@brief Each <b>modio<\b> board devices must have one specific minor
 * 	@details MOD-IO board has: 4 ANALOG INPUTS, 4 OPTOCOUPLERS, 4 RELAYS
 */

typedef enum _minor_for_devices 
{

    ANINPUT1_MINOR = 0,
    ANINPUT2_MINOR = 1,
    ANINPUT3_MINOR = 2,
    ANINPUT4_MINOR = 3,
    OUTPUT1_MINOR  = 4,
    OUTPUT2_MINOR  = 5,
    OUTPUT3_MINOR  = 6,
    OUTPUT4_MINOR  = 7,
    OPTO1_MINOR    = 8,
    OPTO2_MINOR    = 9,
    OPTO3_MINOR    = 10,
    OPTO4_MINOR    = 11
	
}dev_minor_t;

/** @name status_t
 * 	@brief Typedef used to represent state of Relay_State or Optocoupler_State (ON or OFF).
 */ 

typedef enum _state_on_off
{
    ON  = 1,  
    OFF = 0
    
}status_t;

/**
 *  @name commant_t type
 *  @brief a set of all MOD-IO I2C commands (8 bits).
 */      
typedef enum _modio_i2c_addr_a_command
{

    NO_COMMAND = 0x00,   // Empty command
    SET_OUTPUT = 0x10,   // Command to set relai
    GET_OPTO   = 0x20,   // Command to get optocoupler value
    GET_AIN1   = 0x30,   // Command to get Analog Input 0 value in 10 bits
    GET_AIN2   = 0x31,   // Command to get Analog Input 1 value in 10 bits
    GET_AIN3   = 0x32,   // Command to get Analog Input 2 value in 10 bits
    GET_AIN4   = 0x33,   // Command to get Analog Input 3 value in 10 bits
    SET_ADDR   = 0xF0,   // Command to set slave address
    UNKNOWN    = 0xFFFF  // Unkown command
    
}command_t;

/**
 *  @name output_id_t
 * 	@brief ID of each output (Relay) state.
 *  @details The output (Relay) state is returned by the MOD-IO board in an octet of 8bits
 *           Returned_Value = 0 0 0 0 Output3 Output2 Output1 Output0. 
 * 			 These ID's are used as MASK to get the wanted Output(i) state from the returned 
 * 			 8 bit value. Also to set the value of an Output, a unique value of 8 bits is 
 * 			 sent with the command. For example to set Output 1 and 2 the data sent is
 * 			 00000110 for 0 0 0 0 Output3=0 Output2=1 Output1=1 Output0=0
 */
typedef enum _outputs_IDs
{
    OUTPUT1_ID  = 0x01, // 0001
    OUTPUT2_ID  = 0x02, // 0010
    OUTPUT3_ID  = 0x04, // 0100
    OUTPUT4_ID  = 0x08, // 1000
    OUTPUTS_ALL = 0x0F  // 1111
    
}output_id_t;

 /**
 *  @name output_id_t
 * 	@brief ID of each optocoupler state.
 *  @details The optocoupler state is returned by the MOD-IO board in an octet of 8bits
 *           Returned_Value = 0 0 0 0 Opto3 Opto2 Opto1 Opto0. 
 * 			 These ID's are used as MASK to get the wanted Opto(i) state from the returned 
 * 			 8 bit value.
 */
typedef enum _optocouplers_IDs
{
    // 00000001 = First Octocoupler [ opto0State = |0|0|0|0|O3|O2|O1|O0| & |0|0|0|0|0|0|0|1| ] = |0|0|0|0|0|0|0|O0|
    OPTO1_ID = 0x01, 	
    // 00000010 = Secnd Octocoupler [ opto1State = |0|0|0|0|O3|O2|O1|O0| & |0|0|0|0|0|0|1|0| ] = |0|0|0|0|0|0|O1|0|]	
    OPTO2_ID = 0x02, 		
    // 00000100 = Secnd Octocoupler [ opto1State = |0|0|0|0|O3|O2|O1|O0| & |0|0|0|0|0|1|0|0| ] = |0|0|0|0|0|O2|0|0|]
    OPTO3_ID = 0x04, 		
    // 00001000 = Secnd Octocoupler [ opto1State = |0|0|0|0|O3|O2|O1|O0| & |0|0|0|0|1|0|0|0| ] = |0|0|0|0|O3|0|0|0|]
    OPTO4_ID = 0x08 	
    	
}opto_id_t;

/**
 *  @name wait_params_t
 * 	@brief values to manage waiting time before receiving ACK	
 */
 
typedef enum _wait_params_
{
    MAX_DELAY    = 100, //Maximum time to wait ACK (in microseconds)
    MAX_ATTEMPTS = 5,   //Maximum of retransmissions
    DELAY        = 5    //Time between every send/receive in I2C Communication (in microseconds)

}wait_params_t;


/* Private function prototypes -----------------------------------------------*/

/**
 *      @name setOutput    -    turn On/Off one or more output
 *      @param output 8 bits represent the output state to modify : 0 0 0 0 O4 O3 O2 O1
 *      @param output_on_off the wanted state of output given through @output	
 *		@details For example to set O1 to On : the 
 *               The code returned is OUTPUT_SETTED if no problem was encountred
 *	
 */
 
status_t setOutput(uint8_t output, status_t output_on_off);

/**
 *      @name getOptoStates    -       get state of each optocoupler at one time
 * 		@param opto_id 
 *      @details The byte returned is a byte of the form: 0 0 0 0 Oc4 Oc3 Oc2 Oc1 
 *      		 if (Oci = 0) => Oci is Off ; else Oci is On.
 */
status_t getOptoStates(uint8_t opto_id);

/** 
 *      convVoltage    -       Convert ADC value (in 10 bits) to a readable value in Volts
 *		@l_byte : Low Byte is the first 8 bits returned by MOD-IO
 *      @h_byte : High Byte is the byte which contain two bits returned from MOD-IO
 *      
 *      The value returned is the voltage converted from @l_byte and @h_byte 
 */
uint16_t convVoltage(uint8_t l_byte, uint8_t h_byte);

 /**
 *      getAnalog    -       Convert ADC value (in 10 bits) to a readable value in Volts
 *		@id : the id of the INPUT wanted to read the value from. It can be (MOD_IO_IN1, MOD_IO_IN2, MOD_IO_IN3, MOD_IO_IN4).
 *
 *      The value returned is the convertion (with convVoltage) of the 2 bytes received from MOD-IO
 */
uint16_t getAnalog(analog_id_t id);

/**
 *      setAddress    -       Set address
 *		@newAddr : the id of the INPUT wanted to read the value from. It can be (MOD_IO_IN1, MOD_IO_IN2, MOD_IO_IN3, MOD_IO_IN4).
 *
 *      The value returned is the convertion (with convVoltage) of the 2 bytes received from MOD-IO
 */
int setAddress(uint8_t newAddr);



/**************************/
/* MODIO FUNCTIONS ACCESS */
/**************************/

/**
 *  @name Pin Functions Definition 
 */
/*@{*/

void set_scl_input(void);

void set_scl_output(void);

void set_sda_input(void);

void set_sda_output(void);

void set_scl(bit_status_t state);

void set_sda(bit_status_t state);

bit_status_t get_sda(void);

bit_status_t get_scl(void);

/*@}*/

/**
 *  @name Basic I2C Functions Definition 
 */
/*@{*/
/**
 * @name i2c_duty
 * @brief wait a specified time between send/receive
 */
ret_t i2c_duty(void);

/**
 * @name i2c_start
 * @brief I2C start sequence
 */
ret_t i2c_start(void);

/**
 * @name i2c_wait_ack
 * @brief Wait an ACK for a limited time
 */
ret_t i2c_wait_ack(void);

/**
 * @name i2c_stop
 * @brief I2C stop sequence
 */
ret_t i2c_stop(void);

/**
 * @name i2c_read
 * @brief read 8 bits of data after a command request
 */
uint8_t i2c_read(void);

/**
 * @name i2c_send
 * @brief send 8 bits of data to the slave
 */
ret_t i2c_send(uint8_t byte);

/**
 * @name i2c_send_ack
 * @brief send ACK to the slave
 */
ret_t i2c_send_ack(void);

/**
 * @name i2c_send_no_ack
 * @brief send NOACK to the slave
 */
ret_t i2c_send_no_ack(void);

/**
 * @name i2c_ping
 * @brief Make a ping test to know if the slave is reachable or not
 * @param addr the adresse of the slave to ping
 */
ret_t i2c_ping(short addr);

/*@}*/

#endif

/**********************************************************END OF FILE****/
