/**
  ******************************************************************************
  * @file    Olinuxino_A20/I2C/a20.c
  * @author  KOUMAD Salim, Mraichi Rached, Sabbani Ahmed
  * @version V0.0.1
  * @date    25-02-2015
  * @brief   Driver Code
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/cdev.h> // for cdev structure 
#include <linux/slab.h> // for kmalloc() and kfree()
#include <linux/uaccess.h> // for copy_to_user and copy_from_user
#include <linux/errno.h> // error code and defines: -EINTR(interrupted system call) or -EFAULT(bad address)
#include <linux/thread_info.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <asm/io.h>
#include <linux/delay.h>
#include "a20.h"

/* Global Variables ------------------------------------------------------------------*/

static status_t output1, output2, output3, output4;

// ioremap address
static void __iomem *addr;
// Registers addresses
static uint32_t *cfg, *data;
// Return variable to check if functions executed correctly
ret_t ret;
// Check return from copy_to_user function
int copy_ret;

/* Declare device concern variables  */

// Minor of the opened device
static uint8_t curr_minor;

// A trick to fix CAT program acces to driver
static int lock_cat = 0;

// The state of a specific relay selected by the current_minor
bit_status_t relay_flag;

// The dev_t for our device 
dev_t modio_dev;

// The cdev for our device 
struct cdev *modio_cdev;

// modio_class for udev
static struct class *modio_class;

// slave MOD-IO address
static short param_slave_addr = MOD_IO_ADDRESS_DEFAULT;

/* Declare Temporary variables   */

//Optocouplers states
uint8_t optoStates;

//variable which contains last states of outputs (output1, output2, output3, output4)
uint8_t output_old_states,
        output_new_states;

// Read values from registers
uint32_t cfg_tmp, data_tmp;

// Command to read value from analog input
uint8_t analogReadCommand;

// Analog 
uint16_t analog_value;

// Return the Analog value in string format with copy_to_user fct
char analog_strvalue[11];

// Return optocoupler state ( '0' or '1' ) with copy_to_user fct
char opto_str[2];

// Return the output state ( '0' or '1' ) with copy_to_user fct
char output_str[2];

// Low byte and High byte received from MOD_IO Get Analog Command
uint8_t l_byte,
        h_byte;

//Calculate waiting ACK/NOACK response time
int delay;

//Received data from I2C Bus
uint8_t received_data;

//Name of each device file to create with udev
char* device_names[12] = {  
                            ANINPUT1_FILE,
                            ANINPUT2_FILE,
                            ANINPUT3_FILE,
                            ANINPUT4_FILE,
                            OUTPUT1_FILE, 
                            OUTPUT2_FILE,
                            OUTPUT3_FILE,
                            OUTPUT4_FILE,
                            OPTO1_FILE,
                            OPTO2_FILE,
                            OPTO3_FILE,
                            OPTO4_FILE
                          };

// Variable of for loops
int index;

/* Private function -------------------------------------------------------------------------*/

/// Module Param to set slave Addr
module_param(param_slave_addr, short, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);


/*
	Pin Functions Definition
*/

void set_scl_input(void) {
    cfg_tmp = ioread32(cfg);
	iowrite32(cfg_tmp & ~(0x3 << SCL2_PIN), cfg);

}

void set_scl_output(void) {
    cfg_tmp = ioread32(cfg);
	iowrite32((cfg_tmp & ~(0x3 << SCL2_PIN)) | (0x1 << SCL2_PIN), cfg);

}

void set_sda_input(void) {
    cfg_tmp = ioread32(cfg);
	iowrite32(cfg_tmp & ~(0x3 << SDA2_PIN), cfg);

}

void set_sda_output(void) {
    cfg_tmp = ioread32(cfg);
	iowrite32((cfg_tmp & ~(0x3 << SDA2_PIN)) | (0x1 << SDA2_PIN), cfg);

}

void set_scl(bit_status_t state) {
    data_tmp = ioread32(data);
	switch(state)
	{
		case HIGH:
			iowrite32((data_tmp & ~(0x1 << SCL2_BIT)) | (0x1 << SCL2_BIT), data); //Ecrire 1
			break;
		case LOW:
			iowrite32(data_tmp & ~(0x1 << SCL2_BIT), data); //Ecrire 1
			break;
	}
}

void set_sda(bit_status_t state) {
    data_tmp = ioread32(data);
	switch(state)
	{
		case HIGH:
			iowrite32((data_tmp & ~(0x1 << SDA2_BIT)) | (0x1 << SDA2_BIT), data); //Ecrire 1
			break;
		case LOW:
			iowrite32(data_tmp & ~(0x1 << SDA2_BIT), data); //Ecrire 1
			break;
	}
}

bit_status_t get_sda(void) {
    data_tmp = ioread32(data);
	return ( (data_tmp & (0x1 << SDA2_BIT)) ? HIGH : LOW ) ;
}

bit_status_t get_scl(void) {
    data_tmp = ioread32(data);
	return ( (data_tmp & (0x1 << SCL2_BIT)) ? HIGH : LOW ) ;
}

/*
	Basic I2C Functions Definition
*/

ret_t i2c_duty(void) {
    udelay(DELAY);
	return OK;
}

ret_t i2c_start(void) {
    printk(KERN_ALERT "[a20_modio]: ==> Send Start.");
    set_sda_output();
	set_scl_output();
	set_sda(HIGH);
	i2c_duty();
	set_scl(HIGH);
	i2c_duty();
	set_sda(LOW);
	i2c_duty();
	set_scl(LOW);
	i2c_duty();
	printk(KERN_ALERT "[a20_modio]: Start Sent. ==>");
	return OK;
}

ret_t i2c_wait_ack(void) {
    delay = 0;
    printk(KERN_ALERT "[a20_modio]: Waiting ACK ...");
    set_sda_output();
	set_sda(HIGH);
	i2c_duty();
	set_sda_input();
	i2c_duty();
	set_scl(HIGH);
	i2c_duty();
	while(get_sda() == HIGH){
		i2c_duty();
		delay++;
		if (delay * 5 > MAX_DELAY)
		{
			set_scl(LOW);
			i2c_duty();
			printk(KERN_ALERT "[a20_modio]: NOACK Received ...");
			return NOACK;
		}
	}
	set_scl(LOW);
	i2c_duty();
    printk(KERN_ALERT "[a20_modio]: ACK Received ...");
	return ACK;
}

ret_t i2c_stop(void) {
    printk(KERN_ALERT "[a20_modio]: ==> Send Stop.");
    set_sda_output();
	set_sda(LOW);
	i2c_duty();
	set_scl(HIGH);
	i2c_duty();
	set_sda(HIGH);
	i2c_duty();
	printk(KERN_ALERT "[a20_modio]: Stop Sent ==>.");
	return OK;
}

uint8_t i2c_read(void) {
    printk(KERN_ALERT "[a20_modio]: Receiving data ...");
    received_data = 0x0;
	set_sda_input();
	for (index = 7; index >= 0; index--)
	{
		set_scl(HIGH);
		i2c_duty();
		received_data |= (get_sda() == HIGH) ?  (1 << index) : 0x0;
		i2c_duty();
		set_scl(LOW);
		i2c_duty();
	}
	printk(KERN_ALERT "[a20_modio]: Data received: 0x%X", received_data);
	return received_data;
}

ret_t i2c_send(uint8_t byte) {

    printk(KERN_ALERT "[a20_modio]: Sending data : 0x%X", byte);
	set_sda_output();
	for (index = 7; index >= 0; index--)
	{
		if (byte & (HIGH << index)) 
		{
			set_sda(HIGH);
		}
		else
		{	
			set_sda(LOW);
		}
		i2c_duty();
		set_scl(HIGH);
		i2c_duty();
		set_scl(LOW);
		i2c_duty();
	}
	printk(KERN_ALERT "[a20_modio]: Data sent .");
	return OK;
}

ret_t i2c_send_ack(void) {

    set_sda_output();
	set_sda(LOW);
	i2c_duty();
	set_scl(HIGH);
	i2c_duty();
	set_scl(LOW);
	i2c_duty();
    printk(KERN_ALERT "[a20_modio]: ACK Sent. ");
	return OK;
}

ret_t i2c_send_no_ack(void) {

    set_sda_output();
	set_sda(HIGH);
	i2c_duty();
	set_scl(HIGH);
	i2c_duty();
	set_scl(LOW);
	i2c_duty();
    printk(KERN_ALERT "[a20_modio]: NOACK Sent. ");
	return OK;
}

ret_t i2c_ping(short addr) {

    i2c_start();
    i2c_duty();
    i2c_send((addr << 1) | WRITE_MODE);
    i2c_duty();
    if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		printk(KERN_ALERT "i2c_ping return: BUSY");
		return NOACK;
	}
	i2c_stop();
	i2c_duty();
	return ACK;
}

/************************************/
/*     MODIO FUNCTIONS ACCESS       */
/************************************/

/************************************/
/*     SETOUTPUT                    */
/************************************/
status_t setOutput(uint8_t output, status_t output_on_off)
{
    output_new_states = output_old_states;
	
	printk(KERN_ALERT "[a20_modio]: COMMANDE D'OUVERTURE/FERMETURE DES RELAIS \n");
	// Construire la commande 
	output_new_states = (output_on_off == ON) ? output_new_states | output : output_new_states & ~(output);

	
	i2c_start();
	i2c_duty();
	printk(KERN_ALERT "[a20_modio]: Envoi du premier octet : 0x%X\n", (param_slave_addr << 1) | WRITE_MODE);
	i2c_send((param_slave_addr << 1) | WRITE_MODE);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return ERR;
	}
	i2c_duty();
	printk(KERN_ALERT "[a20_modio]: Envoi de la commande d'ouverture.");
	i2c_send(SET_OUTPUT);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return ERR;
	}
	i2c_duty();
	printk(KERN_ALERT "[a20_modio]: Envoi des parametres de la commande : 0x%X\n", output_new_states);
	i2c_send(output_new_states);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return ERR;
	}
	i2c_duty();
	i2c_stop();
	i2c_duty();
	
	//if (output & (1 << 0))
	output1 = (output_new_states & OUTPUT1_ID) ? ON : OFF;
	//if(output & (1 << 1))
	output2 = (output_new_states & OUTPUT2_ID) ? ON : OFF;
	//if (output & (1 << 2))
	output3 = (output_new_states & OUTPUT3_ID) ? ON : OFF;
	//if (output & (1 << 3))
	output4 = (output_new_states & OUTPUT4_ID) ? ON : OFF;

	printk(KERN_ALERT "[a20_modio]: Modification de l'état des relais : (%s, %s, %s, %s)\n", 
								(output1 == ON) ? "ON " : "OFF",
								(output2 == ON) ? "ON " : "OFF",
								(output3 == ON) ? "ON " : "OFF",
								(output4 == ON) ? "ON " : "OFF");

	output_old_states = output_new_states;
	printk(KERN_ALERT "[a20_modio]: FIN DE L'OPERATION AVEC SUCCES.\n");

	return OK;
}

/************************************/
/*     OPTOSTATES                   */
/************************************/
status_t getOptoStates(uint8_t opto_id) {

    optoStates = 0xFF;
	printk(KERN_ALERT "[a20_modio]: Reading optocouplers states\n");
	// Etape 1 : Envoi de la commande
	i2c_start();
	i2c_duty();
	i2c_send((param_slave_addr << 1) | WRITE_MODE);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return optoStates;
	}
	i2c_duty();
	i2c_send(GET_OPTO);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return optoStates;
	}
	i2c_duty();
	i2c_stop();
	i2c_duty();
	i2c_duty();

	// Etape 2 : Lecture de l'etat des optocoupleurs
	i2c_start();
	i2c_duty();
	i2c_send((param_slave_addr << 1) | READ_MODE);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return optoStates;
	}
	i2c_duty();
	optoStates = i2c_read();
	i2c_duty();
	i2c_send_ack();
	i2c_duty();
	i2c_stop();
	i2c_duty();
	printk(KERN_ALERT "[a20_modio]: optoStates = 0x%X", optoStates);
	printk(KERN_ALERT "[a20_modio]: optoStates = 0x%X & 0x%X", optoStates, opto_id);
	
	return((optoStates & opto_id) ? ON : OFF);

}


/************************************/
/*     GET ANALOG INPUT             */
/************************************/
uint16_t getAnalog(analog_id_t id) {
    
    analog_value = 0x0;
    printk(KERN_ALERT "[a20_modio]: Reading Analog value: \n");
	if (curr_minor == ANINPUT1_MINOR)
	{
		analogReadCommand = 0x30;
	} else if (curr_minor == ANINPUT2_MINOR)
	{
		analogReadCommand = 0x31;
	} else if (curr_minor == ANINPUT3_MINOR)
	{
		analogReadCommand = 0x32;
	} else if (curr_minor == ANINPUT4_MINOR)
	{
		analogReadCommand = 0x33;
	}
	// Etape 1
	i2c_start();
	i2c_duty();
	i2c_send((param_slave_addr << 1) | WRITE_MODE);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return ERR;
	}
	i2c_duty();
	
	i2c_send(analogReadCommand);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return ERR;
	}
	i2c_duty();
	i2c_stop();
	i2c_duty();
	i2c_duty();
	i2c_start();
	i2c_duty();
	i2c_send((param_slave_addr << 1) | READ_MODE);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		return ERR;
	}
	i2c_duty();
	
	l_byte = i2c_read();
	i2c_duty();
	i2c_send_ack();
	i2c_duty();
	h_byte = i2c_read();
	i2c_duty();
	i2c_send_ack();
	i2c_duty();
	i2c_stop();
	i2c_duty();
	
	// Construct the analog value from h_byte and l_byte
	for (index = 0 ; index <= 7; index++)
	{
		analog_value |= (l_byte & (1 << index)) ? 1 : 0;
		analog_value <<= 1;
	}
	//Last 2 bits
	analog_value |= ((h_byte & (1 << 1))? 1 : 0) << 8;
	analog_value |= ((h_byte & (1 << 0))? 1 : 0) << 9;
	

	printk(KERN_ALERT "[a20_modio]: Low_Byte:  0x%X\n", l_byte);
	printk(KERN_ALERT "[a20_modio]: High_Byte: 0x%X\n", h_byte);

    printk(KERN_ALERT "[a20_modio]: Analog: 0x%X\n", analog_value);
    
    return analog_value;	
}

/************************************/
/*     SET ADDRESS                  */
/************************************/

int set_address(uint8_t addr)
{
    //start
	//send 0xb0
	//send 0xF0
	//send 0x22
	//close
	printk(KERN_ALERT "[a20_modio]: Try changing default address to 0x%X\n", addr);
    i2c_start();
	i2c_duty();
	i2c_send((MOD_IO_ADDRESS_DEFAULT << 1) | WRITE_MODE);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		printk(KERN_ALERT "[a20_modio]: Unable to change default address to 0x%X\n", addr);
		return ERR;
	}
	i2c_duty();
	i2c_send(SET_ADDR);
	i2c_duty();
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		printk(KERN_ALERT "[a20_modio]: Unable to change default address to 0x%X\n", addr);
		return ERR;
	}
	i2c_duty();
	i2c_send(addr);
	if(i2c_wait_ack() == NOACK)
	{
		i2c_stop();
		printk(KERN_ALERT "[a20_modio]: Unable to change default address to 0x%X\n", addr);
		return ERR;
	}
	i2c_duty();
	i2c_stop();
}

/*********************************/
/* INIT MAPPING                  */
/*********************************/
ret_t init_ioremap()
{
    // Map GPIO addressing space
    printk(KERN_ALERT "[a20_modio]: Mapping 0x%X - 0x%X (size: 0x%X)\n", 
                                    _4K_DIV_BASE_ , 
                                    _4K_DIV_BASE_ + PIO_SIZE, 
                                    PIO_SIZE);
    // Map to a20 registers
    addr = ioremap(_4K_DIV_BASE_, PIO_SIZE);

    // Check map returned address
    if(addr == NULL) {
        printk(KERN_ALERT "[a20_modio]: Unable to map GPIO: %s\n", "-1");
        return (ERR);
    }
    
    // Casting to char to get the correct offset calcul
    addr  = (char *) addr + PIO_BASE_ADDR;
    cfg   = (char *) addr + PBCFG2;
    data  = (char *) addr + PBDATA_REG;
       
    printk(KERN_ALERT "[a20_modio]: *PB_B. %x \n",addr);
    printk(KERN_ALERT "[a20_modio]: *PB_CFG. %x \n",cfg);
    printk(KERN_ALERT "[a20_modio]: *PB_DATA %x\n",data);
    printk(KERN_ALERT "[a20_modio]: *PB_DATA and PB_CFG initialised. \n");
	
    printk(KERN_ALERT "[a20_modio]: PB_CFG. %x \n", ioread32(cfg));
    printk(KERN_ALERT "[a20_modio]: PB_DATA %x \n", ioread32(data));

    return OK;
}

///Reset addresses
ret_t reset_p()
{

    addr            = NULL;
    cfg             = NULL;
    data            = NULL;
    modio_cdev      = NULL;
    modio_class     = NULL;
    delay           = 0;
    output_old_states = 0x0000;
    
}
/******************************************************************/
/******************************************************************/
/******************************************************************/
/*                                  DRIVER IMPLEMENTATION         */
/******************************************************************/
/* This part contain only driver Functions                        */      
/******************************************************************/
/******************************************************************/
/****************************************************/
/* Declare Driver Function Prototypes               */
/****************************************************/

static int a20_open(struct inode *in, struct file *f);
static int a20_release(struct inode *in, struct file *f);
static ssize_t a20_read(struct file* filp, char* outputBuffer, size_t sizeToRead, loff_t* curOffset);
static ssize_t a20_write(struct file* filp, const char* inputData, size_t sizeToWrite, loff_t* curOffset);
static int a20_ioctl (struct file *filp, unsigned int cmd, unsigned long arg);

/************************************/
/* Declare FileOperations Structure */
/************************************/

// standard file_ops for char driver 
struct file_operations fops = 
{

    .owner          = THIS_MODULE,  
    .open           = a20_open,
    .release        = a20_release,
    .read           = a20_read,
    .write          = a20_write,
    .unlocked_ioctl = a20_ioctl

};

/****************************************************/
/* IMPLEMENTATION OF FUNCTIONS                      */
/****************************************************/

/****************************************************/
/* OPEN                                             */
/****************************************************/
static int a20_open(struct inode *in, struct file *f) {

    curr_minor = iminor(in) - MINOR(modio_dev);
    printk(KERN_ALERT "[a20_modio]: Device [Minor:%d]Opened.", curr_minor);
    lock_cat = 1;
    
    return OK;
}


/****************************************************/
/* RELEASE                                             */
/****************************************************/
static int a20_release(struct inode *in, struct file *f) {
    
    printk(KERN_ALERT "[a20_modio]: Device Closed.");
    
    return OK;
}




/****************************************************/
/* READ                                             */
/****************************************************/
static ssize_t a20_read(struct file* filp, char* out_buffer, size_t size, loff_t* offset) {
    
    
    if (lock_cat == 0) 
        return 0;
        
    // Disable disable access
    lock_cat = 0;
    
    printk(KERN_ALERT "[a20_modio]: Reading from device.", size);
    copy_ret = 0;
    if ((curr_minor >= ANINPUT1_MINOR) && (curr_minor <= ANINPUT4_MINOR)) {
        // Read Analog Value
        // In this case of analog devices, ANINPUTx_ID and ANINPUTx_MINOR are same.
        // We can directly use curr_minor to address the correct AnalogDevice throught getAnalog
        // Function.
        analog_value = getAnalog(curr_minor);
        printk(KERN_ALERT "[a20_modio]: Analog_int: 0x%X", analog_value);
        printk(KERN_ALERT "[a20_modio]: Analog_int: %d", analog_value);
           for (index = 9; index >= 0; index--)
	       {
		       if (analog_value & (1 << index) )
		        analog_strvalue[9 - index] = '1';
		       else
		        analog_strvalue[9 - index] = '0';  
	       }
	       analog_strvalue[10] = '\0';
	       
	       printk(KERN_ALERT "[a20_modio]: Analog_str: %s", analog_strvalue);
           copy_ret = copy_to_user(out_buffer, analog_strvalue, 11);
           
           return 11; 
    } 
    
    switch(curr_minor)
    {
        // OPTOCOUPLEURS
        case OPTO1_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Opto 1");
            opto_str[0] = (getOptoStates(OPTO1_ID) == ON) ? '1' : '0';
            opto_str[1] = '\0';
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN VALUE : %s", opto_str);
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN 1 : %c", opto_str[0]);
            copy_ret = copy_to_user(out_buffer, opto_str, 2);
            return 2;
            
        case OPTO2_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Opto 2");
            opto_str[0] = (getOptoStates(OPTO2_ID) == ON) ? '1' : '0';
            opto_str[1] = '\0';
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN VALUE : %s", opto_str);
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN 1 : %c", opto_str[0]);
            copy_ret = copy_to_user(out_buffer, opto_str, 2);
            return 2;
            
        case OPTO3_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Opto 3");
            opto_str[0] = (getOptoStates(OPTO3_ID) == ON) ? '1' : '0';
            opto_str[1] = '\0';
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN VALUE : %s", opto_str);
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN 1 : %c", opto_str[0]);
            copy_ret = copy_to_user(out_buffer, opto_str, 2);
            return 2;
            
        case OPTO4_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Opto 4");
            opto_str[0] = (getOptoStates(OPTO4_ID) == ON) ? '1' : '0';
            opto_str[1] = '\0';
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN VALUE : %s", opto_str);
            printk(KERN_ALERT "[a20_modio]: \t \tRETURN 1 : %c", opto_str[0]);
            copy_ret = copy_to_user(out_buffer, opto_str, 2);
            return 2;
        
        // OUTPUTS
        case OUTPUT1_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Relay 1 ", size);
            output_str[0] = (output1 == ON) ? '1' : '0'; 
            output_str[1] = '\0';
            copy_ret = copy_to_user(out_buffer, output_str, 2);
            return 2;
            
        case OUTPUT2_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Relay 2");
            output_str[0] = (output2 == ON) ? '1' : '0'; 
            output_str[1] = '\0';
            copy_ret = copy_to_user(out_buffer, output_str, 2);
            return 2;
            
        case OUTPUT3_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Relay 3");
            output_str[0] = (output3 == ON) ? '1' : '0'; 
            output_str[1] = '\0';
            copy_ret = copy_to_user(out_buffer, output_str, 2);
            return 2;
            
        case OUTPUT4_MINOR:
        
            printk(KERN_ALERT "[a20_modio]: Relay 4");
            output_str[0] = (output4 == ON) ? '1' : '0'; 
            output_str[1] = '\0';
            copy_ret = copy_to_user(out_buffer, output_str, 2);
            return 2;
            
        default:
        
            printk(KERN_ALERT "[a20_modio]: This device [Minor:%d], is not allowed to read.", curr_minor);

    }
    return copy_ret;
}

/****************************************************/
/* WRITE                                             */
/****************************************************/
static ssize_t a20_write(struct file* filp, const char* input_data, size_t size, loff_t* offset) {
    printk(KERN_ALERT "[a20_modio]: Writing to a device.");
    
    if ( (input_data[0] != '1') && (input_data[0] != '0') )
    {
        printk(KERN_ALERT "[a20_modio]: Received incorrect data, need only ('0' or '1').");
        return ERR;
    } 
    // Get bit state needed
    relay_flag = (input_data[0] == '0') ? ON : OFF;
    
    switch(curr_minor)
    {
        case OUTPUT1_MINOR:
            printk(KERN_ALERT "[a20_modio]: Relay 1");
            setOutput(OUTPUT1_ID, relay_flag);
            break;
        case OUTPUT2_MINOR:
            printk(KERN_ALERT "[a20_modio]: Relay 2");
            setOutput(OUTPUT2_ID, relay_flag);
            break;
        case OUTPUT3_MINOR:
            printk(KERN_ALERT "[a20_modio]: Relay 3");
            setOutput(OUTPUT3_ID, relay_flag);
            break;
        case OUTPUT4_MINOR:
            printk(KERN_ALERT "[a20_modio]: Relay 4");
            setOutput(OUTPUT4_ID, relay_flag);
            break;
        default:
            printk(KERN_ALERT "[a20_modio]: This device [Minor:%d], is not allowed to write.", curr_minor);
    }
    
    return size;
}

/****************************************************/
/* IOCTL                                             */
/****************************************************/
static int a20_ioctl (struct file *filp, unsigned int cmd, unsigned long arg) {
	
    printk(KERN_ALERT "[a20_modio]: Accessing IOCTL.");
    
    return OK;
}


/**************************************************************************************/
/* INIT                                                                               */
/**************************************************************************************/
static int init_a20_module(void) {
    
    printk(KERN_ALERT "[a20_modio]: Inserting Module.");
    reset_p();
    init_ioremap();
    
    printk(KERN_ALERT "[a20_modio]: Scanning addresses ...\n");
    
    // Twice to be sure
    if ((i2c_ping(param_slave_addr) == NOACK) && (i2c_ping(param_slave_addr) == NOACK)) {
        printk(KERN_ALERT "[a20_modio]: Slave Addr : 0x%X => Busy", param_slave_addr);
        printk(KERN_ALERT "[a20_modio]: Changing default address to 0x%X", param_slave_addr);
        set_address(param_slave_addr);
        printk(KERN_ALERT "[a20_modio]: Try using 0x%X address after 5 seconds ...");
        printk(KERN_ALERT "[a20_modio]: If it doesent work use the default address: 0x58");
    }
    
    // Get Major number
    if (alloc_chrdev_region(&modio_dev, FIRST_MINOR, NB_MODIO_DEVICES, "modio_drv") < 0)
    {
        printk(KERN_ALERT "[a20_modio]: Error when getting major number. ");
        return ERR;
    }
    
    printk(KERN_ALERT "[a20_modio]: Init allocated (major, minor)=(%d, %d) !\n", MAJOR(modio_dev), MINOR(modio_dev));

    //cl = kmalloc( sizeof(cl), GFP_KERNEL);

    /***************************************/
    /*  // Udev Creating Devices           */
    /***************************************/
    modio_cdev = cdev_alloc();
    modio_cdev->ops = &fops;
    modio_cdev->owner = THIS_MODULE;

    if (cdev_add(modio_cdev, modio_dev, NB_MODIO_DEVICES) < -1)
    {
        class_destroy(modio_class);
        device_destroy(modio_class, modio_dev);
        unregister_chrdev_region(modio_dev, NB_MODIO_DEVICES);
        printk(KERN_ALERT "[a20_modio]: Failed to add device to kernel.");
        return ERR;
    }
    //Create class
    if ((modio_class = class_create(THIS_MODULE, MODIO_CLASS)) == NULL)
    {
        class_destroy(modio_class);
        unregister_chrdev_region(modio_dev, NB_MODIO_DEVICES);
        printk(KERN_ALERT "[a20_modio]: Failed to create class '%s'", MODIO_CLASS);
        return ERR;
    }
    // Create each device with its minor, 
    for(index = FIRST_MINOR; index < NB_MODIO_DEVICES; index++)
    {
         printk(KERN_ALERT "[a20_modio]: Creating device: /dev/%s ", device_names[index]);
        if (device_create(modio_class, NULL, MKDEV( MAJOR(modio_dev), MINOR(modio_dev) + index ), NULL, device_names[index]) == NULL)
        {
            class_destroy(modio_class);
            unregister_chrdev_region(modio_dev, NB_MODIO_DEVICES);
            printk(KERN_ALERT "[a20_modio]: Unable to create device: %s", device_names[index]);
            return ERR;
        }
        printk(KERN_ALERT "/dev/%s . OK", device_names[index]);
    }
    printk(KERN_ALERT "[a20_modio]: Init Done.");
    return OK;
}


/****************************************************/
/* CLEANUP                                          */
/****************************************************/
static int cleanup_a20_module(void) {
    printk(KERN_ALERT "[a20_modio]: Removing Module.");
    
    for(index = FIRST_MINOR; index < NB_MODIO_DEVICES; index++)
    {
        device_destroy( modio_class, MKDEV( MAJOR(modio_dev), MINOR(modio_dev) + index) );
        printk(KERN_ALERT "[a20_modio]: /dev/%s is Removed .", device_names[index] );
    }
    // Delete cdev 
    unregister_chrdev_region(modio_dev, NB_MODIO_DEVICES);
    cdev_del(modio_cdev);
    printk(KERN_ALERT "[a20_modio]: Destroyng class ...");
    class_destroy ( modio_class );
    printk(KERN_ALERT "[a20_modio]: Remove Mapping ...");
    iounmap(addr);
    printk(KERN_ALERT "[a20_modio]: Reset Addresses ...");
    reset_p();
    printk(KERN_ALERT "[a20_modio]: Cleanup done.");
    return OK;
}

module_init(init_a20_module);
module_exit(cleanup_a20_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("RACHED MAIRACHI, AHMED SABBANI , SALIM KOUMAD");
MODULE_DESCRIPTION("mod_io _ a20 driver");
MODULE_SUPPORTED_DEVICE("Memory driver");











