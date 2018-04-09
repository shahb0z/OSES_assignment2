/*
 * sample.c - The simplest loadable kernel module.
 * Intended as a template for development of more
 * meaningful kernel modules.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/uaccess.h>

/*
 * Definition of the GPIOs we want to use
*/
#define	ECHO	130		// gpio d8, input
#define TRIG	131		// gpio d7, output
#define LED	    129		// user led, output

/*
 * Device major number
 */
static uint module_major = 166;

/*
 * Device name
 */
static char * module_name = "dist_me";

/*
 * Device access lock. Only one process can access the driver at a time
 */
static int dist_me_lock = 0;

/*
 * Device variables
*/
static volatile char measure = 0; // measure flag

/*
 * start and end time of the sensor
 */

static ktime_t echo_start;
static ktime_t echo_end;

/*
 * Declare the workqueue
 */
 
static struct workqueue_struct *my_wq;

static struct work_struct work;
static char blinking = 0;
static int blink_rate = 0;
static int kill_work = 0;
/*
 * Work function
 */
static void my_wq_function( struct work_struct *work )
{
	//int blink_rate;
	//my_work_t *my_work;

	//my_work = (my_work_t *)work;
	//blink_rate = my_work->blink_rate;
	gpio_set_value( LED, 0 );
	while(1){
		if(kill_work)
			break;
		gpio_set_value( LED, 1 );
		msleep( blink_rate );
		gpio_set_value( LED, 0 );
		msleep(blink_rate );
	}
}

/*
 * Button interrupt handler
*/

static irq_handler_t echo_handler( unsigned int irq, struct pt_regs *regs )
{
	ktime_t ktime_dummy;

	if (measure==0) {
		ktime_dummy=ktime_get();
		if (gpio_get_value(ECHO)==1) {
			echo_start=ktime_dummy;
		} else {
			echo_end=ktime_dummy;
			measure=1;
		}
	}

	return (irq_handler_t)IRQ_HANDLED;
}

/*
 * Device open
 */
static int dist_me_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	/*
	 * One process at a time
	 */
	if (dist_me_lock > 0) 
	{
		ret = -EBUSY;
	}
	else
	{
		dist_me_lock++;

		/*
 	 	* Increment the module use counter
 	 	*/
		try_module_get(THIS_MODULE);

		#ifdef SAMPLE_DEBUG 
			printk( KERN_INFO "%s: %s\n", module_name, __func__ ); 
		#endif
		
	}

	return( ret );
}

/*
 * Device close
 */
static int dist_me_release(struct inode *inode, struct file *file)
{
	/*
 	 * Release device
 	 */
	dist_me_lock = 0;

	/*
 	 * Decrement module use counter
 	 */
	module_put(THIS_MODULE);

	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif

	return( 0 );
}

/* 
 * Device read
 */
static ssize_t dist_me_read(struct file *filp, char *buffer,
			 size_t length, loff_t * offset)
{
	int counter,len;

	// Send a 10uS impulse to the TRIGGER line
	gpio_set_value(TRIG,1);
	udelay(10);
	gpio_set_value(TRIG,0);
	measure=0;
	
	
	counter=0;
	while (measure==0) {
		// Out of range
		if (++counter>23200) {
			//return error value
			return -1;
		}
		udelay(1);
	}
	
	//returning measured time
	len = sprintf(buffer, "%lld\n", ktime_to_us(ktime_sub(echo_end,echo_start)));
	return len;
}

/* 
 * Device write
 */
static ssize_t dist_me_write(struct file *filp, const char *buffer,
			  size_t length, loff_t * offset)
{
	int rate;
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	memcpy( &rate, buffer, sizeof( rate) );
	if (!blinking){
		blinking = 1;
		queue_work( my_wq, (struct work_struct *)&work );
	}
	blink_rate=rate;
	return length;
}

static ssize_t dist_me_ioctl(struct inode *inode, struct file *filep, 
			    const unsigned int cmd, const unsigned long arg)
{
	int ret = 0;
	#ifdef SAMPLE_DEBUG
		printk( KERN_INFO "%s: %s\n", module_name, __func__ );
	#endif
	return( ret );
}


/*
 * Device operations
 */
static struct file_operations dist_me_fops = {
	.read = dist_me_read,
	.write = dist_me_write,
	.open = dist_me_open,
	.release = dist_me_release,
	.ioctl = dist_me_ioctl
};

static int __init dist_me_init_module(void)
{
	/*
 	 * Register device
 	 */
	int	ret;

	ret = register_chrdev(module_major, module_name, &dist_me_fops);
	if (ret < 0) {
		printk(KERN_INFO "%s: registering device %s with major %d failed with %d\n",
		       __func__, module_name, module_major, module_major );
		return( ret );
	}
	else
	{
		printk(KERN_INFO "%s: registering device %s with major %d\n",
		       __func__, module_name, module_major );

		/*
 		 * Reserve gpios ECHO (as input) and TRIG (as output, with default output value set to 0)
		*/
		if( gpio_request( ECHO, module_name ) )	// Check if ECHO is available
		{
			printk( KERN_INFO "%s: %s unable to get ECHO gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_request( TRIG, module_name ) )	// Check if TRIG is available
		{
			printk( KERN_INFO "%s: %s unable to get TRIG gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		if( gpio_request( LED, module_name ) )	// Check if LED is available
		{
			printk( KERN_INFO "%s: %s unable to get LED gpio\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		if( gpio_direction_input( ECHO ) < 0 )	// Set ECHO gpio as input
		{
			printk( KERN_INFO "%s: %s unable to set ECHO gpio as input\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}

		if( gpio_direction_output( TRIG, 0 ) < 0 )	// Set TRIG gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set TRIG gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		if( gpio_direction_output( LED, 0 ) < 0 )	// Set LED gpio as output with default value 0
		{
			printk( KERN_INFO "%s: %s unable to set LED gpio as output\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		if( request_irq( gpio_to_irq( ECHO ), 
                                 (irq_handler_t) echo_handler,
				  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING ,
				  module_name,
				  NULL ) < 0 )
		{
			printk( KERN_INFO "%s: %s unable to register gpio irq for ECHO\n", module_name, __func__ );
			ret = -EBUSY;
			return( ret );
		}
		
		my_wq = create_workqueue( "my_queue" );
		if( my_wq )
		{
			INIT_WORK( (struct work_struct *)&work, my_wq_function );
		}
	}
	
	return( ret );
}

static void __exit dist_me_cleanup_module(void)
{
	/*
	 * Free irq
	 */
	free_irq( gpio_to_irq( ECHO ), NULL );

	/*
	 * Release the gpios
	 */
	gpio_free( ECHO );
	gpio_free( TRIG );
	gpio_free( LED );
	printk(KERN_INFO "Destroying queue!\n");
	kill_work = 1;
	flush_workqueue( my_wq );
	destroy_workqueue(my_wq);
	printk(KERN_INFO "Queue Destroyed!\n");
	/*
	 * Unregister device
	 */
	 
	unregister_chrdev(module_major, module_name);
	printk(KERN_INFO "%s: unregistering %s done\n", __func__, module_name );
}

module_init(dist_me_init_module);
module_exit(dist_me_cleanup_module);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Shahbozbek Abdunabiev, shahbozbek.abunabiyev@studenti.poliot.it");
MODULE_DESCRIPTION("Driver for HC-SR04 proximity sensor");

