#define USE_STDPERIPH_DRIVER
#include "stm32f10x.h"
#include <stdarg.h>


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
//#include <string.h>

#include "filesystem.h"
#include "fio.h"

static void setup_hardware();
void send_byte(char );




volatile xQueueHandle serial_str_queue = NULL;
volatile xSemaphoreHandle serial_tx_wait_sem = NULL;
volatile xQueueHandle serial_rx_queue = NULL;

int strcat(char *a, char *b) __attribute__ ((naked));

int strcat(char *a,char *b){
	int i=0,j=0;
	while(a[i++]);//no error handler,be careful

	do{
		a[(i++)-1]=b[j];
	}while(b[j++]);
	return j;
}
int strcmp(const char *a, const char *b) __attribute__ ((naked));
int strcmp(const char *a, const char *b)
{
	asm(
        "strcmp_lop:                \n"
        "   ldrb    r2, [r0],#1     \n"
        "   ldrb    r3, [r1],#1     \n"
        "   cmp     r2, #1          \n"
        "   it      hi              \n"
        "   cmphi   r2, r3          \n"
        "   beq     strcmp_lop      \n"
		"	sub     r0, r2, r3  	\n"
        "   bx      lr              \n"
		:::
	);
}
//#define puts(var) fio_write(1, #var, strlen(#var));
size_t strlen(const char *s) __attribute__ ((naked));
size_t strlen(const char *s)
{
	asm(
		"	sub  r3, r0, #1			\n"
        "strlen_loop:               \n"
		"	ldrb r2, [r3, #1]!		\n"
		"	cmp  r2, #0				\n"
        "   bne  strlen_loop        \n"
		"	sub  r0, r3, r0			\n"
		"	bx   lr					\n"
		:::
	);
}
int str2int(char* str){
	int temp=0,idx=0;
	while(str[idx]){
		temp*=10;
		temp+=(str[idx++]-0x30);
	}
	return temp;
}

void puts(char *s)
{
	while (*s) {
		while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
		USART_SendData(USART2, *s);
		s++;
	}
}

void int2str(int x,char* c_arr){
	int digit[5];
	int temp=10000,idx,i;
	
	for(i=0;i<5;i++){
		digit[i]=x/temp;
		x=x%temp;
		temp/=10;
	}
	
	for(i=0;i<5;i++){
		if(digit[i]!=0){
			for(idx=0;idx<(5-i);idx++)
				c_arr[idx]=0x30+digit[i+idx];
			c_arr[idx]='\0';
			return ;
		}
		
	}
	//for x==zero
	c_arr[0]=0x30;
	c_arr[1]=0x00;
	return ;
}

//only support %d
void printf(char* str,...){
	va_list marker;
	char buf[10];
	int i;
	va_start(marker, str);
	for(i=0;str[i]!='\0';i++){
		if(str[i]!='%'){
			send_byte(str[i]);
		}
		else{
			if(str[++i]=='d'){
				int2str( va_arg(marker, int),buf );
				puts(buf);
			}
		}
	}
	va_end(marker);
}

/* Queue structure used for passing messages. */
typedef struct {
	char str[100];
} serial_str_msg;

/* Queue structure used for passing characters. */
typedef struct {
	char ch;
} serial_ch_msg;

/* IRQ handler to handle USART2 interruptss (both transmit and receive
 * interrupts). */
void USART2_IRQHandler()
{
	static signed portBASE_TYPE xHigherPriorityTaskWoken;
	serial_ch_msg rx_msg;

	/* If this interrupt is for a transmit... */
	if (USART_GetITStatus(USART2, USART_IT_TXE) != RESET) {
		/* "give" the serial_tx_wait_sem semaphore to notfiy processes
		 * that the buffer has a spot free for the next byte.
		 */
		xSemaphoreGiveFromISR(serial_tx_wait_sem, &xHigherPriorityTaskWoken);

		/* Diables the transmit interrupt. */
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
		/* If this interrupt is for a receive... */
	}
	else if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
		/* Receive the byte from the buffer. */
		rx_msg.ch = USART_ReceiveData(USART2);

		/* Queue the received byte. */
		if(!xQueueSendToBackFromISR(serial_rx_queue, &rx_msg, &xHigherPriorityTaskWoken)) {
			/* If there was an error queueing the received byte,
			 * freeze. */
			while(1);
		}
	}
	else {
		/* Only transmit and receive interrupts should be enabled.
		 * If this is another type of interrupt, freeze.
		 */
		while(1);
	}

	if (xHigherPriorityTaskWoken) {
		taskYIELD();
	}
}

void send_byte(char ch)
{
	/* Wait until the RS232 port can receive another byte (this semaphore
	 * is "given" by the RS232 port interrupt when the buffer has room for
	 * another byte.
	 */
	while (!xSemaphoreTake(serial_tx_wait_sem, portMAX_DELAY));

	/* Send the byte and enable the transmit interrupt (it is disabled by
	 * the interrupt).
	 */
	USART_SendData(USART2, ch);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
}
int semihost_call(unsigned char sys_code, void *arg) __attribute__ ((naked));
int semihost_call(unsigned char sys_code, void *arg)
{
    __asm__( \
      "bkpt 0xAB\n"\
      "nop\n" \
      "bx lr\n"\
        :::\
    );
}
int host_sys_cmd(const char *cmd){
	unsigned int arg[2] ={cmd,strlen(cmd) };
	return semihost_call(0x12, arg);
}

char receive_byte()
{
	serial_ch_msg msg;

	/* Wait for a byte to be queued by the receive interrupts handler. */
	while (!xQueueReceive(serial_rx_queue, &msg, portMAX_DELAY));

	return msg.ch;
}

void led_flash_task(void *pvParameters)
{
	while (1) {
		/* Toggle the LED. */
		GPIOC->ODR = GPIOC->ODR ^ 0x00001000;

		/* Wait one second. */
		vTaskDelay(100);
	}
}

void rs232_xmit_msg_task(void *pvParameters)
{
	serial_str_msg msg;
	int curr_char;

	while (1) {
		/* Read from the queue.  Keep trying until a message is
		 * received.  This will block for a period of time (specified
		 * by portMAX_DELAY). */
		while (!xQueueReceive(serial_str_queue, &msg, portMAX_DELAY));

		/* Write each character of the message to the RS232 port. */
		curr_char = 0;
		while (msg.str[curr_char] != '\0') {
			send_byte(msg.str[curr_char]);
			curr_char++;
		}
	}
}





#define KEY_ENTER 0x0D
#define KEY_BACKSPACE 0x7F
void catch_msg(serial_str_msg* msg)
{
	
	char ch;
	int curr_char;
	int done;

	
		curr_char = 0;
		done = 0;
		do {
			/* Receive a byte from the RS232 port (this call will
			 * block). */
			ch = receive_byte();
			
			
			/* If the byte is an end-of-line type character, then
			 * finish the string and inidcate we are done.
			 */
			if ( (curr_char>=sizeof(msg->str)-1 ) || (ch == '\r') || (ch == '\n')) {
				done = -1;
				/* Otherwise, add the character to the
				 * response string. */
				puts("\r\n");
				msg->str[curr_char]=0x00;

			}
			else {
				if(ch==KEY_BACKSPACE){
					if(curr_char){
						puts("\b \b");
						msg->str[curr_char--]=0x00;
					}
				}
				else{
					msg->str[curr_char++] = ch;
					send_byte(ch);
				}
			}
			
		} while (!done);

		/* Once we are done building the response string, queue the
		 * response to be sent to the RS232 port.
		 */
	//	while (!xQueueSendToBack(serial_str_queue, &msg,
		//                         portMAX_DELAY));
	
}
extern xList pxReadyTasksLists[ configMAX_PRIORITIES ];
#define configUSE_TRACE_FACILITY 1
void ps_cmd_function(){
	char str[500];
	int i,j,count;
	struct xList *pxTemp;
	puts("\rtask_name\tpriority\tStack_Remaining\tTCB_Num\n\r");
	for(i=0;i<configMAX_PRIORITIES;i++){
		pxTemp=&pxReadyTasksLists[i];
		count=pxReadyTasksLists[i].uxNumberOfItems;
		if(count){
			str[0]='\0';
			prvListTaskWithinSingleList(str,pxTemp,'?');
			puts(str);
		}
	}
}
#define MAX_DUMMY_TASK_NUM 8

static xTaskHandle xHandle_Dummy[MAX_DUMMY_TASK_NUM];


void kill(char* filename){
	int i;
	for(i=0;i<MAX_DUMMY_TASK_NUM;i++){
		if(strcmp( pcTaskGetTaskName(xHandle_Dummy[i]),filename )==0 ){
			vTaskDelete(xHandle_Dummy[i]);
			xHandle_Dummy[i]=NULL;
			return ;
		}
	}
}

void my_dummy_task(){
	while(1);
}
void read_romfs(char *s)
{
	char buf[128]="/romfs/";
	size_t count;
	int fd;
	strcat(buf,s);
	fd= fs_open(buf, 0, O_RDONLY);
	do {
		//Read from /romfs/test.txt to buffer
		count = fio_read(fd, buf, sizeof(buf));

		//Write buffer to fd 1 (stdout, through uart)
		fio_write(1, buf, count);
	} while (count);

	
}

static unsigned int lfsr=0xACE1;



// Get a pseudorandom number generator from Wikipedia
 int prng(unsigned int* ) __attribute__ ((naked));
 int prng(unsigned int* ptr_lfsr )
{
	asm(
      	"   push    {r1,r2,r3,r4}         \n"
      	
        "   ldrh    r2, [r0]         \n"
        "   lsr     r3, r2,#2     \n"
        "   EOR     r4, r3,r2       \n"
        "   lsr     r3, r2,#3     \n"
        "   EOR     r4, r4,r3       \n"
        "   lsr     r3, r2,#5     \n"
        "   EOR     r4, r4,r3       \n"
        "   AND     r4, r4,#1       \n"
        "   lsr     r2, r2,#1       \n"
        "   lsl     r4, r4,#15      \n"
        "   ORR     r2, r2,r4       \n"
        "   strh    r2,[r0]       \n"
        "   movt    r1, #0  \n"
        "   movw   r1, #65535 \n"
        "   AND     r0,r1,r2  \n"
        "   pop    {r1,r2,r3,r4}         \n"
        "   bx      lr              \n"
		:::
	);
	//	  "   mov	  r0,#2  \n"

   // static unsigned int bit;
    /* taps: 16 14 13 11; characteristic polynomial: x^16 + x^14 + x^13 + x^11 + 1 */
  // bit  = ((*ptr_lfsr >> 0) ^ (*ptr_lfsr >> 2) ^ (*ptr_lfsr >> 3) ^ (*ptr_lfsr >> 5) ) & 1;
  // *ptr_lfsr =  (*ptr_lfsr >> 1) | (bit << 15);
  //  return *ptr_lfsr & 0xffff;
}
char* ptr_mem;



struct slot {
    void *pointer;
    unsigned int size;
    unsigned int lfsr;
};

#define CIRCBUFSIZE 500
unsigned int write_pointer, read_pointer;
static struct slot slots[CIRCBUFSIZE];


static unsigned int circbuf_size(void)
{
    return (write_pointer + CIRCBUFSIZE - read_pointer) % CIRCBUFSIZE;
}

static void write_cb(struct slot foo)
{
    if (circbuf_size() == CIRCBUFSIZE - 1) {
		return ;
    }
    slots[write_pointer++] = foo;
    write_pointer %= CIRCBUFSIZE;
}

static struct slot read_cb(void)
{
    struct slot foo;
    if (write_pointer == read_pointer) {
        // circular buffer is empty
        return (struct slot){ .pointer=NULL, .size=0, .lfsr=0 };
    }
    foo = slots[read_pointer++];
    read_pointer %= CIRCBUFSIZE;
    return foo;
}

void enable_prng_test(){
	printf("%d ",lfsr);
	prng(&lfsr);
	
}
void enable_memory_test(){
	int size,i;
    char *p;
	struct slot foo;
		while (1) {
		
			size = prng(&lfsr) & 0x7FF;
		  	
			p = (char *) pvPortMalloc(size);
			printf("try to allocate %d bytes\n\r",size);
			if (p == NULL) {
				// can't do new allocations until we free some older ones
				while (circbuf_size() > 0) {
					// confirm that data didn't get trampled before freeing
					foo = read_cb();
					p = foo.pointer;
					lfsr = foo.lfsr;  // reset the PRNG to its earlier state
					size = foo.size;
					printf("free a block, size %d\n\r", size);
					for (i = 0; i < size; i++) {
						unsigned char u = p[i];
					
						unsigned char v =  prng(&lfsr);
						if (u != v) {
							printf("ouch:%d %d %d\n\r", u,v,i);
							return 1;
						}
					}
					vPortFree(p);
					if ((prng(&lfsr)  & 1) == 0){
						return;
					}
				}
			} else {
				
				write_cb((struct slot){.pointer=p, .size=size, .lfsr=lfsr});
				for (i = 0; i < size; i++) {
					p[i] = prng(&lfsr);
				}
				printf("success to allocate %d bytes\n\r",size);
			}
		}

}

void my_shell_task(){
	static serial_str_msg msg;
	char pid[6];
	int i,idx=0;
	char *pTemp_c;
	
	//init dummy's handle
	for(i=0;i<MAX_DUMMY_TASK_NUM;i++)
		xHandle_Dummy[i]=NULL;

		//printf("hello\n\r");
	while(1){
		puts("enter command:");
		catch_msg(&msg);
		if( strcmp(msg.str,"echo") ==0){
			puts("echo function:");
			catch_msg(&msg);					
			puts(msg.str);
			puts("\n\r");		
		}
		else if( strcmp(msg.str,"help") ==0){
			puts("command\tdetail\n\r");
			puts("help\tinformation about all commands\n\r");
			puts("echo\ttype a string,the shell will echo\n\r");
			puts("hello\tshow the greetings\n\r");
			puts("ps\tshow the PID and Priority of all undergoing tasks\n\r");
			puts("add\tadd a dummy task \n\r");
			puts("host cmd\tcommand on host bash \n\r");
		}
		else if( strcmp(msg.str,"hello") ==0){
			puts("HelloWorld!\n\r");
		}
		else if( strcmp(msg.str,"ps") ==0){
			ps_cmd_function();
			puts("\n\r");
		}
		else if( strcmp(msg.str,"add") ==0){
			puts("enter name of dummy task:");
			catch_msg(&msg);
			for(i=0;i<MAX_DUMMY_TASK_NUM;i++)
				if(xHandle_Dummy[i]==NULL){
					xTaskCreate(my_dummy_task,(signed portCHAR *) msg.str,	512 , NULL, 4, &xHandle_Dummy[i]);
					break;
				}
		}
		else if( strcmp(msg.str,"kill") ==0){
			puts("enter dummy's name:");
			catch_msg(&msg);
			kill(msg.str);
		}
		else if( strcmp(msg.str,"cat ") >0 && strcmp(msg.str,"cat z")<0 ){
			pTemp_c=&(msg.str[4]);
			read_romfs(pTemp_c);
			puts("\n\r");
		}
		else if( strcmp(msg.str,"host ") >0 && strcmp(msg.str,"host z")<0){
			pTemp_c=&(msg.str[5]);
			host_sys_cmd(pTemp_c);
		}
		else if( strcmp(msg.str,"mmtest") ==0){
			enable_memory_test();
		}
		else if( strcmp(msg.str,"prng") ==0){
			enable_prng_test();
		}
		else{
			puts(msg.str);
			puts(" is invalid command\n\r");
		}
		
	}
}


extern const char _sromfs;


int main()
{
	int i;
	init_led();
	

	init_button();
	enable_button_interrupts();

	init_rs232();
	enable_rs232_interrupts();
	enable_rs232();

	fs_init();
	fio_init();
	register_romfs("romfs", &_sromfs);
	/* Create the queue used by the serial task.  Messages for write to
	 * the RS232. */
	serial_str_queue = xQueueCreate(10, sizeof(serial_str_msg));
	vSemaphoreCreateBinary(serial_tx_wait_sem);
	serial_rx_queue = xQueueCreate(1, sizeof(serial_ch_msg));

	
	/* Create a task to receive characters from the RS232 port and echo
	 * them back to the RS232 port. */
	xTaskCreate(my_shell_task,
					(signed portCHAR *) "Shell",
					512 , NULL,
					4, NULL);
	
	
	/* Start running the tasks. */
	vTaskStartScheduler();

	return 0;
}

void myTraceCreate      (){
}

void myTraceSwitchedIn  (){
}

void myTraceSwitchedOut	(){
}
/*
inline float myTraceGetTick(){
	// 0xE000E014 -> Systick reload value
	// 0xE000E018 -> Systick current value
	return ((float)((*(unsigned long *)0xE000E014)-(*(unsigned long *)0xE000E018)))/(*(unsigned long *)0xE000E014);
}

inline unsigned long myTraceGetTimeMillisecond(){
	return (xTaskGetTickCountFromISR() + myTraceGetTick()) * 1000 / configTICK_RATE_HZ;
}
*/
void vApplicationTickHook()
{
}
