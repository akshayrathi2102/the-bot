/*
 * Team Id: 5158
 * Author List: AKSHAY RATHI
 * Filename: task_5_final_submission
 * Theme: construct-o-bot
 * Functions: 65 functions (including main)
 * Global Variables: find in 'GLOBAL VARIABLE DECLARATION' SECTION
 *
 */


#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.c"
#define V 16

/******************************************GLOBAL VARIABLE DECLARATION**********************************************/

volatile unsigned long int LeftPulseCount = 0;  //keep track of left encoder
volatile unsigned long int RightPulseCount = 0; //keep track of right encoder

unsigned char Left_white_line = 0;      // stores adc value of left white line sensor
unsigned char Center_white_line = 0;   // stores adc value of center white line sensor
unsigned char Right_white_line = 0;    // stores adc value of right white line sensor

int left_ir_distance = 0;
int right_ir_distance = 0;

int nodes = 0;    //counts the number of nodes passed
int flag = 0;    // used in function follow()
int white=0;			// tells whether to follow white line or black line 0-black line   1-white line
int threshold = 0;  // stores the threshold value of white line sensors
int current_node=1;  // stores the current node
int next_node;  // stores the next node
int graph[V][V];  //adjacency matrix of graph
int path[V];  // stores the path to be traversed
int material_node[10];  //node at which warehouse is present
int destination_house_node[5];  // nodes at which houses are present
int materials_picked[10];  //marks the picked item , 0 for unpicked , 1 for picked
int trips=0;  //stores the number of trips to be made   1 trip= 1 pick + 1 place
int path_length;  // stores the length of the path to be traversed

char current_dir='S';  // stores the current direction of the graph
char dir[V][V];   //adjacency matrix of graph that stores relative direction of nodes



/********************************************END GLOBAL VARIABLE DECLARATION*****************************************************/


/*************************************************** INPUT SECTION **************************************************************/

int floor_array[5]={0,0,0,0,0};    // 0 for low rise house, 1 for high rise house
int house_total_requirement[5]={2,2,2,1,2}; // how many required at particular house
int which_material[10]={2,10,8,4,3,6,0,-1,1,7}; //requirement of each house


/*************************************************END INPUT SECTION******************************************************************/


/**************************************************PATH FINDING CODE************************************************************/
//14
/*
* Function Name: void dir_init(void)
* Input: void
* Output: void
* Logic: initialises the matrix of direction graph
* Example Call: dir_init();
*/
void dir_init(void)
{
	int i,j;
	for(i=0; i<V; i++)
	{
		for(j=0 ; j<V; j++)
		{
			dir[i][j]='X';
		}
	}

	dir[0][1]='E'; dir[0][3]='N';
	dir[1][0]='W'; dir[1][2]='E';
	dir[2][1]='W'; dir[2][4]='N';
	dir[3][0]='S'; dir[3][5]='N';
	dir[4][2]='S'; dir[4][6]='N';
	dir[5][3]='S'; dir[5][7]='N'; dir[5][6]='E';
	dir[6][4]='S'; dir[6][8]='N'; dir[6][5]='W';
	dir[7][5]='S'; dir[7][9]='N';
	dir[8][6]='S'; dir[8][10]='N';
	dir[9][7]='S'; dir[9][11]='N'; dir[9][10]='E';
	dir[10][8]='S'; dir[10][12]='N'; dir[10][9]='W';
	dir[11][9]='S'; dir[11][13]='N';
	dir[12][10]='S'; dir[12][15]='N';
	dir[13][11]='S'; dir[13][14]='E';
	dir[14][13]='W'; dir[14][15]='E';
	dir[15][12]='S'; dir[15][14]='W';
}

/*
* Function Name: void graph_init(void)
* Input: void
* Output: void
* Logic: initialises the matrix of path graph
* Example Call: graph_init()
*/
void graph_init(void)
{
	int i,j;
	for(i=0; i<V; i++)
	{
		for(j=0 ; j<V; j++)
		{
			graph[i][j]=-1;
		}
	}

	graph[0][1]=5; graph[0][3]=5;
	graph[1][0]=5; graph[1][2]=5;
	graph[2][1]=5; graph[2][4]=5;
	graph[3][0]=5; graph[3][5]=5;
	graph[4][2]=5; graph[4][6]=5;
	graph[5][3]=5; graph[5][7]=5; //graph[5][6]=3;
	graph[6][4]=5; graph[6][8]=5; //graph[6][5]=3;
	graph[7][5]=5; graph[7][9]=5;
	graph[8][6]=5; graph[8][10]=5;
	graph[9][7]=5; graph[9][11]=5; graph[9][10]=3;
	graph[10][8]=5; graph[10][12]=5; graph[10][9]=3;
	graph[11][9]=5; graph[11][13]=5;
	graph[12][10]=5; graph[12][15]=5;
	graph[13][11]=5; graph[13][14]=2;
	graph[14][13]=5; graph[14][15]=2;
	graph[15][12]=5; graph[15][14]=2;
}

/*
* Function Name: void no_of_trips(void)
* Input: void
* Output: void
* Logic: it calculates the total number of trips to be made.
				 It uses the array house_total_requirement and stores the result
				 in global variable trips
* Example Call: no_of_trips()
*/
void no_of_trips(void)
{
	for(int i=0; i<5; i++)
	{
		trips=trips+house_total_requirement[i];
	}
}

/*
* Function Name: int house_of(int n, int list[])
* Input: It takes the node no of warehouse and list of requirements as input.
* Output: In output it gives out the house of the given input node(of warehouse) in which the material is to be placed.
* Logic: In array 'material_node' integer(index/2) gives the house number of respective material in which it is to be placed.
				 So we take that array as input  and gives out the house no as output. If nothing found it gives -1 as output.
* Example Call: int house = house_of(3, material_node);
*/
int house_of(int n, int list[])
{
	for(int i=0; i<10; i++)
	{
		if(list[i]==n)
		{
			return i/2;
		}
	}

	return -1;
}

/*
* Function Name: int house_node(int house)
* Input: It takes the house number as input.
* Output: It gives out the node no on which the house is present.
* Logic: It simply maps the house numbers on respective node numbers.
* Example Call: int node = house_node(3);
*/
int house_node(int house)
{
	if(house == 0) return 5;
	if(house == 1) return 6;
	if(house == 2) return 9;
	if(house == 3) return 10;
	if(house == 4) return 14;
	else return -1;
}

/*
* Function Name: Material_node
* Input: void
* Output: void
* Logic: It uses the global array 'which_material' (which stores warehouse numbers)
 				 and assigns the value of warehouse nodes in another global array 'material_node'.
* Example Call: Material_node();
*/
void Material_node(void)
{
	int i;
	for(i=0; i<10; i++)
	{
		if(which_material[i] == 0 || which_material[i] == 1)
			material_node[i]=3;
		else if(which_material[i] == 2 || which_material[i] == 3)
			material_node[i]=4;
		else if(which_material[i] == 4 || which_material[i] == 5)
			material_node[i]=7;
		else if(which_material[i] == 6 || which_material[i] == 7)
			material_node[i]=8;
		else if(which_material[i] == 8 || which_material[i] == 9)
			material_node[i]=11;
		else if(which_material[i] == 10 || which_material[i] == 11)
			material_node[i]=12;
        else
            material_node[i]= -1;
	}
}

/*
* Function Name: int minDistance(int dist[], int set[])
* Input: An array set[] which contains the nodes to be visited,
*				 another array dist[] which stores the distance of these nodes from source
* Output: It returns the node in set[] which has minimum distance in dist[].
* Logic: It checks in set[] for unvisited nodes and finds the node with minimum distance in dist[] among them.
* Example Call: int min = minDistance(dist, set);
*/
int minDistance(int dist[], int set[])
{
		int min=1000, min_index;
		for (int j=0; j < V; j++)
		{
			if(set[j]==0 && dist[j]<=min)
			{
				min=dist[j];
				min_index = j;
			}
		}
	return min_index;
}

/*
* Function Name: int is_present(int u, int list[], int n)
* Input: An integer u, array list[], int n(length of list[])
* Output: 1 or 0 based on whether u is present in list or not.
* Logic: Function checks the presence of u in list[]
* Example Call: int check = isPresent(5, list, 10);
*/
int isPresent(int u, int list[],int n)
{
	for(int i=0; i< n; i++)
	{
		if(u==list[i])
			return 1;
	}
	return 0;
}

/*
* Function Name: int closest_node(int src, int list[], int n)
* Input: An int src which is source node,
				 an array list[] which contains different nodes,
				 an int n which is length of list[]
* Output: returns a node from list[] which is closest to src.
* Logic: Starting from source node, this function traverses each node based on
				 minimum distance in a breadth first manner. If the visited node is found in list
				 the function returns the visited node else it continues its search.
* Example Call: int node = closest_node(3, list, 10);
*/
int closest_node(int src, int list[], int n)
{
	int dist[V], set[V];
	int i, count;

	for(i=0; i < V; i++)		//initialisation
	{
		set[i]=0;  // setting all nodes to uvisited
		dist[i]=1000;   //setting distance of each node to infinite
	}

	dist[src]=0;  // distance of source is zero

	for(count=0; count< (V-1); count++)
	{
		int u = minDistance(dist, set);    // getting the closest unvisited node

		set[u]=1;  // marking that node to visited

		if(isPresent(u,list,n))   // if visited node is present in our list then return
			return u;

		for (int v = 0 ; v < V; v++)
		{		// if node is unvisited, path exists between u and v, distance of u is not infinite and new distance of v is less than previous distance
			if(!set[v] && graph[u][v]!=-1 && dist[u]!=1000 && (dist[u]+graph[u][v] < dist[v] ))
			{
				dist[v]=dist[u]+graph[u][v];   // assign new distance to v
			}
		}
	}

}

/*
* Function Name: void construct_path(int dest, int came_from[])
* Input: destination node and an array came_from[] which stores the
*				 node from where we have come to present node
* Output: stores the final path in global array path[]
* Logic: First we construct a reverse path using came_from array,
*				 then we reverses that array to get our final path.
* Example Call: construct_path(dest,came_from);
*/
void consrtuct_path(int dest,int came_from[])
{
		int rev_path[V];
		int i=1;int j=0;
		rev_path[0]=dest; // starting with destination
		while(i < V)
		{
			rev_path[i] = came_from[rev_path[i-1]];  //keep assigning the came_from value of previous element to get reverse path
			if(rev_path[i]== -1) break;
			i++;
		}
		while(i>0)
		{
			i--;
			path[j] = rev_path[i];   // reversing the rev_path to get final path
			j++;
		}
		path_length=j;
		/*
		for(i=0; i<j; i++)
		{
			printf("\t ->%d", path[i]);
		}
		*/
	}

/*
	* Function Name: void shortest_path(int src, int dest)
	* Input: source node and destination node
	* Output: gives the shortest path between source and destination
	* 				and stores it in global variable path[]
	* Logic: uses dijkstra algorithm to find shortest path between source and destination
	* Example Call: shortest_path(current_node,11);
	*/
void shortest_path(int src, int dest)
{
	int dist[V], set[V], came_from[V];
	int i, count;

	for(i=0; i < V; i++)		//initialisation
	{
		set[i]=0; // setting all nodes to uvisited
		dist[i]=1000; //setting distance of each node to infinite
	}

	dist[src]=0; // distance of source is zero
    came_from[src]=-1; // came_from source is -1

	for(count=0; count< (V-1); count++)
	{
		int u = minDistance(dist, set);  // getting the closest unvisited node
		set[u]=1;  // marking that node to visited

		if(u==dest)
			break;  // if visited node is destination then break the loop

		for (int v = 0 ; v < V; v++)
		{// if node is unvisited, path exists between u and v, distance of u is not infinite and new distance of v is less than previous distance
			if(!set[v] && graph[u][v]!=-1 && dist[u]!=1000 && (dist[u]+graph[u][v] < dist[v] ))
			{
				dist[v]=dist[u]+graph[u][v];  // assign new distance to v
				came_from[v]=u; // set came_from of v as u
			}
		}
	}

	consrtuct_path(dest, came_from);  // construct the path using came_from and destination

}

/*
* Function Name: void eliminate(int n)
* Input: int n.
* Output: removes the element n from array material_node.
* Logic: find element n in material_node[] and replaces it with -1
* Example Call: eliminate(5);
*/
void eliminate(int n)
{
	for(int i=0;i<10; i++)
	{
		if(material_node[i]==n)
		{
			material_node[i]=-1;
			return;
		}
	}
}

/*
* Function Name: int index_of(int n, int list[])
* Input: int n and an array list[]
* Output: returns the index of n in list
* Logic: search for n in list and return its index
* Example Call: int n = index_of(5, materials_node);
*/
int index_of(int n,int list[])
{
	for(int i=0;i<10;i++)
	{
		if(list[i]==n)
		return i;
	}
	return -1;
}

/*
* Function Name: void initialize(void)
* Input: void
* Output: void
* Logic: initialises matrix of path ggraph and direction graph,
					calculate no of trips and also calls Material_node.
* Example Call: initialize();
*/
void initialize(void)
{
	graph_init();
	no_of_trips();
	Material_node();
	dir_init();
}
/************************************************ END OF PATH FINDER CODE ***************************************************/

/************************************************ TIMERS ************************************************************************/
/*
* Function Name: void timer5_init_free(void)
* Input: void
* Output: void
* Logic: frees initialises the timer5 int PWM mode for velocity control
*			   Prescale :256
* 	 		 PWM 8bit fast, TOP=0x00FF
* 			 Timer Frequency:225.000Hz
* Example Call: timer5_init();
*/
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

/*
* Function Name: void timer1_init(void)
* Input: void
* Output: void
* Logic: initialises the timer 1 for servo comtrol
* Example Call: void timer1_init();
*/
void timer1_init()
{
	TCCR1A = 0x00;
	ICR1 = 1023 ;  // TOP= 1023
	TCNT1H = 0xFC ; // counter high value to which
	TCNT1L = 0x01 ;
	OCR1A = 1023 ;
	OCR1B = 1023 ;
	TCCR1A = 0xAB ;
	TCCR1B = 0x0C ;
}
/*******************************************************************************************************************************/

/************************************************** ADC *********************************************************************/
/*
* Function Name: void adc_pin_config(void)
* Input: void
* Output: void
* Logic: initialises adc pins
* Example Call: adc_pin_config();
*/
void adc_pin_config (void)
{
	DDRF = DDRF & 0xF1;  //1111 0001
	PORTF = PORTF | 0x0E;  //0000 1110
}

/*
* Function Name: void adc_init(void)
* Input: void
* Output: void
* Logic: initialises adc registers
* Example Call: adc_init();
*/
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

/*
* Function Name: void ADC_Conversion(void)
* Input: unsigned char Ch, channel number
* Output: Converted digital values of channel no ch
* Logic: converts analog signals to digital of signal at channel no ch
* Example Call: unsigned char A = ADC_Conversion(1);
*/
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

/***************************************************************************************************************************/

/************************************************ BUZZER *********************************************************************/
/*
* Function Name: void buzzer_pin_config(void)
* Input: void
* Output: void
* Logic: initialises the buzzer pins
* Example Call: buzzer_pin_config();
*/
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC | 0x08;		//Setting PORTC 3 logic low to turnoff buzzer
}

/*
* Function Name: void buzzer_off(void)
* Input: void
* Output: void
* Logic: turns off the buzzer
* Example Call: buzzer_pin_config();
*/
void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

/*
* Function Name: void buzzer_on(void)
* Input: void
* Output: void
* Logic: turns the buzzer on.
* Example Call: buzzer_on();
*/
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}
/*******************************************************************************************************************************/

/************************************************ SERVO ************************************************************************/
/*
* Function Name: void servo_port_init(void)
* Input: void
* Output: void
* Logic: initialises the servo ports
* Example Call: servo_port_init();
*/
void servo_port_init()
{
	DDRB = DDRB | 0x60 ;// 0110 0000  setting the B5 and B6 as output pins
	PORTB = PORTB | 0x60 ; // 0110 0000 setting the output to high
}

/*
* Function Name: void servo_1(void)
* Input: unsigned char degrees
* Output: void
* Logic: sets the servo_1 to 'degrees' degree
* Example Call: servo_1(90);
*/
void servo_1(unsigned char degrees)
{
	float regval = ((float)degrees * 0.512) + 34.56 ;
	OCR1A = (uint16_t) regval;
}

/*
* Function Name: void servo_2(void)
* Input: unsigned char degrees
* Output: void
* Logic: sets the servo_2 to 'degrees' degree
* Example Call: servo_2(90);
*/
void servo_2(unsigned char degrees)
{
	float regval = ((float)degrees * 0.512) + 34.56 ;
	OCR1B = (uint16_t) regval;
}

/*
* Function Name: void servo_1_free(void)
* Input: void
* Output: void
* Logic: frees the servo_1 , elbow servo
* Example Call: servo_1_free();
*/
void servo_1_free()
{
	OCR1A = 1023;
}

/*
* Function Name: void servo_2_free(void)
* Input: void
* Output: void
* Logic: frees the servo_2, claw servo
* Example Call: servo_2_free();
*/
void servo_2_free()
{
	OCR1B = 1023;
}
/********************************************************************************************************************************/

/***************************************************** SHARP SENSOR *************************************************************/
/*
* Function Name: void sharp_sensor_init(void)
* Input: void
* Output: void
* Logic: initialises sharp sensors
* Example Call: sharp_sensor_init();
*/
void sharp_sensor_init(void)
{
	DDRK = DDRK & 0xF9 ; // 1111 1001  setting B0 B1 as input for ir sensors
	PORTK = PORTK | 0x06 ; // 0000 0110 enabling pullups
}

/*
* Function Name:int Sharp_GP2D12_estimation(unsigned char adc_reading)
* Input: unsigned char adc_reading
* Output: the actual of sharp sensor distance in millimeters from wall
* Logic: This Function calculates the actual distance in millimeters(mm) from the input analog value of Sharp Sensor.
* Example Call:int dist = Sharp_GP2D12_estimation(ADC_Conversion(7));
*/
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

/*
* Function Name: int wall_distance(unsigned char ch)
* Input: unsigned char ch, the channel no to which sharp sensors are connected
* Output: disatnce from wall
* Logic: This Function calculates the actual distance in millimeters(mm) from the input channel number of Sharp Sensor.
* Example Call: wall_distance();
*/
int wall_distance(unsigned char ch)
{
	int sharp, distance;
	sharp = ADC_Conversion(ch);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
	distance = Sharp_GP2D12_estimation(sharp);

	return distance;
}
/*******************************************************************************************************************************/

/************************************************* LCD ************************************************************************/
/*
* Function Name: void lcd_port_config(void)
* Input: void
* Output: void
* Logic: initialises LCD ports
* Example Call: lcd_port_config();
*/
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

/*
* Function Name: void print_sensor(char row, char column, unsigned char channel)
* Input: row no, column no, and channel no.
* Output: void
* Logic: prints the ADC value of channel on LCD at given row and column
* Example Call: print_sensor(1,1,3);
*/
void print_sensor(char row, char coloumn,unsigned char channel)
{
	unsigned char ADC_Value;

	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

/****************************************************************************************************************************/

/************************************************* ENCODER *******************************************************************/
/*
* Function Name: void encoder_pin_config(void)
* Input: void
* Output: void
* Logic: initialises encoder pins
* Example Call: encoder_pin_config();
*/
void encoder_pin_config(void)
{
	DDRE = DDRE & 0xCF ; // 1100 1111 port E4 is for LEFT encoder and E5 is for RIGHT encoder
	PORTE = PORTE | 0x30 ; // 0011 0000  enabling internal pull ups
}

/*
* Function Name: void encoder_interrupt_init(void)
* Input: void
* Output: void
* Logic: initialises encoder interrupt
* Example Call: encoder_interrupt_init();
*/
void encoder_interrupt_init(void)
{
	cli();
	EICRB = EICRB | 0x0A ; // external interrupt control register B 0000 1010
	EIMSK = EIMSK | 0x30 ; // external interrupt mask register 0011 0000  pin4 for LEFT encoder and pin5 for RIGHT encoder
	sei();
}

/*
* Function Name: ISR(INT5_vect)
* Input: void
* Output: void
* Logic: This is interrupt service routine, it increments thr variable RightPulseCount on every interrupt generated by right position encoder.
* Example Call: NIL
*/
ISR(INT5_vect)    // ISR for right position encoder
{
	RightPulseCount++;
}

/*
* Function Name: ISR(INT4_vect)
* Input: void
* Output: void
* Logic: This is interrupt service routine, it increments thr variable RightPulseCount on every interrupt generated by left position encoder.
* Example Call: NIL
*/
ISR(INT4_vect)    // ISR for left position encoder
{
	LeftPulseCount++;
}

/***************************************************************************************************************************/

/************************************************ MOTORS ******************************************************************/
/*
* Function Name: void motion_pin_config(void)
* Input: void
* Output: void
* Logic: initialises motor pins
* Example Call: motion_pin_config();
*/
void motion_pin_config(void)
{
	DDRA = DDRA | 0x0F ;
	PORTA = PORTA & 0x00 ;
	DDRL = DDRL | 0x18 ; //0001 1000
	PORTL = PORTL | 0x18 ;//0001 1000
}

/*
* Function Name: void motion_set(unsigned char direction)
* Input: unsigned char direction
* Output: void
* Logic: sets motion of both motors using given diretion
* Example Call: motion_set(0x02);
*/
void motion_set(unsigned char direction)
{
	unsigned char PortARestore = 0;

	direction &= 0x0F;
	PortARestore = PORTA;
	PortARestore &= 0xF0;
	PortARestore |= direction;
	PORTA = PortARestore;
}

/*
* Function Name: void velocity(unsigned char left_motor, unsigned char right_motor)
* Input: unsigned char left_motor, unsigned char right_motor
* Output: void
* Logic: sets velocity of both motors to given values
* Example Call: velocity(150,150);
*/
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

/*******************************************************************************************************************************/

/************************************************MOTION FUNCTIONS****************************************************************/

//makes the robot move forward
void forward(void)
{
	motion_set(0x06) ; //0110
}
//stops the robot
void stop(void)
{
	motion_set(0x00); //0000
}
//make the robot move backward
void backward(void)
{
	motion_set(0x09); //1001
}
//turns the robot to left by rotating left wheel backward and right wheel forward
void left(void)
{
	motion_set(0x05) ; //0101
}
// turns the robot to right by rotating right wheel backward and left wheel forward
void right(void)
{
	motion_set(0x0A); //1010
}
// turns the robot to left by stopping left wheel and rotating right wheel forward
void soft_left(void)
{
	motion_set(0x04); //0100
}
// turns the robot to right by stopping right wheel and rotating left wheel forward
void soft_right(void)
{
	motion_set(0x02); //0010
}




void distance_mm(unsigned int DistanceInMM)
{
	float reqPulseCount = 0;
	unsigned int reqPulseCountInt = 0;

	reqPulseCount = DistanceInMM/0.378154; //division by resolution dia of wheels =65mm
	reqPulseCountInt = (unsigned long int) reqPulseCount ;

	RightPulseCount = 0;
	while(1)
	{
		if(RightPulseCount > reqPulseCountInt)
		{
			break;
		}
	}
	stop();
}
//moves the bot forward by given distance in mm
void forward_mm(unsigned int DistanceInMM)
{
	forward();
	//velocity(156,149);
	velocity(130,125);
	//velocity(104,99);
	distance_mm(DistanceInMM);
}
//moves the bot backward by given distance in mm
void backward_mm(unsigned int DistanceInMM)
{
	backward();
	velocity(104,99);
	//velocity(130,125);
	//velocity(156,149);
	distance_mm(DistanceInMM);
}
//turns the bot right by given degrees
void right_deg(unsigned int AngleInDegree)
{
	float distance = 0;
	unsigned int distanceInt = 0;

	distance = 1.589449 * AngleInDegree ; // distance= pi*(distance between wheels=205mm)*(angle to be turned)/(360*2)
	distanceInt = (unsigned int)distance;

	right();
	velocity(105,100);
	//velocity(156,149);
	//velocity(130,125);
	//velocity(89,85);
	distance_mm(distanceInt);
}
// turns the bot left by given degrees
void left_deg(unsigned int AngleInDegree)
{
	float distance = 0;
	unsigned int distanceInt = 0;

	distance = 1.589449 * AngleInDegree ; // distance= pi*(distance between wheels)*(angle to be turned)/(360*2)
	distanceInt = (unsigned int)distance;

	left();
	//velocity(156,149);
	//velocity(130,125);
	velocity(105,100);
	distance_mm(distanceInt);
}



// turns the robot left till black line is encountered
void left_wls(int n)
{
	Center_white_line=ADC_Conversion(2);
	int count=0;
	while(Center_white_line<threshold && count < n)
	{
		left_deg(5);
		Center_white_line=ADC_Conversion(2);
		count++;
	}
	stop();
	return;
}
// turns the robot right till black line is encountered
void right_wls(int n)
{
	Center_white_line=ADC_Conversion(2);
	int count=0;
	while(Center_white_line<threshold && count < n )
	{
		right_deg(5);
		Center_white_line=ADC_Conversion(2);
		count++;
	}
	stop();
	return;
}
//turns the robot left till white line is encountered
void white_left_wls(int n)
{
	Center_white_line=ADC_Conversion(2);
	int count=0;
	while(Center_white_line>threshold && count < n)
	{
		left_deg(5);
		Center_white_line=ADC_Conversion(2);
		count++;
	}
	stop();
	return;
}
//turns the robot right till white line is encountered
void white_right_wls(int n)
{
	Center_white_line=ADC_Conversion(2);
	int count=0;
	while(Center_white_line>threshold && count < n )
	{
		right_deg(5);
		Center_white_line=ADC_Conversion(2);
		count++;
	}
	stop();
	return;
}

/*********************************************************************************************************************/

/********************************************** PICK AND PLACE  *******************************************************/
/*
* Function Name: void pick(void)
* Input: void
* Output: void
* Logic: picks the construction material from warehouse
* Example Call: pick();
*/
void pick(void)
{
	left_deg(12);
	_delay_ms(200);

	servo_2(80);  // claw open
	_delay_ms(1500);

	servo_1(80);  //elbow  down
	_delay_ms(1500);


	right_deg(15);
	_delay_ms(200);


	servo_2(200);  //claw grip
	_delay_ms(1500);

	servo_1(0);   // elbow up
	_delay_ms(1500);
}

/*
* Function Name: void high_place(void)
* Input: void
* Output: void
* Logic: places the construction material in high place houses
* Example Call: high_place();
*/
void high_place(void)
{
	left_wls(5);
	_delay_ms(200);
	right_wls(9);
	_delay_ms(200);

	servo_2(120);
	_delay_ms(200);
}

/*
* Function Name: void low_place(void)
* Input: void
* Output: void
* Logic: places the construction material in low place houses
* Example Call: low_place();
*/
void low_place(void)
{

	servo_1(45);
	_delay_ms(500);
	/*
	left_wls(5);
	_delay_ms(200);
	right_wls(9);
	_delay_ms(200);
	*/

	servo_2(120);
	_delay_ms(200);

	servo_1(0);
	_delay_ms(200);


}

/************************************************************************************************************************/

/********************************************************** MISCELLANOUS *********************************************************/
/*
* Function Name: void ports_init(void)
* Input: void
* Output: void
* Logic: initialises the ports in device
* Example Call: ports_init();
*/
void ports_init(void)
{
	buzzer_pin_config();
	servo_port_init();
	adc_pin_config();
	motion_pin_config();
	encoder_pin_config();
	sharp_sensor_init();
	lcd_port_config();
}

/*
* Function Name: void devices_init(void)
* Input: void
* Output: void
* Logic: initialises the device
* Example Call: devices_init();
*/
void devices_init(void)
{
	cli();
	ports_init();
	encoder_interrupt_init();
	adc_init();
	timer5_init();
	timer1_init();
	sei();
}

/*
* Function Name: void set_threshold(void)
* Input: void
* Output: void
* Logic: autocalibrates the threshold value for white line sensors
* Example Call: set_threshold();
*/
void set_threshold(void)
{

	float avg, float_threshold=0;;

	for(int i=1; i<=100; i++)
	{
		Left_white_line = ADC_Conversion(1);
		Center_white_line = ADC_Conversion(2);
		Right_white_line = ADC_Conversion(3);

		avg = (Left_white_line+Center_white_line+Right_white_line)/3; // takes the average of all the initial readings of white line sensor

		float_threshold=((i-1)*float_threshold+avg)/i; // takes the average of 100 such 'avg' to get final threshold

	}

	threshold = (int)float_threshold + 25;  // adjustment in threshold value
}

/*
* Function Name: void follow(void)
* Input: void
* Output: void
* Logic: fllows the line and returns on node
* Example Call: follow();
*/
void follow(void)
{
	while(1)
	{


		Left_white_line = ADC_Conversion(1);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(3);	//Getting data of Right WL Sensor
		left_ir_distance = wall_distance(9);
		right_ir_distance = wall_distance(10);

	/*	print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		lcd_print(1, 13, threshold, 3);
		lcd_print(2,1,nodes,2);
		lcd_print(2,5,left_ir_distance,3);
		lcd_print(2,9,right_ir_distance,3);
	*/


		if(white==0)
		{
			flag=0;

			if(Center_white_line>threshold )    //Center WL Sensor on black
			{
				flag=1;
				forward();
				//velocity(104,99);
				velocity(130,124);
				//velocity(156,149);
			}

			if((Left_white_line>threshold) && (flag==0))  //Left WL Sensor on black
			{
				flag=1;
				forward();
				//velocity(79,99);
				velocity(99,125);
				//velocity(119,149);
				_delay_ms(200);
			}

			if((Right_white_line>threshold) && (flag==0))  // Right WL Sensor on black
			{
				flag=1;
				forward();
				//velocity(104,73);
				velocity(130,91);
				//velocity(156,110);

			}


			if((Center_white_line>threshold && Left_white_line>threshold && Right_white_line>threshold)||(Center_white_line>threshold && Right_white_line>threshold && nodes==1))  //node
			{
				forward_mm(100);
				stop();
				_delay_ms(300);
				nodes++;
				lcd_print(2,1,nodes,2);
				return;
			}

			if(Center_white_line>threshold && Left_white_line<threshold && Right_white_line>threshold && (flag == 0))  //black white black
			{
				flag=1;
				white=1;
				forward();
				velocity(0,0);
			}

			if(Center_white_line<threshold && Left_white_line<threshold && Right_white_line<threshold && (flag==0))  // white white white
			{
				while(Center_white_line<threshold && Left_white_line<threshold && Right_white_line<threshold)
				{

					left_wls(6);

					right_wls(10);
				}
			}


		}

		else
		{
			flag=0;

			if(Center_white_line<threshold )    //Center WL Sensor on white
			{
				flag=1;
				forward();
				//velocity(104,99);
				velocity(130,124);
				//velocity(156,149);
			}

			if((Left_white_line<threshold) && (flag==0))  //Left WL Sensor on white
			{
				flag=1;
				forward();
				//velocity(79,99);
				velocity(99,125);
				//velocity(119,149);
				_delay_ms(200);
			}

			if((Right_white_line<threshold) && (flag==0))  // Right WL Sensor on white
			{
				flag=1;
				forward();
				//velocity(104,73);
				velocity(130,91);
				//velocity(156,110);

			}


			if((Center_white_line<threshold && Left_white_line<threshold && Right_white_line<threshold) &&(flag==0))  //node
			{
				flag=1;
				forward_mm(100);
				stop();
				_delay_ms(300);
				nodes++;
				lcd_print(2,1,nodes,2);
				return;
			}

			if(Center_white_line<threshold && Left_white_line>threshold && Right_white_line<threshold && (flag == 0))  //white black white
			{
				flag=1;
				white=0;
				forward();
				velocity(0,0);
			}

			if(Center_white_line>threshold && Left_white_line>threshold && Right_white_line>threshold && (flag==0))  // black black black
			{
				while(Center_white_line>threshold && Left_white_line>threshold && Right_white_line>threshold)
				{

					white_left_wls(6);

					white_right_wls(10);
				}
			}


		}

	}
}

/*
* Function Name: void orient(char next_dir)
* Input: next direction
* Output: void
* Logic: turns the robot to given next direction
* Example Call: orient('W');
*/
void orient(char next_dir)
{
	if((current_dir == 'N' && next_dir == 'N')||
	(current_dir == 'E' && next_dir == 'E')||
	(current_dir == 'W' && next_dir == 'W')||
	(current_dir == 'S' && next_dir == 'S'))
	{
		return;
	}

	if((current_dir == 'N' && next_dir == 'W')||
	(current_dir == 'E' && next_dir == 'N')||
	(current_dir == 'W' && next_dir == 'S')||
	(current_dir == 'S' && next_dir == 'E'))
	{
		left_deg(90);
		current_dir = next_dir;
		return;
	}

	if((current_dir == 'N' && next_dir == 'E')||
	(current_dir == 'E' && next_dir == 'S')||
	(current_dir == 'W' && next_dir == 'N')||
	(current_dir == 'S' && next_dir == 'W'))
	{
		right_deg(90);
		current_dir = next_dir;
		return;
	}

	if((current_dir == 'N' && next_dir == 'S')||
	(current_dir == 'E' && next_dir == 'W')||
	(current_dir == 'W' && next_dir == 'E')||
	(current_dir == 'S' && next_dir == 'N'))
	{
		left_deg(180);
		current_dir = next_dir;
		return;
	}
	else return;

}

/*
* Function Name: void traverse(int path[])
* Input: int path[]
* Output: void
* Logic: follows the path given in path[]
* Example Call: traverse(path);
*/
void traverse(int path[])
{
	current_node=path[0];
	 for(int i=0; i<(path_length-1); i++)
	{
		next_node=path[i+1];
		if(next_node==-1)
		 return;
		orient (dir[current_node][next_node]);
		follow();
		current_node = next_node;
	}

}

/***********************************************************************************************************************************/
/********************************************************* MAIN ********************************************************************/

void main()
{
	initialize();
	devices_init();
	lcd_set_4bit();
	lcd_init();
	buzzer_off();
	set_threshold();
	_delay_ms(3000);

	int cn;  // closest node
	int hn;  // house node
	int index, house;
	int n=10;

	follow();

	while(trips!=0)
	{
		cn = closest_node(current_node, material_node, n);  // find closest node in material_node[] from current node
		shortest_path(current_node, cn);  // calculate shortest path from current node to closest node
		traverse(path);  // traverse that path
		index = index_of(cn,material_node); // find index of cn in material_node, used for finding warehouse

		if(which_material[index]%2==0) orient('W');  // if index is even orient to west
		else orient('E');														// else orient to east

		pick();    // pick the item

		/*buzzer_on();
		_delay_ms(500);
		buzzer_off();*/

		/**********************************************material picked*********************************************/

		house=house_of(cn, material_node);  // find the house in which picked maaterial is to be placed
		hn=house_node(house);		// find the node of that house
		shortest_path(current_node, hn);  // calculate the shortest path from current node to house node
		traverse(path);  // traverse that path

		if(house==4) orient('S');    // if house no is 4 orient to south
		else if(house%2==0) orient('W');  // else if house no is even orient to west
		else orient('E');  // else orient to east

		if(floor_array[house]==0) low_place();  // if house is low rise then execote low_place
		else high_place();  // else execute high_place

		/*buzzer_on();
		_delay_ms(500);
		buzzer_off();*/

		/***********************************************material placed**********************************************/

		eliminate(cn);  // eliminate  the picked material from material_node
		trips--;


	}


	buzzer_on();
	_delay_ms(5000);
	buzzer_off();

}

/******************************************************** END CODE *******************************************************************/
