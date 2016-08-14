

// Main program Version 0.9.1 080116
// Support for LCD Dispaly added





#define Software_Version	0.051

// Analog Scaling Values

 float ADC_min = 0;    // minimum Arduino A/D convertor value
 float ADC_max = 1024;  // maximum Arduino A/D convertor value (10 bit A/D)

 float PV_min = 0;   // minimum PV Array Voltage
 float PV_max = 44.620;    // maximum PV Array Voltage in volts

 float BV_min = 0;    // minimum battery voltage
 float BV_max = 51.000;    // maximum battery voltage in volts ..voltage that gives max output for battery voltage read

 float GV_min = 0;    // minimum grid volatage
 float GV_max = 56.000;    // maximum grid voltage in volts

 float GI_min = 0.00;    // minimum grid current
 float GI_max = 11.500;    // maximum grid current in amps

 float PVI_min = 0;    // minimum PV Array current
 float PVI_max = 11.000;    // maximum PV Array current in volts

 float BT_min = -20.4;    // minimum battery temperature
 float BT_max = 234.7;    // maximum battery temperature


// board setup parameters
 float Bat_Lo_sp =31.352; // the battery charge voltage commanded with the setpoint from the controller to max
 float Bat_Hi_sp =55.409; //the battery charge voltage commanded with the setpoint from the controller at min


 //system setup parameters 
 float PV_day = 8.000; // voltage setpoint for determining if it is daylight outsidein millivolts
 float PV_ocV= 44.400; //  open circuit voltage of the pv array... measured at 255 reference

 float Grid_V = 10.000;  // voltage where grid power is determined to be available in millivolts
 float Up_BOV = 38.700;  // 85% Charged Voltage in millivolts ... reset to 39000 after test
 float Lo_BOV = 36.100;  // 36 volts is about 25% charged... some charts show 50% Charged Voltage in millivolts 
 float PWR_B = 6.400;  // power available from battery in watts
   
const unsigned long ocv_time = 15000;   // battery resting time before read during charging in milliseconds   
float Bat_MaxI= 2.100; // current limit for battery charging

float Bat_Temp_Hi = 125.0;  // High Battery Temperature in deg F
float Bat_Float = 42.105; // Bat_Float is the battery float charging voltage in millivolts
float Bat_Charge = 44.805; // Bat_Charge is the battery charging voltage in millivolts
float Bat_Ch_Adj = .078; // Bat_Ch_Adj is the negative temp co efficient for charging the battery in millivolts

float BatteryCutOffVoltage = 34.5;      // Lowest Voltage to discharge battery
const byte TimeBetweenMeasurements = 10;        // Number of minutes between discharge time calculations

//Void Loop Parameters

//unsigned long Loop_Delay =00 ;

// Battery Trickle Charge Setpoint

int B_TrickleVsp;
int B_TrickleS;

// Intial PV Array System Parameters

float PV_V; // Voltage output from PV Array
float PV_V_last; // Last Voltage output reading from PV Array
float PV_I; // Current Output from PV DC/DC Convertor
float power;  // Power Output from PV DC/DC convertor
float Power_Now;
float Power_Last;
float Power_1;
float Power_2;
float Power_3;
float PV_Volt1; // PV Voltage 1
float PV_Volt2; // PV Voltage 2
float PV_Volt3; // PV Voltage 3
unsigned long Regulator_timer;
float Power_D;
byte MPPT_Last;
byte MPPT_1;
byte MPPT_2;
byte MPPT_3;
byte Regulator_ON;
byte MPPT_Sel;
byte PV_overload; // PV stays at low voltage
byte PV_underload; // PV stays at high voltage
byte PV_FastFlag; // marker to know when PV fast search is activated
float Pwr_1; // PV convertor overload function
float Pwr_2;  // PV convertor overload function
float Pwr_3;  // PV convertor overload function
byte PV_itisday; // marker set to 1 if it is daytime.. 0 if it is night
byte PV_latch; // lets you know if PV is turned off due to night or some other reason
float Day_counter; // counts the number of cycles where PV is > than the daylight setpoint
float Night_counter; // countes the number of cycles where PV shows it is night
byte PV_OL_Counter;   // PV convertor overload function
byte PV_Knee; // the PV knee is identified if 1,  not identified if 0
byte PV_reset; // counts times pv voltage is low if exceeded resets PV convertor
byte PV_Ocount; // counts the number of times pv voltage under 8 volts.. overload indication
byte PV_O_counter; // counts the number pf PV_Ocounts
byte PV_Ucount; // counts the number of times pv voltage is near the top limit.. not providing 48V indication
byte PV_Zcount; // counts the number of times the PV power is zero
byte PV_Ncount; //counts number of times the PV voltage is above 10 Volts
byte BusV_low; // inidcator that the 48 volt bus is low
unsigned long PV_OL_timer;
unsigned long PV_Update_timer;
float PVpwr_Av; // PV power to grid setpoint



// Required DELAY Function Intialization Parameters

byte Timer_Set[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };    // Intialize Timer Set state to false
unsigned long Timer_T0[20];
unsigned long Timer_OVC;
byte Timer_Exp;              // has Timer expired?
byte OVC = 0; // set for ovc first call


// Required Battery Initialization Parameters

byte Lo_Bat_flag; // this is the flag that shows battery is under Lo_BOV... latching
byte Hi_Bat_flag; // this is the flag that shows the battery is over Up_BOV
byte Bat_OL; // this is the flag that shows that the battery charge reference has reverted to the lower value
byte Float_Flag = false;  //flag for trickle charger calibration
byte OCR1A_Peak;  // this records the OCR1A value for the peak current
byte OCR1A_Last; 
byte OCR1A_Last_1;
byte OCR1A_Last_2;
byte OCR1A_Last_3;
byte OCR1A_Last_4;
byte Bat_zero_count; // counts the number of times in a row that battery charge current is 0
float Ch_V;    // Charging Voltage
float Fl_V;    // Float Voltage
float Bat_OV;       // Battery Operating Voltage
float Bat_Temp;    // Battery Temperature
float Bat_I_Read;// this is the battery charging current
float Bat_Start_I;  // this is the peak charging current recorded
float Bat_Peak_V; 
float Bat_V_Read_T; // may not be needed
float Bat_I_Read_T; // test may not be needed
float Bat_V_Read; // this is the battery charging feedback voltage
float Bat_V_Read_Last; //last battery Voltage read
float Bat_V_Last_5;
float Bat_V_Last_4;
float Bat_V_Last_3;
float Bat_V_Last_2;
  




float Bat_Pwr; // present charging power 
float Bat_I_Last; // Last charging power
float  Bat_I_Last_1;
float Bat_Pwr_D; // difference in battery charging power now to last



byte Shed; // 1 is command to shed load
byte tmp; // temporary variable
float Bat_Rem = 1.00; // remaining battery charge in percent
//float tmp2; // temporary float variable
byte Ch_Cycle; // Flag to indicate Battery is charging
byte Sleep; // 1= sleep mode for when it is night and battery is < min set point
byte Grid_Op; // battery low operate from grid

//  Values for Serial Monitor Diagnostics
int input_str[10]; // input strings of up to 10 characters
float analog;
int pin_state;
int char_cnt = 0;
int duty_cycle;


// Grid Connection Parameters

float V_DC;  // Grid voltage
float I_DC; // Grid Current or battery charging current
float I_DC_Last; // the last grid current read
float I_DC_Peak;
float Pwr_to_grid;   // Grid Power Output
float Grid_Pout; // not sure if this is still used
float Tot_Power;


// Parameters for Time of Day Clock

int hr;
int min;
int sec;
unsigned long T0;

// Required for battery discharge method
int y1;      // variable used to calculate float
int y2;     // variable used to calculate float
float x1;   // variable used to calculate float
float x2;   // variable used to calculate float
byte numberOfMinutes;
int dischargeMinutes;               // number of minutes since start of discharge


// Parameters for LCD Dispaly Function
byte input_buffer[10];
byte char_count;
byte function_number;
byte parameter_number;
byte parameter_code;
byte message_received = 0;
byte char_pointer;
String data_string = "";
float return_value;
byte digital_IO_offset[] = { 2,3,3,3,3,3,6,6 };
float analog_scaling_values[] = { PV_min,PV_max,BV_min,BV_max,GV_min,GV_max,GI_min,GI_max,PVI_min,PVI_max,BT_min,BT_max };
float *analog_scaling[] = { &ADC_min,&ADC_max,&PV_min,&PV_max,&BV_min,&BV_max,&GV_min,&GV_max,&GI_min,&GI_max,&PVI_min,&PVI_max,&BT_min,&BT_max };
float *setpoint[] = { &Bat_Lo_sp, &Bat_Hi_sp, &PV_day, &PV_ocV, &Grid_V, &Up_BOV, &Lo_BOV, &PWR_B, &Bat_MaxI, &Bat_Float, &Bat_Charge, &Bat_Ch_Adj, &BatteryCutOffVoltage };
float *calc_analog[] = { &Power_Now,&Ch_V, &Bat_OV, &Bat_I_Read, &Bat_V_Read, &Pwr_to_grid };
byte  *system_digital[] = { &PV_itisday,&BusV_low,&Lo_Bat_flag,&Hi_Bat_flag,&Ch_Cycle,&Sleep,&BusV_low };


void setup()
{

 Serial.begin(9600);  //enable serial monitor at 9600 baud

  // Intialize Pins as Input/Outputs
  pinMode(0, INPUT_PULLUP);  // It is absolutely critical to use pullup when using LCD display or you will damage Arduino Board
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT); pinMode(4, OUTPUT); pinMode(5, OUTPUT); pinMode(6, OUTPUT);
  pinMode(7, OUTPUT); pinMode(8, OUTPUT); pinMode(12, OUTPUT); pinMode(13, OUTPUT);
  pinMode(10, INPUT);

  // Intialize Outputs to turn Off All Devices

  digitalWrite(2, LOW); // PV DC/DC Convertor
  digitalWrite(4, LOW); // Battery Charger
  digitalWrite(5, HIGH); // Grid to 48V Bus
  digitalWrite(6, LOW); // 48V Bus To Grid
  digitalWrite(7, HIGH); // Battery Discharge to 48V Bus
  digitalWrite(8, LOW); // 24V DC/DC Convertor
  digitalWrite(12, LOW);//set 12 output on for testing
  digitalWrite(13, LOW); // 48V Current Limited

  // Intialize Arduino Timer2 for PV Array functionality

  pinMode(3, INPUT);  // Timer 2 Channel B (PV DC/DC Convertor) pin Output enable
  pinMode(11, OUTPUT);  // Timer 2 Channel A (PV Array Current) pin output enable
  pinMode(2, OUTPUT);   // PV DC/DC Convertor Enable/Diable pin
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM22) | _BV(WGM20); // Set Timer 2 Mode to PWM Phase Correct
 // TCCR2B = _BV(CS21);  // Intialalize Timer 2 Frequency to less than 32K Hz
  OCR2A = 255;  // Timer 2 Channel A (PV Voltage DC/DC Convertor) Duty Cycle intially set to 100%
 // OCR2B = 255;  // Timer 2 Channel B (PV Array Current) Duty Cycle intially set to 100%

  // Intialize Arduino Timer1 for Battery & Grid functionality

  pinMode(9, OUTPUT);  // Timer 1 Channel A (Battery Charger) pin output enable
  pinMode(10, OUTPUT); // Timer 1 Channel B (48V Bus to Grid) pin output enable
  pinMode(4, OUTPUT);   // PV DC/DC Convertor Enable/Diable pin
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM12) | _BV(WGM10);  // Set Timer 1 Mode to PWM Phase Correct
  TCCR1B = _BV(CS10);  // Intialalize Timer 1 Frequency to less than 32K Hz
  OCR1A = 255;  // Timer 1 Channel A (Battery Charger) Duty Cycle intially set to 100%
  OCR1B = 255;  // Timer 1 Channel B (48V Bus to Grid) Duty Cycle intially set to 100%

  // Intialize Time of Day Clock Parameters

  hr = 0;			// Intially set hours at zero
  min = 0;		// Intially set minutes at zero
  sec = 0;		// Intially set seconds at zero
  T0 = millis();	// Intialize milliseconds


  // Initialize other variables

 
  Bat_OV = analogReadAvg(1, 5);
  Bat_OV = mapf(Bat_OV, ADC_min, ADC_max, BV_min, BV_max); 
  Ch_V = Bat_Float;
  PV_overload = 0;
  PV_underload = 0;
  PV_OL_Counter = 0;
  PV_Zcount = 0;
  PV_Update_timer = 0;
  tmp = 0;
  Ch_Cycle = 1; 
  Regulator_ON = 0;
  Power_Now = 0;
  B_TrickleS = int((Bat_Float / BV_max) * 1023);
  PV_FastFlag = 1;
  MPPT_Sel = 255;
  Lo_Bat_flag=1;
  Hi_Bat_flag= 0;
  PV_itisday =1;
  Bat_I_Last= 0;
  Bat_Pwr = 0;
  PV_O_counter =0;
  OCR1A_Peak =0;
  Bat_zero_count=0;
 
  BatteryDischargeTime(true);
  PV_V= analogReadAvg(0, 5);
  PV_V = mapf( PV_V, ADC_min, ADC_max, PV_min, PV_max); // Scale PV Array Voltage to 0 to 45.3 V
  if(PV_V >  PV_day) 
		   {
			 PV_itisday =1;
		   }
	   else
		   {
			 PV_itisday =0;
		   }  
}



void loop()
{
  // loop() main function is dedicated to Power Management of MicroGrid

  //Serial.print(" Low bus sensing  low is low bus=");// test only
  //Serial.println( digitalRead(10));// test only
  //Serial.print(" PV OCR2A=");// test only
  //Serial.println( OCR2A);// test only
  //Serial.print(" PV_V=");// test only
  //Serial.println(PV_V   );// test only
  //Serial.print(" PV_I=");// test only
  //Serial.println( PV_I   );// test only
  //Serial.print(" MPPT_Sel=");// test only
  //Serial.println(MPPT_Sel);// test only
  //Serial.print(" Power_Now=");// test only
  //Serial.println(Power_Now);// test only
  //Serial.print(" pv overload =");// test only
  //Serial.println(PV_overload );// test only
  //Serial.print(" pv underload =");// test only
  //Serial.println(PV_underload );// test only
  //Serial.print(" charge voltage ch_v=");// test only
  //Serial.println(Ch_V );// test only  
  //Serial.print(" Bat_OV =");// test only
  //Serial.println( Bat_OV  );// test only
  //Serial.print(" Bat_V_Read= ");// test only
  //Serial.println( Bat_V_Read );// test only
  //Serial.print(" battery charge current=");// test only
  //Serial.println(  Bat_I_Read  );// test only 
  //Serial.print(" BusV_low =");// test only
  //Serial.println( BusV_low );// test only
  //Serial.print("fast pv search with flag=");// test only
  //Serial.println(PV_FastFlag);// test only
  //Serial.print("battery enabled ==");// test only
  //Serial.println(digitalRead(4));// test only
  //Serial.print("battery analog in  ==");// test only
  //Serial.println(analogRead(1));// test only
  //Serial.print("Sleep ==");// test only
  //Serial.println(Sleep);// test only
  //Serial.print("Shed==");// test only
  //Serial.println(Shed);// test only
  //Serial.print("PV_itisday==");// test only
  //Serial.println(PV_itisday);// test only
  //Serial.print("bus V high =1    low =0 ==");// test only
  //Serial.println(digitalRead(3));// test only  
  //Serial.print("battery to bus converter ==");// test only
  //Serial.println(digitalRead(7));// test only
  //Serial.print("bus low ===");// test only
  //Serial.println(digitalRead(3));//
   
	
	//if (digitalRead(3)== LOW) //remove after testing.. this is to measure the cycle time of the loop
	//      {
	//        digitalWrite(3, HIGH);
	//      }
	//    else
	//      {
	//        digitalWrite(3, LOW);
	//      }
	 
	
 
 if (digitalRead(6) == LOW && digitalRead(4) == HIGH)    // this if for protection only... if  battery being charged,  bat to bus must be off
	{
	 if(digitalRead(7) == LOW)
	  {
		//Serial.print(" 7 violation...................................... 7 low with 6 low and 4 high"); //test only        
		digitalWrite ( 7, HIGH);
	  }
	}
TOD_clock();	// Keep Time of Day with 24 hour clock
  
  //  Get_CMD();    // Look For Diagnostic Command From Console
  
PV_power_management();   // PV Array Power Output Optimization "A"

Monitor ();
 
Battery_Management();  // Battery Management Function "B"

Battery_Charge_Voltage_Set_Point();  // "G"

Load_Enable();          // Load Enable Function "C"  

Power_To_Grid();    // Power to Grid function "H"
 
Shed_Load();  
 
BatteryDischargeTime(true);

LCD_Message();			// Look for request message from LCD Display panel

 
 //delay(Loop_Delay);        // enable if you want to slow things down for debugging
}



// Determine the Battery Trickle Voltage Seppoint

//void Trickle_Charge_SP()
//{
//  if (( Float_Flag == true) && (digitalRead (7) == HIGH))
//  {
//    Bat_V_Read = analogReadAvg(1, 5);
//    if( Bat_Float > (Bat_V_Read+1))
//        {
//          B_TrickleS = ((Bat_V_Read+1) / BV_max * 1023);        
//        }
//       else
//       {
//          B_TrickleS = (Bat_Float / BV_max) * 1023; // modify this to take into account temperature
//       }
//
//    if (Bat_V_Read > B_TrickleS)
//    {
//      if (OCR1B == 255)
//      {
//        OCR1B = 255;
//      }
//      else
//      {
//        OCR1B = OCR1B + 1;
//     }
//    }
//    else
//    {
//      if (B_TrickleS > Bat_V_Read)
//      {
//        if (OCR1B == 1)
//        {
//          OCR1B = 1;
//          return;
//        }
//        else
//        {
//          OCR1B =  OCR1B - 1;
//        }
//      }
//    }
//  }
//  return;
//}





// PV Array Power Output Optimization Function

void PV_power_management()
{
  PV_V_last = PV_V; // Put the last PV voltage reading in PV_V_last
  PV_V = mapf(analogRead(0), ADC_min, ADC_max, PV_min, PV_max); // Scale PV Array Voltage to 0 to 45.3 V
  PV_I = mapf(analogRead(4), ADC_min, ADC_max, PVI_min, PVI_max); // Scale PV current to 0 top 11 amps  

  if( PV_V >  PV_day) 
		   {
			 Day_counter= Day_counter+1;
		   }

  if( Day_counter > 10)
		{     
		  PV_itisday =1;
		  Day_counter =0;
		  Night_counter=0;
		} 
  if(( PV_V  <  PV_day)  && (  PV_I < 0.1   ))
		{
		  Night_counter = Night_counter  + 1;
		}

  if( Night_counter > 5500)
		{     
			PV_itisday =0; // it is night
			Night_counter=0;
			Day_counter =0;
			PV_latch=0;
		}     
  if ( PV_itisday ==1)   // it is day, enable the PV convertor
		  {
			if (digitalRead(2) == LOW)  // if the PV Convertor is running
			  {
				Power_Now = PV_V * PV_I;
				MPPT_Last = OCR2A;    // Get current Duty Cycle From Timer register 2
				PV_Find_Knee ();                                  
			  }
			else  // turn on PV converter... there is sunlight and set initial parameters
			  {                              
			   digitalWrite(2, LOW);   //Turn PV DC DC conveter on
			   if(  PV_latch==0)
				   {
					PV_latch=1; 
					Power_Last = 0;
					Power_Now = PV_V * PV_I;
					MPPT_Last = OCR2A;        //// Get current Duty Cycle From Timer register 2A
					Power_1 = 0;
					Power_2 = 0;
					Power_3 = 0;
					PV_FastFlag = 0; //   This is a marker for the fast search function that is run on startup
					MPPT_1  = 2;  //   This is a marker for the fast search function that is run on startup
					MPPT_Sel = 0; //   This is a marker for the fast search function that is run on startup
					PVpwr_Av = 0;
				   }
				 else
				   {
					 if ( (digitalRead(7) == LOW) || (digitalRead(5) == LOW)) //PV plus battery or grid power are on 
						 {                 
						   OCR2A = 200; // set the first point to test
						 }
					   else
						 {                 
						   OCR2A = 100; // set the first point to test
						 }
				   }
			  }
		  }
		else // it is night
		  {
			digitalWrite(2,HIGH);   //turn off converter to save power
			PV_FastFlag = 1;// dont run Fast Flag at night
			return;
		  } 
		  
   if ( (digitalRead(7) == LOW) || (digitalRead(5) == LOW)) //PV plus battery or grid power are on 
	{          
	  Fast_PVSearch  ();
	  if (PV_FastFlag == 0) //looking for first setpoint
		  {
			return;
		  }
	  PV_Pwr_Max();

	  if (Regulator_ON == 1)
		   {
			  return;
		   }     
	  Power_D = Power_Now - Power_Last;
	  
	  if ( Power_Now > 0  &&  Power_Last>0 )
		  {               
			   if ( DELAY(3, 500) == false)// sets a delay time... the code executes  faster than pv panel response time
				  {
					return;
				  }
		  }
	  if ( Power_D >= 0 )
		  {
			if ((PV_V_last  > PV_V) && ( PV_V < (PV_ocV * 0.5))) // check if this really works  ... it is an attempt to keep the pv voltage near the knee
			  {
				OCR2A = OCR2A + 1;
				return;
			  }                                       
			OCR2A = OCR2A - 1;   // raise the reference to the DC DCconverter
			Power_Last = Power_Now;
			return;
		  }
	  Power_D = Power_Last - Power_Now;
	  if (Power_D > 0)
		  {
			OCR2A =  OCR2A + 1; //lower  the reference to the DC DCconverter if PV voltage colapsed
			Power_Last = Power_Now;
			return;
		  }
	}
   else // PV Only
	{
	   PV_FastFlag = 1;
	   if ( DELAY(3, 400) == false)// sets a delay time... the following regulator is faster than pv panel response time
		  {
			return;
		  }        
	  if ( (Power_Now == 0)  &&  (PV_V > (PV_ocV * 0.5)) )
		  {            
			OCR2A = OCR2A - 1;   // raise the reference to the DC DCconverter           
			Power_Last = Power_Now;
			return;
		  }
	  Power_D = Power_Now - Power_Last;
	  if ( (Power_D > 0))
		  {            
			OCR2A = OCR2A - 1;   // raise the reference to the DC DCconverter
			Power_Last = Power_Now;
			return;
		  }     
	  Power_D = Power_Last - Power_Now;
	  if (Power_D > 0)
		  {
			OCR2A =  OCR2A + 1; //lower  the reference to the DC DCconverter if PV voltage colapsed
			Power_Last = Power_Now;
			return;
		  }
	}
  return;
}




void Monitor ()
{
  if (digitalRead(2) == LOW)
  {
	if ( ( Power_Now == 0 )  &&  (  PV_V < 8.000 ) )
		  {
			PV_Zcount = PV_Zcount + 1;
		  }
		else
		  {
			PV_Zcount = 0;
		  }
	if ( PV_Zcount > 20  )
		  {
			PV_Zcount = 0;
			Regulator_ON = 0;
			PV_FastFlag = 0; // this turns on the fast search when the zero power count exceeds the specified value
		  }
	if (   PV_V < 10.000  )//  this counts the cycles where pV is overloaded
		{
			PV_Ocount = PV_Ocount + 1;
		  }
		 else
		  {
			PV_Ocount = 0;
		  }
	if ((PV_V > 10.000 )&& (PV_overload == 1))
			{
			  PV_Ncount=PV_Ncount+1;  // counts number of times PV voltage is not under...for reset
			}
		  else
			{
			  PV_Ncount = 0;
			}
	if ( PV_Ncount > 20 ) 
			{
			  PV_overload = 0;
			  PV_Ncount = 0;
			}
	if (( PV_Ocount > 15 ) && (MPPT_Sel |= 255))
		  {
			PV_overload = 1;
			PV_underload = 0;
			PV_Ocount = 0;
			Regulator_ON = 0;
			PV_O_counter = PV_O_counter +1;// when PV_O_count exceeds 50,  PV is used to augment battery for 10 min then reset
		  }
	if ( ( Power_Now > 0 )  &&  (  (PV_V > (PV_ocV * 0.97))))
		  {
			PV_Ucount = PV_Ucount + 1;
		  }
		 else
		  {
			PV_Ucount = 0;
		  }
	if ( PV_Ucount > 50 )
		  {
			PV_underload = 1; // PV underload is when the PV array is at max voltage... PVregulator probably not working and bus is probably at PV Max
			PV_Ucount = 0;
			PV_overload = 0;
			Regulator_ON = 0;
		  }
	if ((PV_V > 10.000) &&  (PV_V < (PV_ocV * 0.97))) // if the PV Convertor is running  and pv voltage is between these values system is working correctly
		  {
			PV_overload = 0;
			PV_underload = 0;
			PV_Ucount = 0;
			PV_Ocount = 0;
			PV_Zcount = 0;     
		  }
	 if ( (PV_O_counter >= 50) && (DELAY(8, 600000) == true))
				{
				  PV_O_counter =0;
				}
	  } 
  else
	  {
			PV_overload = 0;
			PV_underload = 0;
			PV_Ucount = 0;
			PV_Ocount = 0;
			PV_Zcount = 0;
			PV_O_counter =0;
	  }
				 
	if ((digitalRead(6) == LOW)  &&  (digitalRead(4) == HIGH)) // if the battery is being charged
		  {
			if (digitalRead(2) == LOW)
				  {
					if (( (PV_V - Bat_V_Read) < 2.000)  && ( OCR1A == 0)) // and if the PV voltage is essentially the battery charge voltage  with the ref at the limit
						  { // then the PV supplied voltage is not at the 48 volt level
							BusV_low = 1;
							Regulator_ON = 0;
							return;
						  }
					if ( (PV_V < Bat_V_Read)   && ( OCR1A == 1)) // and if the PV voltage is essentially the battery charge voltage  with the ref at the limit
						  {
							BusV_low = 0;
						  }        
				  }
			if (digitalRead(5) == LOW) // grid to bus is enabled
				  {
				   if ((( V_DC - Bat_V_Read) < 2.000)  && ( OCR1A == 0)) // and if the Grid  voltage is essentially the battery charge voltage  with the ref at the limit
						{ // then the PV supplied voltage is not at the 48 volt level
						  BusV_low = 1;
						  Regulator_ON = 0;
						  return;
						}  
				  }
				  else
				  {
					 if (( V_DC  < Bat_V_Read)   && ( OCR1A == 0)) // and if the Grid  voltage is less than the battery charge voltage  the bus voltage is not low
					 {
					  BusV_low = 0;       
					 }
				  }
		 }
  return;
}





void Fast_PVSearch  ()
{ // test to insure that this routine will work
  //  MPPT_2 is the last highest reference setpoint
  //  MPPT_3 is the last lowest reference setpoint
  //  MPPT_1 is the setpoint from the prior run
  //  MPPT_Last is the setpoint from the last run

  if (PV_FastFlag == 1) //already have found the first setpoint
	  {
		return;
	  }
  if (DELAY(9, 30000) == true)  // if cannot find the setpoint in xx seconds, abort
	  {
		PV_FastFlag = 1;
		return;
	  }

  if ((digitalRead(5) == LOW) || ( digitalRead (7) == LOW ))// run this only when there is battery power or grid power providing the internal bus voltage
	  {
		if ( DELAY(3, 400) == false)// sets a delay time... the following regulator is faster than pv panel response time
			{
			  return;
			}
		MPPT_1 = OCR2A;
		if ( OCR2A == 255)
			{
			  MPPT_2 = 255;
			  Power_2 = Power_Now;
			  OCR2A = 1; // if 255 result is ok,  test 1
			  return;
			}
		if ( OCR2A == 1)
			{
			  MPPT_3 = 1;
			  Power_3 = Power_Now;
			  OCR2A = 125;
			  return;
			}
		tmp =    MPPT_2 - MPPT_3;
		if ( 10 > tmp  )  // knee is known when the power > 0 and the spread in the readings is 10 or less
			{
			  OCR2A = MPPT_2;
			  MPPT_Sel = MPPT_2;
			  PV_FastFlag = 1;
			  return;
			}
//      if (Power_Now == 0  && Power_2 == 0  && Power_3 == 0)
//            {
//              MPPT_Sel = MPPT_2;
//              PV_FastFlag = 1;
//              return;
//            }
		if (Power_Now == 0  && Power_2 > 0)
			{
			  OCR2A = ( MPPT_1 + MPPT_2) / 2; //if the last reading gave a 0 current,  raise the setpoint
			  MPPT_3 = MPPT_1; // put the last reading into the new high... did not provide a positive current
			  Power_3 = Power_Now;
			  return;
			}
		if (Power_Now == 0  &&  Power_3 > 0)
			{
			  OCR2A = ( MPPT_1 + MPPT_3) / 2; //if the last reading gave a 0 current,  lower the setpoint
			  MPPT_2 = MPPT_1; // put the last reading into the new high... did not provide a positive current
			  Power_2 = Power_Now;
			  return;
			}
		if (Power_Now < Power_2 )
			{
			  OCR2A = ( MPPT_1 + MPPT_2) / 2; //if the last reading gave a 0 current,  lower the reading
			  MPPT_3 = MPPT_1; // put the last reading into the new high... did not provide a positive current
			  Power_3 = Power_Now;
			  return;
			}
		if (Power_Now < Power_3)
			{
			  OCR2A = ( MPPT_1 + MPPT_3) / 2; //if the last reading gave a 0 current,  lower the reading
			  MPPT_2 = MPPT_1; // put the last reading into the new high... did not provide a positive current
			  Power_2 = Power_Now;
			  return;
			}
		if ( Power_Now >= Power_2  && Power_Now >= Power_3)
			{
			  OCR2A = ( MPPT_1 + MPPT_3) / 2; //if the last reading gave a 0 current,  lower the reading
			  MPPT_3 = MPPT_1; // put the last reading into the new high... did not provide a positive current
			  Power_3 = Power_Now;
			  return;
			}
		if (  Power_2 == 0 &&  Power_3 == 0  &&  Power_1 == 0 )
			{
			  PV_FastFlag = 1; // search has failed if we get to 0,1,254 or 255
			  MPPT_Sel = 255;
			  return;
			}
		PV_FastFlag = 1; // search has failed if we get to 0,1,254 or 255
		if ( (digitalRead(7) == HIGH) && (digitalRead(5) == HIGH)) //if battery or grid power not on, turn off regulator
			  {      
				MPPT_Sel = 255;
			  }
			else
			  {
				MPPT_Sel = 2;
			  }
	}
  else
  {
	PV_FastFlag = 1;
	return;
  }
}







void PV_Find_Knee()
{
  if (  (OCR2A == 255) ||  (OCR2A == 1) ||  (OCR2A == 0) ||  (OCR2A == 254))
	  {
		MPPT_Sel = 255;
		Power_1 = 0;
		Power_2 = 0;
		Power_3 = 0;
	  }
  if ((Power_Now > 0) && (PV_V > 10000) && ((PV_ocV * 0.95) < PV_V))
	  {
		if (Power_1 == 0)
			{
			  Power_1 = Power_Now;
			  MPPT_1 = OCR2A;
			  PV_Volt1 = PV_V;
			  return;
			}
		else
			{
			  if ((Power_2 == 0) &&  ((MPPT_1 - OCR2A) == 1) && (MPPT_1 > OCR2A ) )
			  {
				Power_2 = Power_Now;
				MPPT_2 = OCR2A;
				PV_Volt2 = PV_V;
				return;
			  }
			}
	  }
  if (Power_2 > 0)
  {
	if ( Power_1 > Power_2)
	{
	  MPPT_Sel = MPPT_1;
	  Power_1 = 0;
	  Power_2 = 0;
	  Power_3 = 0;
	  return;
	}
	else
	{
	  MPPT_Sel = MPPT_2;
	  Power_1 = 0;
	  Power_2 = 0;
	  Power_3 = 0;
	  return;
	}
  }
  return;
}


void PV_Pwr_Max()
{
  if (PV_overload == 1 || PV_underload == 1 )
  {
	Regulator_ON = 0;
	return;
  }
  if (DELAY(7, 1000) == false)
  {
	return;
  }
  if ( (digitalRead(7) == HIGH) && (digitalRead(5) == HIGH)) //if battery or grid power not on, turn off regulator
  {
	Regulator_ON = 0;
	return;
  }
  if ( (digitalRead(7) == LOW) || (digitalRead(5) == LOW)) //run regulator if battery or grid power available
  {
	if (Regulator_ON == 0) //  regulator is off & all MPPTs not set
	{
	  if ((Power_Now > 0) && (PV_V > 10.000) && ((PV_ocV * 0.95) < PV_V)) // if power is positive, start regulator function
	  {
		Regulator_ON = 1; // turn on regulator function
		Regulator_timer = millis();
		Power_1 = Power_Now; // store the power level for this first point
		MPPT_1 = OCR2A;     /// store the setpoint for this power level
		MPPT_2 = MPPT_1 - 1;
		MPPT_3 = MPPT_1 - 2;
		OCR2A = MPPT_2;  // set the new setpoint at the old setpoint minus 1
		return;
	  }
	}
	else
	{
	  if (Power_3 == 0 )  // when power 3=0 , need to check the last MPPT setting.  MPPT_3
	  {
		if ( OCR2A == MPPT_2) // if OCR2A=MPPT_2,  then need to check MPPT_3 setpoint and power
		{
		  if ( Power_Now == 0)  // if MPPT_2 power =0, then exit this routine...
		  {
			Regulator_ON = 0; // exit subroutine and turn regulator flag off
			MPPT_1 = 0;     // store the setpoint for this power level
			MPPT_2 = 0;
			MPPT_3 = 0;
			Power_3 = 0;
			return;
		  }

		  Power_2 = Power_Now; // put power with MPPT_T setpoint into Power_2
		  OCR2A = MPPT_3; //  set OCR2A to MPPT_3 to test power at this setpoint
		  return;
		}
		if ( OCR2A == MPPT_3)
		{
		  Power_3 = Power_Now;
		  if (  Power_1 >  Power_2)
		  {
			MPPT_2 = MPPT_1;
		  }
		  if ( (Power_2 >=  Power_1) && (Power_2 >=  Power_3)) // pick the best setpoint or the 2 setpoints to move between
		  {
			MPPT_1 = MPPT_2;
		  }
		  if ( Power_3 >  Power_2)
		  {
			MPPT_1 = MPPT_2;
			MPPT_2 = MPPT_3;
		  }
		}
	  }
	  if (OCR2A == MPPT_1)
	  {
		OCR2A = MPPT_2;
		if ( Power_Now == 0 )
		{
		  Regulator_ON == 0;
		  Power_3 = 0;
		  OCR2A = OCR2A + 3;
		  return;
		}
		return;
	  }
	  else
	  {
		OCR2A = MPPT_1;
		if ( Power_Now == 0 )
		{
		  Regulator_ON = 0;
		  Power_3 = 0;
		  OCR2A = OCR2A + 3;
		  return;
		}

		return;
	  }
	}
  }
  else
  {
	Regulator_ON = 0;
  }
  return;
}


// Function that averages analog readings over a number of readings

float analogReadAvg(byte pin, byte number)

{
  int total = 0;
  int x;
  for (x = 0; x <= (number - 1); x++)
  {
	total = total + analogRead(pin);  // sum of readings for "number" of readings
	delay(1);
  }
  total = total / number;
  return (total);
}

// Map analog inputs to scaled floating point values
// x = analog reading from Arduino analogRead() function
// in_min = minimum reading from Arduino analogRead() function
// in_max = maximum reading from Arduino analogRead() function
// out_min = minimum analog scaling value
// out_max = maximum analog scaling value

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x  * ((out_max - out_min) / (in_max - in_min)) + out_min);
}

// Function that intializes a delay timer
// A Maximum of 10 delay timers can be initailized



byte DELAY(byte timer_num, unsigned long d_time)    // Up to 10 timers supported, D-time is in seconds
{
  if (Timer_Set[timer_num - 1] == false)
  {
	Timer_Set[timer_num - 1] = true;    // Set Timer to ON
	Timer_T0[timer_num - 1] = millis();  // Set starting time T0

  }
  if (d_time > (millis() - Timer_T0[timer_num - 1]))
  {
	return (false); // Timer has not timed out!
  }
  else
  {
	Timer_Set[timer_num - 1] = false;  //Reset timer
	return (true);	// Timer has timed out
  }
}





// Function to read Battery Open Circuit Voltage
void read_OCV()
{ // Delay 15 seconds for voltage to settle set bu ocv_time
  
  
  if ( Ch_Cycle == 1)
  {
	return;  // charging, not reading voltage
  }
  digitalWrite(4, LOW);   // turn off battery charger
  digitalWrite(7,HIGH);  // ensure battery to bus stays off
  if (OVC == 0)
  {
	OVC = 1;  // OVC to skip if already done once on call
	Timer_OVC = millis();  // Set starting time T0
	digitalWrite(4, LOW);   // turn off batter charger
	digitalWrite(7,HIGH);  // ensure battery to bus stays off
  }
  if (ocv_time < (millis() - Timer_OVC))
	  {
		Bat_OV = analogReadAvg(1, 5);
		Bat_OV = mapf(Bat_OV, ADC_min, ADC_max, BV_min, BV_max); 
		if (Bat_OV > Up_BOV ) // Check operating limits of battery
			  {
				Ch_V = Bat_Float - (Bat_Ch_Adj * Bat_Temp);   // Set battery float voltage
			  }
			else
			  {
				Ch_V = Bat_Charge - (Bat_Ch_Adj * Bat_Temp);      // Set Battery charge voltage
			  }
		digitalWrite (4, HIGH); // Turn Charger back on
		digitalWrite(7,HIGH);  // ensure battery to bus stays off
		Ch_Cycle = 1;
		OVC = 0;
		OCR1A = OCR1A + 10;
		return;  // Timer has timed out
	  }
	else
	  {
		return;
	  }
  return;
}



void TOD_clock()
{
  // Check for millis() rollover every 49 days (0xFFFFFFFF milliseconds)
  if (millis() < T0)	{
	T0 = millis();			// May loose 1 sec every 49 days!
  }
  if ((millis() - T0) >= 1200)	{		// 20 Millisec bias added to clock
	T0 = T0 + 1000;	// Update T0
	sec = sec + 1;	// 1000 milliseconds equals 1 second
  }
  if (sec == 60)	{
	sec = 0;	// reset seconds to zero
	min = min + 1;	// 60 seconds equals 1 minute
  }
  if (min == 60)	{
	min = 0;		// reset minutes to zero
	hr = hr + 1;	// 60 minutes = 1 hour
  }
  if (hr == 24)	{
	hr = 0;			// Roll clock over to mid night
  }
  return;
}




// Battery Management Function
void Battery_Management()
{ 
  if (digitalRead(6) == LOW && digitalRead(4) == HIGH)    // this if for protection only... if  battery being charged,  bat to bus must be off
	{
	  digitalWrite ( 7, HIGH);
	}
   if( (Bat_OV < (Lo_BOV-0.150)) && ( Bat_OV > 1.00))  // shed if batt is low and there is a battery
			{
			 Shed=1;
			}    
  if( Bat_OV >= (Lo_BOV + 0.150))  // provides hysterysis
			{
			 Shed=0;
			} 
  if ( PV_itisday ==1)    // is it day time?   
	{    
	  if (PV_FastFlag == 0) //looking for first setpoint
		  {
			return;
		  }   
	  if (PV_overload == 1 )
		  {
			return;
		  }   
	} 
  Bat_Temp = analogReadAvg(5, 5);
  Bat_Temp = mapf(Bat_Temp, ADC_min, ADC_max, BT_min, BT_max);
  V_DC = analogReadAvg(2, 5);
  V_DC = map(V_DC, ADC_min, ADC_max, GV_min, GV_max);

  if (Bat_Temp >= 200)  // if battery temp >= 200 then sensor is broken so set bat temp to zero
	  {
		Bat_Temp = 0;
	  }
  if (Bat_Temp > Bat_Temp_Hi) // Is Battery Temperature High ??
	  {
		digitalWrite(4, LOW);   // Turn off battery charger
		digitalWrite(7, HIGH);   // turn off the battery to bus converter
		Float_Flag = 0;
		OCR1B = 255; // set trickle charge  voltage to minimum
		return;
	  }
  
  if (DELAY(2, 1200000) == true)   // timer for charging the battery
	  {
		if (  (digitalRead(4) == HIGH) && ( digitalRead (6) == LOW) ) //battery is charging,  read battery voltage with delay
		  {
			Ch_Cycle = 0;
			//OCR1B = 255; // set trickle charge  voltage to minimum
		  }
	  }
  read_OCV();

// Bat_Rem = (Bat_OV - Lo_BOV ) / (Up_BOV - Lo_BOV);
//  if (Bat_Rem > 1)
//      {
//        Bat_Rem = 1;
//      }
  if ((digitalRead(6) == HIGH) && (digitalRead(4) == HIGH))    // 
		{
		  Bat_OV = analogReadAvg(1, 5);
		  Bat_OV = mapf(Bat_OV, ADC_min, ADC_max, BV_min, BV_max); 
		}     
  if ( PV_itisday ==1)    // is it day time?
	  {
		Grid_Op =0;
		if (Bat_OV > Up_BOV) // this section provides hysterysis around Up_BOV
			  {
				Hi_Bat_flag= 1 ;
				Lo_Bat_flag=0; // battery is not low if it is high
				Sleep =0;
				Shed=0;
				Grid_Op =0;
				
			  }
		if (Bat_OV < (Up_BOV- 0.500)) 
			  {
				Hi_Bat_flag= 0 ;
			  }
						   
		if (Hi_Bat_flag==1)   // If the  battery is charged and PV is available ok TO SEND PV POWER TO GRID
			  {        
				  digitalWrite(5, HIGH);//sending power to grid... turn off grid to 48V convertor
				  digitalWrite(6, HIGH);//sending power to grid... assign relay to power grid
				  digitalWrite(7, HIGH);//sending power to grid from PV and putting battery in float mode, disable batt to bus
				  digitalWrite(4, HIGH);//sending power to grid... enable convertor to send power to the grid
				 // Float_Flag = 1;   // put battery in float charge
			  }
			else  // Battery not charged
			  {
				if((Bat_OV < (Lo_BOV-0.100))&&( Bat_OV > 10.000))
					  {
						Sleep =1;
					   
					  } 
				if( Bat_OV >( Lo_BOV+1.000))
					  {
						Lo_Bat_flag=0;
						Sleep =0;
					  }                
				if( Bat_OV < 1.00)  // there is no battery
					  {
						Lo_Bat_flag=0;
						Sleep =0;
					  }  
				if (V_DC > Grid_V) //there is grid power
				  {
					digitalWrite(5, LOW); // enable the grid to bus convertor
					digitalWrite(6, LOW); // converter assigned to battery .. this is insurance to prevent circulating current                            
					if (Ch_Cycle == 1) // and the battery voltage is not being read.. then charge the battery
					  {
						digitalWrite(7, HIGH); //disable the battery to bus convertor
						digitalWrite(6, LOW); // converter assigned to battery
						digitalWrite(4, HIGH); // enable battery charger convertor
						Float_Flag = 0; // disable the separate float charger circuit
						return;
					  }
					}
				if ( (digitalRead(2) == LOW) &&  (PV_O_counter < 50) )// is power available from PV and PV is not overloaded
					{
					  if (Ch_Cycle == 1) // and the battery voltage is not being read.. then charge the battery
						  {
							digitalWrite(7, HIGH); //disable the battery to bus convertor
							digitalWrite(6, LOW); // converter assigned to battery
							digitalWrite(4, HIGH); // enable battery charger convertor
							Float_Flag = 0; // disable the separate float charger circuit
						  }
					}   
				if ( (digitalRead(2) == LOW) &&  (PV_O_counter >= 50) )// this was an  else statement in pervious version
					  {
						digitalWrite(7, LOW); //enable the battery to bus convertor
						digitalWrite(6, LOW); // converter assigned to battery
						digitalWrite(4, LOW); // disable battery charger convertor
						Float_Flag = 0; // disable the separate float charger circuit
					  }
			  }
		return;
	  }
  else  //it is night
	  {
		if(digitalRead(6)==LOW)
		  {
			digitalWrite(4,LOW); // this insures that the converter is low before reading the battery voltage
		  }
		Bat_OV = analogReadAvg(1, 5);
		Bat_OV = mapf(Bat_OV, ADC_min, ADC_max, BV_min, BV_max); 
		if( Bat_OV < Lo_BOV)
			{
			  Lo_Bat_flag=1;
			  Hi_Bat_flag=0;  // if battery is low,  it cannot be high
			}
		if( Bat_OV >( Lo_BOV+1.000))
			{
			  Lo_Bat_flag=0;
			  Sleep =0;
			  Grid_Op =0;
			}        
		if ( Bat_OV > (( Lo_BOV+Up_BOV)/2))   // Is Battery charged above 50%, ok to send battery power to the grid
			{
			  digitalWrite(5, HIGH); // turn off grid to bus converter
			  digitalWrite(7, LOW); //battery to bus  converter on
			  digitalWrite(6, HIGH); // assign converter to the grid
			  digitalWrite(4, HIGH); // turn on the bus to grid convertor
			  Float_Flag = 0; // do not float charge.. send battery power to grid
			  
			}
		   else // battery is less than 50%
			{
			  if (V_DC > Grid_V) //there is grid power
				  {                    
					digitalWrite(6, LOW);   // Power not sent to grid and battery not charged, leave relay off
					digitalWrite(4, LOW);  // Power not sent to grid and battery not charged, do not charge at night convetor off
					digitalWrite (7, HIGH); //disable battery to bus convertor
					digitalWrite(5, LOW); // enable the grid to bus convertor
					Float_Flag = 0; // do not charge battery at night
					if(Bat_OV > Lo_BOV)
					  {
						Sleep =0;
						Grid_Op =0;
					  }
					 if((Bat_OV < (Lo_BOV-0.100))&&( Bat_OV > 10.000))// ok to run from grid at night as long as load does not drain battery
					  {
						Grid_Op =0;
					  }   

					 if((Bat_OV < (Lo_BOV-0.500))&&( Bat_OV > 10.000)) // go to sleep mode if battery drain continues even with grid power on
					  {
						Grid_Op =1;
						Sleep =1;
					  } 
				  }
			  else //no grid power
				  {
					
				   
					if (Lo_Bat_flag== 1)
					
						  {
						  digitalWrite(6, LOW);   // No power to grid & not charging battery..disable relay and save power
						  digitalWrite(4, LOW);  //  disable grid & battery charging
						  digitalWrite (7, HIGH); //disable battery to bus convertor
						  digitalWrite(5, HIGH); // disable the grid to bus convertor
						  Float_Flag = 0;    // do not charge battery at night
						  Sleep =1;
						  Grid_Op =0;
						  }
						else
						 {
						  digitalWrite(6, LOW);   // No power to grid & not charging battery..disable relay and save power
						  digitalWrite(4, LOW);  //  disable grid & battery charging
						  digitalWrite (7, LOW); //OK for  battery to bus convertor enabled
						  digitalWrite(5, HIGH); // disable the grid to bus convertor
						  Float_Flag = 0;    // do not charge battery at night                      
						 }
						  
				  }
			}
	  }
  return;
}








void  Power_To_Grid()
{ 
  if ((digitalRead(4) == HIGH)  &&  (digitalRead(6) == HIGH))   // power being sent to grid
  {
	 V_DC = analogReadAvg(2, 5);
	 V_DC = map(V_DC, ADC_min, ADC_max, GV_min, GV_max);
	 I_DC = analogReadAvg(3, 5);
	 I_DC = map(I_DC, ADC_min, ADC_max, GI_min, GI_max);
	 Bat_OV = analogReadAvg(1, 5);
	 Bat_OV = mapf(Bat_OV, ADC_min, ADC_max, BV_min, BV_max); 
	 Grid_Pout = V_DC * I_DC;
	
	 if ( PV_itisday ==0)          //it is night and power is coming from the battery
		{
		  Pwr_to_grid = (Bat_Rem * PWR_B * 1000); //Power to grid decreases as battery is discharged
		  if (Pwr_to_grid - Grid_Pout > 0)
			  {
				if (OCR1A == 0)
				  {
					OCR1A = 0;
				  }
				else
				  {
					OCR1A = OCR1A - 1;  // Timer 1 Channel A (Battery Charger/Gid Output) Duty Cycle intially set to 100%
				  }
			  }
		  else
			  {
				if (OCR1A == 255)
				  {
					OCR1A = 255;
				  }
				else
				  {
					OCR1A = OCR1A + 1;  // Timer 1 Channel A (Battery Charger/Gid Output) Duty Cycle intially set to 100%
				  }
			  }
		return;
	  }

	if ( PV_itisday == 1)          //it is day and power is coming from the pv array
	  {
		 if (PV_overload == 1 )
			  {
				OCR1A =254;
				return;
			  }
		if ( I_DC > I_DC_Peak)
			  {
				I_DC_Peak =  I_DC; 
				OCR1A_Peak =   OCR1A;
			  }                  
	   if(( I_DC > 5.000) && ( OCR1A != 255))
			{
			  
			  OCR1A = OCR1A + 1;
			  return;
			}
	  if((V_DC >= (Bat_OV - 0.500 )) && ( OCR1A != 255))
			{
			  OCR1A = OCR1A + 1;
			  return;
			}  
	  if (I_DC > 500)
			{
			   if ( DELAY(8, 2000) == false)// sets a delay time... the following regulator is faster than pv panel response time
				  {
					return;
				  } 
			}     
	  if( OCR1A < 6)
			{
			  OCR1A = 250;
			}
	   if ((I_DC_Last - I_DC)> (5.000 * 0.05))
			{
			  OCR1A = OCR1A +2;
			//  return;
			}      
	  if ((I_DC==0) && ( I_DC_Last > 0))
			{
			  OCR1A = OCR1A + 3;
			  return;
			}  
	 if ( ( OCR1A < OCR1A_Peak) && (I_DC==0)) // try this
		   {
			OCR1A = OCR1A_Peak  + 4;
			OCR1A_Peak =0;       
		   }           
	  if ( BusV_low == 1)
		{         
		  OCR1A = 250;//test only
		  return;
		  
		  if (I_DC_Last > I_DC)
			   {
				  OCR1A = OCR1A + 3;
				  return;
			   } 
		   if ( I_DC >= I_DC_Last)
				{            
				  OCR1A = OCR1A - 1;   // raise the reference to the DC DCconverter
				  return;
				}  
		}
	 OCR1A = OCR1A - 1;                       
  }
  return;
  }
}










// Load Shed Function

void Shed_Load()
{  
   if (PV_underload == 1)
	  {
	   if( MPPT_Sel != 255)
		   {
			digitalWrite(8, LOW);// 24 v converter
			digitalWrite(13, LOW);//48 volt output
			return;
		   }
	   else
		   {
			  digitalWrite(8, LOW);
			  digitalWrite(13, LOW);
			  PV_FastFlag =0;
			  return;
		   }
	   
	  } 
  if ( PV_overload == 1)
	  {                 
		 Power_Last = Power_Now;
		 OCR2A =  OCR2A + 4; 
		 Regulator_ON = 0;
		 digitalWrite(2, HIGH);   //Turn PV DC DC conveter off
		 return;
	  }
   if(Shed==1)
	 {
	  Pwr_to_grid = 0;    
	  digitalWrite(13, LOW);    // Turn off 48 volt bus
	  digitalWrite(12, HIGH);    // Turn off 12 volt bus
	 }
   if (  Sleep ==1 )
	  {
		 digitalWrite(12, HIGH); // disable 12 volt converter
		 digitalWrite(13, LOW); // disable the 48V output
		 if ( PV_itisday == 0) // if it is night
		 {
		   digitalWrite(2, HIGH);   //Turn PV DC DC conveter off
		 }
		 digitalWrite (7, HIGH); //disable battery to bus convertor
		 
		 if (V_DC > Grid_V) //there is grid power
			  {
				  digitalWrite(5, LOW); // enable the grid to bus convertor
			  }
			 else
			 {
				  digitalWrite(5, HIGH); // disable the grid to bus convertor
			 }
		 if (digitalRead(8) == HIGH ) // Has the 24V been on for > 15 Minutes
			{
			  if (DELAY(4, 900000) == true ) // Has the 24V been on for > 15 Minutes
			  {
				digitalWrite(8, LOW); // Turn off 24 Volts
				return;
			  }
			}
		 else
			{
			  if (DELAY(5, 2700000) == true) // Has the 24V been off for > 45 Minutes
			  {
				digitalWrite(8, HIGH); // Turn on 24 Volts
			  }
			}
		 if ( PV_itisday ==0)    //if it is night
			{   
			   digitalWrite(6, LOW);   // Power not sent to grid and battery not charged, leave relay off
			   digitalWrite(4, LOW);  // Power not sent to grid and battery not charged, do not charge at night convetor off             
			}
	  }

  if (  Grid_Op =1 )
	  {
	   //digitalWrite(12, LOW); // enable able 12 volt converter
	   //digitalWrite(13, HIGH); // enable  the 48V output
	   //digitalWrite(2, HIGH);   //Turn PV DC DC conveter off
	   //digitalWrite(6, LOW);   // Power not sent to grid and battery not charged, leave relay off
	   //digitalWrite(4, LOW);  // Power not sent to grid and battery not charged, do not charge at night convetor off
	   //digitalWrite (7, HIGH); //disable battery to bus convertor
	  }
   if ( BusV_low == 1)
		{
			// OCR1A = 255; // battery charging        
			digitalWrite(2, HIGH);   //Turn PV DC DC conveter off this was added march 4 for testing
			digitalWrite(8, LOW);
			digitalWrite(13, LOW);
			Pwr_to_grid = 0;                              
		}
  return;
}



// Load Enable Function

void Load_Enable()
{
  if ((PV_overload == 0) && (Sleep == 0))
  {
	PVpwr_Av = Power_Now * 0.5; //set PV to grid at the maximum of 50% of the PV power
	if ( Bat_OV > Lo_BOV)    //Enable load if there is no overload and battery power
		{
		  digitalWrite(8, HIGH);
		  digitalWrite(12, LOW);
		  digitalWrite(13, HIGH);
		  return;
		}
	if ( digitalRead(5) == LOW) //Enable load if there is no overload and grid power
		{
		  digitalWrite(8, HIGH);
		  digitalWrite(12, LOW);
		  digitalWrite(13, HIGH);
		}
	if ( digitalRead(2) == LOW) //Enable load if there is no overload and PV power
		{
		  digitalWrite(8, HIGH);
		  digitalWrite(12, LOW);
		  digitalWrite(13, HIGH);
		}                             
  }
  return;
}


// Battery Charge Voltage Set Point Management Function

void Battery_Charge_Voltage_Set_Point()
{
  // if (PV_overload == 1 )
  //   {
  //    OCR1A =254;
  //    return;
  //   }

  if (digitalRead(6) == LOW && digitalRead(4) == HIGH)    // Is battery being charged?
  {
	  if( Bat_OL ==0)
		  {     
			 if ( DELAY(12,200) == false)// sets a delay time... the following regulator is faster than pv panel response time
				{                       
				  return;
			   }          
			 Bat_V_Read_T = analogReadAvg(1, 5);
			 Bat_V_Read_T  = mapf(Bat_V_Read_T, ADC_min, ADC_max, BV_min, BV_max);

			 if (( Bat_I_Last > 0.05 )&&(Bat_V_Read > (Bat_V_Read_T)) && ((OCR1A_Last-1) == OCR1A))// this tries to catch an overload and resets to the previous setting
				   {                                                   
					 if (OCR1A < 250)
					 {                   
						OCR1A=OCR1A_Last+1;  // set all readings back to thier previous readings.
						Bat_V_Read= Bat_V_Read_Last;
						Bat_I_Read= Bat_I_Last; 
						Bat_OL =1;
						return;
					 }
				   }                        
		  }                
	  if ((Bat_I_Read > (Bat_MaxI*0.05)) && ( Bat_I_Read  > Bat_I_Last ))
		  {
			 if ( DELAY(8,4000) == false)// sets a delay time... the following regulator is faster than pv panel response time
			  {
				return;
			  } 
		  }   
			else
			  {
				if( Bat_I_Read > (Bat_MaxI*0.025))
				{
				  if ( DELAY(11,1800) == false)// sets a delay time... the following regulator is faster than pv panel response time
				  {                          
					return;
				  }
				  else
				  {
					 if ( DELAY(13,100) == false)// sets a delay time... the following regulator is faster than pv panel response time
				  {                          
					return;
				  }
				  }
			  }    
			} 
	Bat_V_Last_5=Bat_V_Last_4;
	Bat_V_Last_4=Bat_V_Last_3;
	Bat_V_Last_3=Bat_V_Last_2;
	Bat_V_Last_2= Bat_V_Read_Last;        
	Bat_V_Read_Last = Bat_V_Read;

	OCR1A_Last_4=OCR1A_Last_3;
	OCR1A_Last_3=OCR1A_Last_2;
	OCR1A_Last_2=OCR1A_Last_1;
	OCR1A_Last= OCR1A_Last_1;
	
	Bat_I_Last= Bat_I_Read; // Last battery charging charging power
	Bat_V_Read = analogReadAvg(1, 5);
	Bat_V_Read  = mapf(Bat_V_Read, ADC_min, ADC_max, BV_min, BV_max);
	Bat_I_Read = analogReadAvg(3, 30);
	Bat_I_Read = mapf(Bat_I_Read, ADC_min, ADC_max, GI_min, GI_max ); // // Scale charging  current to 0 top 11 amps
	Bat_Pwr = Bat_V_Read*Bat_I_Read ; // present battery charging power 

	if (( Bat_V_Read > Bat_Peak_V)&&(Bat_I_Read> ( Bat_MaxI* .05)))//using I peak for voltaghe peak in this version
		{
		  Bat_Peak_V= Bat_V_Read;
		  OCR1A_Peak =   OCR1A;
		}
	if (  Bat_I_Read  <  0.050)
			  {
				Bat_zero_count =  Bat_zero_count +1;  
			  }
 
	if ( ( Bat_I_Read > 0.045) &&  ( Bat_zero_count > 10) )
		{
		  Bat_Start_I = OCR1A;
		  Bat_zero_count=0;         
		}     
 
	if((Bat_I_Read > Bat_MaxI) && ( OCR1A != 255))
		  {           
			OCR1A_Last=OCR1A;
			OCR1A = OCR1A + 1;
			return;
		  } 
	if((Bat_V_Read > Ch_V) && ( OCR1A != 255))
		  {
			OCR1A_Last=OCR1A;
			OCR1A = OCR1A + 1;
			return;
		  } 
		 
	if( OCR1A < 5)
		  {
			OCR1A_Last=OCR1A;
			OCR1A = 254;
		  } 
   Bat_OL =0;  
		if ((Bat_I_Read==0) && ( Bat_I_Last > (Bat_MaxI*0.1)))
		  {   
			  OCR1A= OCR1A_Peak+5;
			  return;
		  }  
	 
if( (   Bat_V_Read  < Bat_Peak_V ) && (Bat_Start_I >( OCR1A + 5 ) ))
		{
		  OCR1A=Bat_Start_I;//try this
		  OCR1A_Last=Bat_Start_I +1;//try this
		  Bat_Peak_V=0;
}
  
	// if ((Bat_V_Read_Last >   Bat_V_Read  )&& (OCR1A_Peak < 250) && (OCR1A_Last < OCR1A)   ) 
	//            {
	//              OCR1A_Last=OCR1A;;
	//              OCR1A = OCR1A_Peak+2;
	//             Bat_Peak_V = 0;                                  
	//               return;
	//           }  
  
	 if ((Bat_V_Read_Last > Bat_V_Read)&& (OCR1A_Last > OCR1A)&& (Bat_I_Last >( Bat_I_Read+ 0.03)))// this will only run if the first version does not catch the drop
				 {
					if ( OCR1A <  240)
					   {
						OCR1A_Last=OCR1A;
						OCR1A = OCR1A + 5;

					   // OCR1A_Last=(((OCR1A +Bat_Start_I)/2)-1);//try this
					   // OCR1A = Bat_Start_I;
						
						return;
					   }
				   else 
					   { 
						OCR1A_Last=OCR1A;
						OCR1A = 255;
					   return;
					   }
				 }  
  //if(  ( ( OCR1A_Last_4- 4)== OCR1A) && (( Bat_V_Read -Bat_V_Last_5) <= 0.2))
	   //{
	   //   OCR1A_Last= 255;
	   //   OCR1A = 254;
	   //}            
  //    if ( Bat_I_Read < 0.05)
	   //   {            
	   //      OCR1A_Last=OCR1A;
	   //      OCR1A = OCR1A - 1;   // raise the reference to the DC DCconverter
	   //      return;
	   //    }               
	if ((Bat_V_Read > Bat_V_Read_Last ) && ( OCR1A_Last > OCR1A   ))
		{            
		  OCR1A_Last=OCR1A;
		  OCR1A = OCR1A - 1;   // raise the reference to the DC DCconverter
		  return;
		}  
	 OCR1A_Last=OCR1A;
	 OCR1A = OCR1A - 1;                       
  }
  return;
}


void BatteryDischargeTime(byte intializationFlag)
  
{
  
  if (intializationFlag == true)  // Check if intialization required
  {
	
	y1 = min;   // Set to current Minute
	x1 = mapf(analogRead(1), ADC_min, ADC_max, BV_min, BV_max);   // Read battery voltage
	numberOfMinutes = 0;    // Reset minute counter
	dischargeMinutes = 0;   // Reset number of minutes since discharge started
	return;
  }
  else if (min != y1)     // Check if 1 minute has passed
  {
	numberOfMinutes++;
	dischargeMinutes++;
	y1 = min;
	if (numberOfMinutes == TimeBetweenMeasurements)   
	{ 
	  y1 = min - TimeBetweenMeasurements;
	  y2 = min;
	  x2 = mapf(analogRead(1), ADC_min, ADC_max, BV_min, BV_max);   // Read battery voltage
	  if ((x2 - x1) == 0)
	  {
	   // Serial.println(" Battery Is Not Discharging");
	  }
	  else
	  {
		float slope = (y2 - y1) / (x2 - x1);    // Calculate slope of discharge curve
		float intercept = y2 - (slope * x2);    // Calculate intercept point
		float timeToCutOff = (slope * BatteryCutOffVoltage) + intercept;  // Calculate time till we reach the battery cutoff voltage
 
		
		
		
		if ((timeToCutOff - dischargeMinutes) > 60)
		{
		 // Serial.print("Discharge Time Remaing = ");
		 // Serial.print(int((timeToCutOff - dischargeMinutes) / 60));
		 // Serial.print(" Hours ");
		 // Serial.print(int(timeToCutOff - dischargeMinutes) % 60);
		 //  Serial.println(" minutes");

		}
	   else
		{
		 // Serial.print("Discharge Time Remaing = ");
		 // Serial.print(int(timeToCutOff - dischargeMinutes)); // Amount of discharge time remaining
		 // Serial.println(" minutes");
		}
		
	  }
	  y1 = min;
	  x1 = x2;
	  numberOfMinutes = 0;
	  return;
	}
	else
	{
	  return;
	}
	
  }
  else
  {
	return;
  }
}


// LCD Messaaging Function

void LCD_Message()
{
	char_count = 0;
	while (Serial.available() > 0)		//check if data has been received on serial portt
	{
		input_buffer[char_count] = Serial.read();	// Read data and store in buffer
		char_count++;
		if (input_buffer[char_count - 1] == '\r' || char_count > 17)			// Check for "Line Feed" which marks end of message
		{
			if (input_buffer[0] == 'U' && input_buffer[1] == 'U')
			{
				function_number = input_buffer[2];
				parameter_number = input_buffer[3];
				parameter_code = input_buffer[4];
				message_received = 0;			// Reset Message flag

				// Exuecute request
				if (function_number == 0)
				{
					return_value = mapf(analogRead(parameter_number), ADC_min, ADC_max, analog_scaling_values[parameter_number * 2], analog_scaling_values[(parameter_number * 2) + 1]);
				}
				else if (function_number == 1)
				{
					if ((parameter_code & 0x10) == 0)
					{
						return_value = digitalRead(parameter_number + digital_IO_offset[parameter_number]);
					}
					else
					{
						if ((parameter_code & 0x08) != 0)
						{
							digitalWrite((parameter_number + digital_IO_offset[parameter_number]), HIGH);
							return_value = digitalRead(parameter_number + digital_IO_offset[parameter_number]);					// Acknowledge Control Execution
						}
						else
						{
							digitalWrite((parameter_number + digital_IO_offset[parameter_number]), LOW);
							return_value = digitalRead(parameter_number + digital_IO_offset[parameter_number]);						// Acknowledge Control Execution
						}
					}

				}
				else if (function_number == 2)
				{
					if ((parameter_code & 0x01) == 0)		// Zero data bytes send
					{
						return_value = *analog_scaling[parameter_number];	//Return Analog Scaling Factor requested by dispaly
					}
					else
					{
						parameter_code = parameter_code & 0x01;		// Data Bytes Sent
						return_value = ASCII_to_float();
						*analog_scaling[parameter_number] = return_value;
						return_value = *analog_scaling[parameter_number];	//Return Analog Scaling Factor requested by dispaly

					}
				}
				else if (function_number == 3)
				{
					if ((parameter_code & 0x01) == 0)		// Zero data bytes send
					{
						return_value = *setpoint[parameter_number];	//Return Analog Scaling Factor requested by dispaly

					}
					else
					{
						parameter_code = parameter_code & 0x01;		// Data bytes sent
						return_value = ASCII_to_float();
						*setpoint[parameter_number] = return_value;
						return_value = *setpoint[parameter_number];	//Return Analog Scaling Factor requested by dispaly

					}
				}
				else if (function_number == 4)
				{
					return_value = *calc_analog[parameter_number];
				}
				else if (function_number == 5)
				{
					if ((parameter_code & 0x10) == 0)
					{
						return_value = *system_digital[parameter_number];
					}
					else
					{
						if ((parameter_code & 0x08) != 0)
						{
							*system_digital[parameter_number] = HIGH;
							return_value = *system_digital[parameter_number];					// Acknowledge Control Execution
						}
						else
						{
							*system_digital[parameter_number] = LOW;
							return_value = *system_digital[parameter_number];						// Acknowledge Control Execution
						}
					}

				}
				else
				{
					return_value = 0xFFFF;	// Invalid message received
					
				}
			}
			else
			{
				return_value = 0xFFFF;			// Invalid Message Recieved
				
			}
			if (return_value != 0xFFFF)
			{
				Serial.print('U');		// Send data to display module
				Serial.print('U');
				Serial.println(return_value, 3);
			}
			
			return;					// Valid message recieved

		}
	}
	return;
}



// ASCII to Float Conversion Method
float ASCII_to_float()
{
	float data;
	char_pointer = 5; // use char_pointer as string index
	data_string = "";
	while (input_buffer[char_pointer] != '\r')
	{
		data_string += (char)input_buffer[char_pointer]; // Add char's from input_buffer to print_string
		char_pointer++;
	}
	data = data_string.toFloat();
	return(data);
}


