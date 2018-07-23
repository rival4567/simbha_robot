

 /*End of auto generated code by Atmel studio */
 static char global_m1a;
 static char global_m2a;
 static char global_m1b;
 static char global_m2b;

 static int global_counts_m1;
 static int global_counts_m2;

 static char global_error_m1;
 static char global_error_m2;

 static char global_last_m1a_val;
 static char global_last_m2a_val;
 static char global_last_m1b_val;
 static char global_last_m2b_val;
 static int8_t lookup_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
 static uint8_t enc1_val = 0;
 static uint8_t enc2_val = 0;
 
 #define Left_Encoder_PinA 19
 #define Left_Encoder_PinB 18
 #define Right_Encoder_PinA 25
 #define Right_Encoder_PinB 26


 void do_Right_Encoder()
 {
   
   enc1_val = enc1_val << 2;
   enc1_val = enc1_val | (PIOD->PIO_PDSR & 0b0011);
   
   global_counts_m1 = global_counts_m1 + lookup_table[enc1_val & 0b1111];
 }


 void do_Left_Encoder()
 {
   
   enc2_val = enc2_val << 2;
   enc2_val = enc2_val | ((PIOD->PIO_PDSR & 0b1100) >> 2) ;
   
   global_counts_m2 = global_counts_m2 + lookup_table[enc2_val & 0b1111];

 }

 
 void encoder_init()
 {
   noInterrupts();
   pinMode(Left_Encoder_PinA, INPUT_PULLUP);
   pinMode(Right_Encoder_PinA, INPUT_PULLUP);
   pinMode(Left_Encoder_PinB, INPUT_PULLUP);
   pinMode(Right_Encoder_PinB, INPUT_PULLUP);
   attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinA), do_Right_Encoder, CHANGE);
   attachInterrupt(digitalPinToInterrupt (Left_Encoder_PinA), do_Left_Encoder, CHANGE);
   attachInterrupt(digitalPinToInterrupt(Right_Encoder_PinB), do_Right_Encoder, CHANGE);
   attachInterrupt(digitalPinToInterrupt (Left_Encoder_PinB), do_Left_Encoder, CHANGE);


   // initialize the global state
   global_counts_m1 = 0;
   global_counts_m2 = 0;
   global_error_m1 = 0;
   global_error_m2 = 0;

   enc1_val = enc1_val | ((PIOD->PIO_PDSR & 0b0011));      //right encoder value
   enc2_val = enc2_val | ((PIOD->PIO_PDSR & 0b1100) >> 2 );  //left encoder value
   

   // enable interrupts
   interrupts();
 }
 
 int getCountsM1()
 {
  noInterrupts();
   int tmp = global_counts_m1;
  interrupts();
   return tmp;
 }

 int getCountsM2()
 {
  noInterrupts();
   int tmp = global_counts_m2;
   interrupts();
   return tmp;
 }

 int getCountsAndResetM1()
 {
   noInterrupts();
   int tmp = global_counts_m1;
   global_counts_m1 = 0;
    interrupts();
   return tmp;
 }

 int getCountsAndResetM2()
 {
   noInterrupts();
   int tmp = global_counts_m2;
   global_counts_m2 = 0;
   interrupts();
   return tmp;
 }

 unsigned checkErrorM1()
 {
   unsigned char tmp = global_error_m1;
   global_error_m1 = 0;
   return tmp;
 }

 unsigned char checkErrorM2()
 {
   unsigned char tmp = global_error_m2;
   global_error_m2 = 0;
   return tmp;
 }

 void setup()
 {
   encoder_init();
   Serial.begin(57600);
   while (!Serial) {
     ; // wait for Serial1 port to connect. Needed for native USB port only
   }
   
 }

 void loop()
 {
     Serial.print("EncoderA: ");
     Serial.println(global_counts_m1);

    Serial.print("EncoderB ");
    Serial.println(global_counts_m2);
    //Serial.print(PIOD->PIO_PDSR);

    delay(1000);

    //  Serial.print("PIC0: ");
    //  Serial.println((PINJ & (1<<0)),HEX);

    //    Serial.print("PIC1: ");
    //   Serial.println((PINJ & (1<<1)), HEX);


 }
