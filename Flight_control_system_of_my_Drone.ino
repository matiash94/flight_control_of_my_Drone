#include <Wire.h>
 
//Direccion I2C de la IMU
#define MPU 0x68
#define BMP 0x76//direccion I2C de BMP
#define Reg1 0xF4
#define Reg2 0xF5
#define Med1 0xF7
#define Med2 0xF8
#define Med3 0xF9
#define P1 1

//Ratios de conversion
#define A_R 4096.0 // 32768/8

#define G_R 131.0 // 32768/250
 
//Conversion de radianes a grados 180/PI
#define RAD_A_DEG = 57.295779
 
//MPU-6050 da los valores en enteros de 16 bits
//Valores RAW
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
unsigned long F7,F8,F9,P;//usados para el barometro
//unsigned int F8,F9;
 
//Angulos
float Acc[2];
float Gy[3];
float Angle[3];




float b = 0.99;  // constantes filtro complementario ........ con 0.999 tarda mucho en estabilizarse.. con 0.995 tarda 20 seg aprox
float c = 1-b;
float nn;//evita el problema NAN
/*
float d = 9.726*pow(10,-6);  //filtro 4to orden .. 1hz para aceleraciones
float e = 0.0001001;
float f = 9.377*pow(10,-5);
float g = 7.987*pow(10,-6);
float h = 3.672;
float i = 5.069;
float j = 3.117;
float k =0.7201;
float h1,h2,h3,h4,h5,h6,h7,h8;
float g1,g2,g3,g4,g5,g6,g7,g8;
float f1,f2,f3,f4,f5,f6,f7,f8;
*/

float d = 2.382*pow(10,-6);  //filtro 4to orden .. 0.7hz .. para aceleraciones
float e = 2.502*pow(10,-5);
float f = 2.39*pow(10,-5);
float g = 2.075*pow(10,-6);
float h = 3.77;
float i = 5.337;
float j = 3.361;
float k =0.7946;
float h1,h2,h3,h4,h5,h6,h7,h8;//variables para los filtros de las aceleraciones
float g1,g2,g3,g4,g5,g6,g7,g8;
float f1,f2,f3,f4,f5,f6,f7,f8;

//--------------------------------------------
float l = 0.0006889;//filtro 4do orden .. 3 HZ .. para veloc ang
float m = 0.006189;
float n = 0.005083;
float o = 0.0003815;
float p = 3.024;
float q = 3.521;
float r = 1.858;
float s = 0.3734;

/*
float l = 0.002034;//filtro 4do orden .. 4 HZ .. para veloc ang
float m = 0.01704;
float n = 0.01311;
float o = 0.0009248;
float p = 2.709;
float q = 2.896;
float r = 1.423;
float s = 0.2689;
*/

/*
float l = 0.004634;//filtro 4do orden .. 5 HZ .. para veloc ang
float m = 0.03616;
float n = 0.02605;
float o = 0.001731;
float p = 2.402;
float q = 2.361;
float r = 1.084;
float s = 0.1936;
*/

float c1,c2,c3,c4,c5,c6,c7,c8;//variables para los filtros de veloc. angulares
float d1,d2,d3,d4,d5,d6,d7,d8;
float e1,e2,e3,e4,e5,e6,e7,e8,e9;
//-------------------------------------------

String valores;

float AcTotal,AcTotal1,AcTotalf;//filtro del vector aceleracion
float AcX1,AcY1,AcZ1;//aceleraciones filtradas
float Gy0,Gy1,Gy2;//veloc angulares filtradas


 float dt, tiempo_prev;

 float a0, a1, a3;//guarda los valores iniciales de los angulos
 float Anglex, Angley, Wz;//guarda valores de los angulos medidos por IMU listos para el controlador

 float Wzr, Angxr, Angyr, Vz,DT1;//guarda valores de referencia

 float ex, ey, ez;//guarda errores (usados para los controladores)

 float Mx,Mx1,Mx2,ex1,ex2;//valres usados para la accion de control Mx
 float My,My1,My2,ey1,ey2;
 float Mz;
 float DT, Vz1, T,e33;//usados para calcular accion de control del empuje T
 float M=1050;//masa del drone
 float a = 0.01745329;   // conversion grados a radianes para cosenos y senos de e33

 float E1,E2,E3,E4;//guarda los valores de los empujes de cada motor
 float pwm_value1, pwm_value2, pwm_value3, pwm_value4;//guarda los pwm a aplicar en cada motor
 


 //---------------------------------------------------------------------------------------------------------------------------LECTURA DE PWM DEL RECEPTOR
//We create variables for the time width values of each PWM input signal
unsigned long counter_1, counter_2, counter_4, counter_5, current_count, current_count1;

//We create 4 variables to stopre the previous value of the input signal (if LOW or HIGH)
byte last_CH1_state, last_CH2_state, last_CH4_state, last_CH5_state;
byte lecture_THROTTLE = 2;


//To store the 1000us to 2000us value we create variables and store each channel
int input_YAW;      //In my case channel 5 of the receiver and pin D12 of arduino
int input_PITCH;    //In my case channel 2 of the receiver and pin D9 of arduino
int input_ROLL;     //In my case channel 1 of the receiver and pin D8 of arduino
int input_THROTTLE; //In my case channel 3 of the receiver and pin D2 of arduino
int input_AUX;      //In my case channel 6 of the receiver and pin D10 of arduino

#include <Servo.h>

 Servo esc1;//motor1
 Servo esc2;//motor2
 Servo esc3;//motor3
 Servo esc4;//motor4


void setup()
{
  //--------------------------------------------------------MPU9250
Wire.begin(); // D2(GPIO4)=SDA / D1(GPIO5)=SCL

//activa MPU
/*
Wire.beginTransmission(MPU);
Wire.write(0x6B);
Wire.write(0x00);
Wire.endTransmission(true);
*/
//config acelerometro
Wire.beginTransmission(MPU);
Wire.write(0x1C);
//Wire.write(0x10);//8g
Wire.write(0x10);//
Wire.endTransmission(true);
//Config giroscopo
Wire.beginTransmission(MPU);
Wire.write(0x1B);
Wire.write(0x00);//500 dps
Wire.endTransmission(true);
Serial.begin(115200);
/*
//config BMP280
Wire.beginTransmission(BMP);
Wire.write(Reg1);
Wire.write(23);
Wire.endTransmission(true);
delay(10);
Wire.beginTransmission(BMP);
Wire.write(Reg2);
Wire.write(144);
Wire.endTransmission(true);
*/




  //----------------------------------------------------------------------------------CONFIGURACION DE INTERRUPCIONES PARA LEER PWM DEL RECEPTOR
PCICR |= (1 << PCIE0);    //enable PCMSK0 scan                                                 
  PCMSK0 |= (1 << PCINT0);  //Set pin D8 trigger an interrupt on state change. 
  PCMSK0 |= (1 << PCINT1);  //Set pin D9 trigger an interrupt on state change.                                             
  //PCMSK0 |= (1 << PCINT2);  //Set pin D10 trigger an interrupt on state change.                                               
  PCMSK0 |= (1 << PCINT4);  //Set pin D12 trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT5);  //Set pin D13 trigger an interrupt on state change.  
                                                 
   pinMode(lecture_THROTTLE, INPUT);                                            
  
  esc1.attach(5);//motor1 pin5
  esc2.attach(3);//motor2 pin3
  esc3.attach(11);//motor3 pin11
  esc4.attach(6);//motor4 pin6

  esc1.writeMicroseconds(1000);//aplica el PWM mapeado anteriormente al motor
 esc2.writeMicroseconds(1000);//motor2
 esc3.writeMicroseconds(1000); //motor3
 esc4.writeMicroseconds(1000);//motor4
//--------------------------------------------------------------------------------------------------------------------------------
nn=1;
 delay(100);

}


void loop()
{
  /*
   Wire.beginTransmission(BMP);//barometro
   Wire.write(Med1);
   Wire.write(Med2);
   Wire.write(Med3);
   Wire.endTransmission(false);
   Wire.requestFrom(BMP,3,true);   //
   if(Wire.available()<=3) {
    F7=Wire.read();
    F8=Wire.read();
    F9=Wire.read();
    F7=F7<<16;
    F8=F8<<8;
    P=(F7+F8+F9);
    P=P>>4;
    */
   //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(MPU);
   Wire.write(0x3B); //Pedir el registro 0x3B - corresponde al AcX
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x3B, se piden 6 registros
   AcX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
   
/*
   AcZ1 = d*h1 + e*h2 + f*h3 + g*h4 + h*h5 - i*h6 + j*h7 - k*h8;
   h4=h3;
   h3=h2;
   h2=h1;
   h1=AcZ/A_R;

   h8=h7;
   h7=h6;
   h6=h5;
   h5=AcZ1;

   AcY1 = d*g1 + e*g2 + f*g3 + g*g4 + h*g5 - i*g6 + j*g7 - k*g8;
   g4=g3;
   g3=g2;
   g2=g1;
   g1=AcY/A_R;

   g8=g7;
   g7=g6;
   g6=g5;
   g5=AcY1;

   AcX1 = d*f1 + e*f2 + f*f3 + g*f4 + h*f5 - i*f6 + j*f7 - k*f8;
   f4=f3;
   f3=f2;
   f2=f1;
   f1=AcX/A_R;

   f8=f7;
   f7=f6;
   f6=f5;
   f5=AcX1;
   */
   
   /*AcZ1=0.007323*h1 + 0.006791*h2 + 1.783*h3 -0.7976*h4;  //frec de corte  Hz tiempo muestre seg   FILTRO 2do ORDEN
   h2=h1;
   h1=AcZ/A_R;
   h4=h3;
   h3=AcZ1;
   */

   /*
   AcZ1=(AcZ/A_R)*d + e*AcZ1; //FILTR PRIMER ORDEN
   AcX1=(AcX/A_R)*d + e*AcX1;
   AcY1=(AcY/A_R)*d + e*AcY1;
   */

   AcX1 = AcX/4096.0;
   AcY1 = AcY/4096.0;
   AcZ1 = AcZ/4096.0;

   AcTotal = sqrt((AcX1*AcX1)+(AcY1*AcY1)+(AcZ1*AcZ1));
   AcTotalf = 0.01249*AcTotal1 + 0.9875*AcTotalf;
   AcTotal1 = AcTotal;

   //A partir de los valores del acelerometro, se calculan los angulos Y, X
   //respectivamente, con la formula de la tangente.
   Acc[1] = atan(-1*(AcX1)/sqrt(pow((AcY1),2) + pow((AcZ1),2)))*RAD_TO_DEG;
   Acc[0] = atan((AcY1)/sqrt(pow((AcX1),2) + pow((AcZ1),2)))*RAD_TO_DEG;

   Wire.beginTransmission(MPU);
   Wire.write(0x43);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU,6,true);   //A partir del 0x43, se piden 6 registros
   GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
   GyY=Wire.read()<<8|Wire.read();
   GyZ=Wire.read()<<8|Wire.read();
 
   //Calculo del angulo del Giroscopio
   Gy[0] = GyX/G_R;
   Gy[1] = GyY/G_R;
   Gy[2] = GyZ/G_R;

   Gy0 = l*c1 + m*c2 + n*c3 + o*c4 + p*c5 - q*c6 + r*c7 - s*c8;
   c4=c3;
   c3=c2;
   c2=c1;
   c1=Gy[0];

   c8=c7;
   c7=c6;
   c6=c5;
   c5=Gy0;

   Gy1 = l*d1 + m*d2 + n*d3 + o*d4 + p*d5 - q*d6 + r*d7 - s*d8;
   d4=d3;
   d3=d2;
   d2=d1;
   d1=Gy[1];

   d8=d7;
   d7=d6;
   d6=d5;
   d5=Gy1;

  Gy2 = l*e1 + m*e2 + n*e3 + o*e4 + p*e5 - q*e6 + r*e7 - s*e8;
   e4=e3;
   e3=e2;
   e2=e1;
   e1=Gy[2];

   e8=e7;
   e7=e6;
   e6=e5;
   e5=Gy2;
   

   
   /*
   Gy0 = Gy[0]*d + e*Gy0;   //Filtro pasabajos
   Gy1 = Gy[1]*d + e*Gy1;
   Gy2 = Gy[2]*d + e*Gy2; 
   */

 dt = (millis() - tiempo_prev) / 1000.0;
   tiempo_prev = millis();
 
   //Aplicar el Filtro Complementario
   Angle[0] =  b*(Angle[0]+Gy0*dt)+c*Acc[0];
   Angle[1] =  b*(Angle[1]+Gy1*dt)+c*Acc[1];

   if(nn==1){   //evita que Angle[0] y Angle [1] tome el valor NAN.. el primer valor de Acc[0] y Acc[1] es NAN porque AcX1 AcY1 y AcZ1 empiezan en 0.. se produce division 0/0 en el calculo de Acc
    Angle[0]=0;
    Angle[1]=0;
    nn=nn+1;
   }

   e33 = (cos(Angle[0]*a)*cos(Angle[1]*a))/(pow(cos(Angle[0]*a),2)*pow(cos(Angle[1]*a),2) + pow(cos(Angle[0]*a),2)*pow(sin(Angle[1]*a),2) + pow(sin(Angle[0]*a),2)*pow(cos(Angle[1]*a),2) + pow(sin(Angle[0]*a),2)*pow(sin(Angle[1]*a),2));

  if(millis()<6000.0){
    a0=Angle[0];
    a1=Angle[1]; 
    a3=Gy2;
  }//Guarda los valores iniciales de los angulos
  

   Anglex=-(Angle[0]-a0)*(3.1415/180);//valores para la realimentacion del lazo cerrado (en rad), los signos negativos tanto en Angley como en Anglex se deben al sistema de referencia adoptado
   Angley=(Angle[1]-a1)*(3.1415/180);
   Wz=-(Gy2-a3)*(3.1415/180);//(veloc angular en rad/seg)

  input_THROTTLE =  pulseIn(lecture_THROTTLE, HIGH);

  Wzr=map(input_YAW, 1000, 2000, -1000, 1000);//referencia para veloc angular en z, varia entre -20 y 20 grados/seg--------REFERENCIAS
  Wzr=(Wzr/1000)*0.35;
  Angxr=map(input_PITCH, 1000, 2000, -1000, 1000);//ref para ang eje x, varia entre -20 y 20 grados=0.35 rad, stick izquierdo
  Angxr=(Angxr/1000)*0.35;
  Angyr=map(input_ROLL, 1000, 2000, -1000, 1000);//ref para ang eje y, varia entre -20 y 20 grados,stick derecho
  Angyr=(Angyr/1000)*0.35;
  Vz=map(input_THROTTLE, 990, 1990, -300, 300);//ref para veloc vertical, varia entre -0.3 y 0.3 metros/seg, stick potencia
  Vz=Vz/1000;
  DT1=map(input_AUX, 1200, 2000, 0, 100);// sirve para trimar el empuje de los motores, DT1 es un empuje adicional CONSTANTE que se puede sumar [gramos]
  DT1=(DT1/1000)*9.81;//empuje adicional en N

//ERRORES-----
ex=Angxr-Anglex;
ey=Angyr-Angley;
ez=Wzr-Wz;

//ACCIONES DE CONTROL
Mx=0.1402*ex - 0.2779*ex1 + 0.1378*ex2 + 1.899*Mx1 - 0.8988*Mx2;// momento en x
ex2=ex1;
ex1=ex;
Mx2=Mx1;
Mx1=Mx;

My=0.2231*ey - 0.4425*ey1 + 0.2194*ey2 + 1.899*My1 - 0.8988*My2;//momento en y
ey2=ey1;
ey1=ey;
My2=My1;
My1=My;

Mz=0.05*ez;//momento en z

DT=M*(Vz-Vz1)/dt;//accion de control delta T.. proporcional a la derivada de la veloc de ref --> MASA=1046 gramos
Vz1=Vz;
T=(M*9.81+DT+DT1)/e33;//accion de control T

//EMPUJE MOTORES----------
E1=-141.57*Mx+141.57*My-1070.33*Mz+25.48*T;//empuje motor 1
E2=141.57*Mx+141.57*My+1070.33*Mz+25.48*T;//motor2
E3=141.57*Mx-141.57*My-1070.33*Mz+25.48*T;//motor3
E4=-141.57*Mx-141.57*My+1070.33*Mz+25.48*T;//motor4

pwm_value1=map(E1, 12, 562, 1280, 1800);//mapea los empujes a valores PWM
pwm_value2=map(E2, 12, 562, 1280, 1800);
pwm_value3=map(E3, 12, 562, 1280, 1800);
pwm_value4=map(E4, 12, 562, 1280, 1800);


if(input_AUX <1050){
 esc1.writeMicroseconds(1000);//motor1
 esc2.writeMicroseconds(1000);//motor2
 esc3.writeMicroseconds(1000); //motor3
 esc4.writeMicroseconds(1000);//motor4 
  
}
else if(input_AUX >1050 && input_AUX <1100 ){
 esc1.writeMicroseconds(1400);//aplica el PWM mapeado anteriormente al motor
 esc2.writeMicroseconds(1400);//motor2
 esc3.writeMicroseconds(1400); //motor3
 esc4.writeMicroseconds(1400);//motor4
}

else if(input_AUX >1100 && input_AUX <1200 ){
 esc1.writeMicroseconds(1450);//aplica el PWM mapeado anteriormente al motor
 esc2.writeMicroseconds(1450);//motor2
 esc3.writeMicroseconds(1450); //motor3
 esc4.writeMicroseconds(1450);//motor4
}

else if(input_AUX >1200 ){
 esc1.writeMicroseconds(pwm_value1);//aplica el PWM mapeado anteriormente al motor
 esc2.writeMicroseconds(pwm_value2);//motor2
 esc3.writeMicroseconds(pwm_value3); //motor3
 esc4.writeMicroseconds(pwm_value4);//motor4
}
  
    
  
   
//Serial.println(P);
//valores = String(AcTotalf*100);
//valores= "4, " + String(AcTotal) + "," + String(AcTotalf) + ", 0";
//valores = String(dt);
    //Mostrar los valores por consola  ------------------------------------------------------------------------------------------------------------------------------------------------
  //valores = String(p);
//valores = "30, " + String(Angle[0]) + "," + String(Angle[1]) + ", -30";
 //valores = "1, " + String(AcX) + "," + String(AcY) + ","  + String(AcZ) + " ,-1";
 //valores = "600, " + String(E1) + "," + String(E2) + ","  + String(E3) + ","  + String(E4) + " ,-100";
 //valores = String(AcX1*10) + "," + String(AcY1*10) + ","  + String(AcZ1*10);
 //valores = "10, " + String(AcZ1*10) + "," +  String(h1*10);
 //valores = "10, " + String(AcZ1*10) + "," +  String(h1*10);
 //valores = String(Gy2) + "," +  String(Gy[2]);
 //valores = String(Gy1) + "," +  String(Gy[1]);
 //valores = String(Gy0) + "," +  String(Gy[0]);
  //valores = String(Gy0)  + "," + String(Gy1);
valores = String(Acc[0]);// + "," +  String(Acc[1]);
//valores = String(Angle0);
//valores=String(input_THROTTLE);
//valores=String();
   //valores = "90, " + String(Gy[0])  + "," + String(Gy[1])  + "," +  String(Gy[2]) + ", -90";  //Velocidad angular alrededor eje z
//valores = "25,"+  String(AzWorld3*100) + ", -25";          //+ String((AcX/A_R)*100) + String((AcY/A_R)*100) + " ,-0.5";
   Serial.println(valores);
   
   //delay(25);
   //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
}//FIN VOID LOOP




//This is the interruption routine
//----------------------------------------------

ISR(PCINT0_vect){
//First we take the current count value in micro seconds using the micros() function
  
  current_count = micros();
  ///////////////////////////////////////Channel 1
  if(PINB & B00000001){                              //We make an AND with the pin state register, We verify if pin 8 is HIGH???
    if(last_CH1_state == 0){                         //If the last state was 0, then we have a state change...
      last_CH1_state = 1;                            //Store the current state into the last state for the next loop
      counter_1 = current_count;                     //Set counter_1 to current value.
    }
  }
  else if(last_CH1_state == 1){                      //If pin 8 is LOW and the last state was HIGH then we have a state change      
    last_CH1_state = 0;                              //Store the current state into the last state for the next loop
    input_PITCH = current_count - counter_1;   //We make the time difference. Channel 1 is current_time - timer_1.
  }



  ///////////////////////////////////////Channel 2
  if(PINB & B00000010 ){                             //pin D9 -- B00000010                                              
    if(last_CH2_state == 0){                                               
      last_CH2_state = 1;                                                   
      counter_2 = current_count;                                             
    }
  }
  else if(last_CH2_state == 1){                                           
    last_CH2_state = 0;                                                     
    input_ROLL = current_count - counter_2;                             
  }


  
  


  
  ///////////////////////////////////////Channel 6
  if(PINB & B00010000 ){                             //pin D12  -- B00010000                      
    if(last_CH4_state == 0){                                               
      last_CH4_state = 1;                                                   
      counter_4 = current_count;                                              
    }
  }
  else if(last_CH4_state == 1){                                             
    last_CH4_state = 0;                                                  
    input_AUX = current_count - counter_4;                            
  }


///////////////////////////////////////Channel 5
  if(PINB & B00100000 ){                             //pin D13  -- B00010000                      
    if(last_CH5_state == 0){                                               
      last_CH5_state = 1;                                                   
      counter_5 = current_count;                                              
    }
  }
  else if(last_CH5_state == 1){                                             
    last_CH5_state = 0;                                                  
    input_YAW = current_count - counter_5;                            
  }

 
}
