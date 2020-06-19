/***
      pinCalibracionAng = Pin del switch para calibrar el Angulo 0 MOTOR
      pinModoVP = Pin del switch para seleccionar el modo de control. Por Volumen o por Presión
      modoVP = modo seleccionado con respecto a forma de control
      esperarCalibracion = booleano que da un tiempo para que presionen botón a calibrar o no
      Tr = Periodo Respiratorio
      Ti = Tiempo Inspiracion
      Vs = Tiempo Subida
      Tex = Tiempo Expiracion
      Tes = Tiempo Espera (delay)
      P0 = Sensor de presion 0
      P1 = sensor de presion 1
***/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <Servo.h>

#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// PINES PUENTE H (MOTOR)
#define BRAKE 0
#define CW 1
#define CCW 2

// MOTOR 1
#define MOTOR_A1_PIN 7
#define MOTOR_B1_PIN 8
#define PWM_MOTOR_1 11
#define MOTOR_1 0

String error1 = "NO HAY BOMBEO", error2 = "SOBRE VOLUMEN", error3 = "SUBVOLUMEN", error4 = "SOBRE PRESION", error5 = "SUB PRESION", error6 = "DESCONEXION", error7 = "ALIMENTACIÓN ELECTRICA";

short usSpeed = 100;  //default motor speed
unsigned short usMotor_Status = BRAKE;

int anguloCeroMotor = 140, anguloActMotor = 0;

// Variables Potenciometros
const int presionUserPot = A0, volumenUserPot = A1, anguloMotorPot = A2, IEUserPot = A3, p0 = A6, p1 = A7;
int frecRespUserPot = 1023;

// Variables Usuario
int volumenUser = 0, frecRespUser = 0, presionUser = 0;
float IEUser = 0;
int volumenUserAnt = 0, frecRespUserAnt = 0, presionUserAnt = 0;
float IEUserAnt = 0;
String presionI, flujoI, e1, e2, e3, e4, e5, e6, e7;

// Pines control valvula, switch
const int pinCalibracionAng = 3, pinModoVP = 2;

int modoVP = 0;

// Variables fórmulas
float Tr = 0.0, Ti = 0.0, Ts = 0.0, Vs = 0.0, Tex = 0.0, Tes1 = 0.0, Tes2 = 0.0;

bool esperarCalibracion = false, buttonCalibrar = false, doCalibracion = false;

// CONTROL TIEMPOS
unsigned long tiempoInicio = 0, tiempoFin = 0, tiempoBajada = 0, tiempoMax = 0;


Servo servoMotor;

void setup()
{
  Serial.begin(9600);

  Wire.begin();                // join i2c bus with address #4
  // pinMode(pinCalibracionAng, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinCalibracionAng), buttonPressedCalibracion, RISING);

  pinMode(pinModoVP, INPUT);

  lcd.init();

  lcd.backlight();

  delay(500);

  lcd.setCursor(2, 0);
  lcd.print("Si desea calibrar");

  lcd.setCursor(2, 1);
  lcd.print("presiona el boton");

  Serial.println("IMPRIMIENDO");

  esperarCalibracion = true;

  int i = 0;

  while ( esperarCalibracion )
  {
    if ( i > 100 )
    {
      esperarCalibracion = false;
    }
    delay(50);
    i = i + 1;
  }

  lcd.clear();

  if (buttonCalibrar == true)
  {
    doCalibracion = true;
    calibracion();
  }

  setMotorCero();

  modoVP = digitalRead(pinModoVP);

  lcd.clear();

  servoMotor.attach(6);
}

void lecturaI2C()
{
  char c;
  String frecuencia, flujo, presion;
  Wire.requestFrom(1, 15);
  c = Wire.read();
  frecuencia += c;
  c = Wire.read();
  frecuencia += c;
  c = Wire.read();
  presion += c;
  c = Wire.read();
  presion += c;
  c = Wire.read();
  flujo += c;
  c = Wire.read();
  flujo += c;
  c = Wire.read();
  flujo += c;
  c = Wire.read();
  e1 = c;
  c = Wire.read();
  e2 = c;
  c = Wire.read();
  e3 = c;
  c = Wire.read();
  e4 = c;
  c = Wire.read();
  e5 = c;
  c = Wire.read();
  e6 = c;
  c = Wire.read();
  e7 = c;
  
  frecRespUser = frecuencia.toInt();

  if ( (frecRespUser < 12) || (frecRespUser > 27) )
  {
      frecRespUser = 12;  
  }


  // Serial.println("error 1: " + e1 + " error 2: " + e2 + " error 3: " + e3 + " error 4: " + e4 + " error 5: " + e5 + " error 6: " + e6 + " error 7: " + e7);
  presionI = presion;
  flujoI = flujo;
}

void buttonPressedCalibracion()
{
  Serial.println("Calibrando");
  if ( buttonCalibrar == false)
  {
    buttonCalibrar = true;
    esperarCalibracion = false;
    detachInterrupt(digitalPinToInterrupt(pinCalibracionAng));
  } else
  {
    buttonCalibrar = false;
    detachInterrupt(digitalPinToInterrupt(pinCalibracionAng));
  }
}

void calibracion()
{
  int anguloAnterior = 0;
  attachInterrupt(digitalPinToInterrupt(pinCalibracionAng), buttonPressedCalibracion, RISING);
  while (doCalibracion)
  {
    anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);

    if (anguloAnterior != anguloActMotor)
    {
      anguloAnterior = anguloActMotor;
      writeLCDcalibracion();
    }

    delay(100);

    if (buttonCalibrar == false)
    {
      anguloCeroMotor = anguloActMotor;
      // EEPROM.write(addr, anguloCeroMotor);
      doCalibracion = false;
    }
  }
}

void setMotorCero()
{
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("CONFIGURANDO ");
  lcd.setCursor(7, 2);
  lcd.print("EQUIPO");
  lecturaPot();
  if ( anguloCeroMotor > anguloActMotor )
  {
    usMotor_Status = CW;
    motorGo(MOTOR_1, usMotor_Status, 60);

    while ( anguloActMotor > anguloCeroMotor )
    {
      anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);
    }
  } else
  {
    usMotor_Status = CCW;

    motorGo(MOTOR_1, usMotor_Status, 60);
    while ( anguloActMotor < anguloCeroMotor )
    {
      anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);
    }
  }
}

void writeLCDcalibracion()
{
  lcd.setCursor(5, 0);
  lcd.print("Calibrando: ");

  lcd.setCursor(1, 1);
  lcd.print("Angulo Motor: ");
  lcd.print(anguloActMotor);

  lcd.setCursor(0, 2);
  lcd.print("Continuar pres boton");
}

void lecturaPot()
{
  int IE;
  volumenUser = map((analogRead(volumenUserPot)), 0, 1023, 40, 70);

  presionUser = map((analogRead(presionUserPot)), 0, 1023, 90, 255);

  IE = map((analogRead(IEUserPot)), 0, 1023, 10, 32);

  IEUser = float(IE) / 10;

  lecturaI2C();
  
}

String digitos(int n, int d) {
  int d1 = 0, d2 = 0, d3 = 0;
  if (d == 3) {
    d1 = n % 10;
    d2 = (n % 100) / 10;
    d3 = (n % 1000) / 100;
    return String(d3) + String(d2) + String(d1);
  } else {
    d1 = n % 10;
    d2 = (n % 100) / 10;
    return String(d2) + String(d1);
  }
}

void showLCD()
{
  lecturaPot();

  if ( ( e1 == "1" ) || ( e2 == "1" ) || ( e3 == "1" ) || ( e4 == "1" ) || ( e5 == "1" ) || ( e6 == "1" ) || ( e7 == "1" ) ) {
    showLCDError();
  } else {
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    
    lcd.setCursor(4, 0);
    lcd.print("EIRA DINNTEC");
  }
  

  if ( modoVP == 1 )
  {
    showLCDVolumen();
  } else
  {
    showLCDPresion();
  }

  String secondLine = "FR: " + String(frecRespUser) + " res/min";
  int espacioLine = (20 - secondLine.length()) / 2;

  lcd.setCursor(espacioLine, 2);
  lcd.print(secondLine);

  lcd.setCursor(0, 3);
  lcd.print("P:" + presionI + "cmH2O F:" + flujoI + "mml");
}

void showLCDPresion()
{
    float presionUserFloat;
    presionUserFloat = float(presionUser) * 0.170 - 3.3;
    String firstLine = "PU:" + digitos(round(presionUserFloat),2) + " cmH2O" + " IE:" + String(IEUser, 1);
    int espacioLine = (20 - firstLine.length()) / 2;
    lcd.setCursor(espacioLine, 1);
    lcd.print(firstLine);
}

void showLCDVolumen()
{
    String firstLine = "Vol: " + String(volumenUser) + " L " + "IE: " + String(IEUser, 1);
    int espacioLine = (20 - firstLine.length()) / 2;
    lcd.setCursor(espacioLine, 1);  
    lcd.print(firstLine);
}

void showLCDError()
{
    String error;
    if ( (e1 == "1") ) 
    {
        error = error1;
    }
    if ( e2 == "1" ) 
    {
        error = error2;
    }
    if ( e3 == "1" ) 
    {
        error = error3;
    }
    if ( e4 == "1" ) 
    {
        error = error4;
    }
    if ( e5 == "1" ) 
    {
        error = error5;
    }
    if ( e6 == "1" ) 
    {
        error = error6;
    }
    if ( e7 == "1" ) 
    {
        error = error7;
    }

    lcd.setCursor(0, 0);
    lcd.print("                    ");

    // Serial.println(error);
    int espacioLine = (20 - error.length()) / 2;
    lcd.setCursor(espacioLine, 0);  
    lcd.print(error);

}

void formulasVolumen()
{
    Tr = 60000.0 / float(frecRespUser);
    Ti = Tr / (1.0 + float(IEUser));
    Ts = 0.6 * Ti;
    Tex = Ti * float(IEUser);

    Tes1 = Ti - Ts; //  Tiempo de Espera Arriba

    Tes2 = Tex - tiempoBajada;  // Tiempo Espera abajo
}

void formulasPresion() 
{
    Tr = 60000.0 / float(frecRespUser);
    Ti = Tr / (1.0 + float(IEUser));
    Ts = 0.6 * Ti;
    Tex = Ti * float(IEUser);
    
    Tes1 = 0.3 * Ti;
    Tes2 = Tex - Ts;
}

void motorSubida()
{
  anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);
  usMotor_Status = CW;
  if ( modoVP == 1 ) // MODO VOLUMEN
  {
      lecturaI2C();
      Vs = 4.125 * presionI.toFloat() + 90;
      tiempoInicio = millis();
      motorGo(MOTOR_1, usMotor_Status, Vs);
      while ( anguloActMotor <= (anguloCeroMotor + volumenUser) ) {
          anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);
          Serial.println(anguloActMotor);
      }
      tiempoMax = millis();
      Ts = tiempoMax - tiempoInicio;
      Tes1 = Ti - Ts;

      Serial.println("Tiempo de Espera Subiendo: " + String(Tes1) + " Velocidad Subida: " + Vs);
      
  } else // MODO PRESIÓN
  {
      // Serial.println("Subiendo");
      motorGo(MOTOR_1, usMotor_Status, (presionUser));
      tiempoInicio = millis();
      while ( anguloActMotor <= (anguloCeroMotor + 90) ) 
      {
          anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);
          Serial.println(anguloActMotor);
          tiempoMax = millis();
          if ( (tiempoMax - tiempoInicio) >= Ts )
          {
              break;
          }
      }
  }
  
}

void motorBajada()
{
  usMotor_Status = CCW;
  if ( modoVP == 1 )
  {
    tiempoInicio = millis();
    motorGo(MOTOR_1, usMotor_Status, 70);
    while ( anguloActMotor >= anguloCeroMotor )
    {
      anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);
    }
    tiempoMax = millis();
    unsigned long Tb = 0;
    Tes2 = Tex - Tb;
  } else
  {
      motorGo(MOTOR_1, usMotor_Status, (60));
      tiempoInicio = millis();
      while ( anguloActMotor >= anguloCeroMotor ) 
      {
          anguloActMotor = map((analogRead(anguloMotorPot)), 0, 1023, 0, 300);
          tiempoMax = millis();

          /*if ( (tiempoMax - tiempoInicio) >= Ts )
          {
              break;
          }*/
      }
  }

  usMotor_Status = BRAKE;
  motorGo(MOTOR_1, usMotor_Status, 0);
  tiempoFin = millis();
  tiempoBajada = tiempoFin - tiempoInicio;
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)         //Function that controls the variables: motor(0 ou 1), direction (cw ou ccw) e pwm (entra 0 e 255);
{
  if (motor == MOTOR_1)
  {
    if (direct == CW)
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, HIGH);
    }
    else if (direct == CCW)
    {
      digitalWrite(MOTOR_A1_PIN, HIGH);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }
    else
    {
      digitalWrite(MOTOR_A1_PIN, LOW);
      digitalWrite(MOTOR_B1_PIN, LOW);
    }

    analogWrite(PWM_MOTOR_1, pwm);
  }
}

void loop()
{
  showLCD();

  if ( modoVP == 1 )
  {
    formulasVolumen();
  } else
  {
    formulasPresion();
  }

  imprimir();

  motorSubida();

  usMotor_Status = BRAKE;
  motorGo(MOTOR_1, usMotor_Status, 0);

  showLCD();

  delay(Tes1);
  
  servoMotor.write(55);

  motorBajada();

  if ( Tes2 < 0 ) {
    Tes2 = 0;
  }

  delay(Tes2);

  servoMotor.write(0);
}


void imprimir() 
{
  Serial.println("Tr: " + String(Tr) + " Ti: " + String(Ti) + " Ts: " + String(Ts) + " Tex: " + String(Tex));
  Serial.println("Velocidad: " + String(presionUser));
  Serial.println("////////////////////////////////////////");
}
