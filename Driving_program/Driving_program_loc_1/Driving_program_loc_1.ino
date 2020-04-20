#include <SPI.h>                                          //LOKOMOTYWA SZARA
#include "RF24.h"

#define PWMa 10
#define A 4
#define B 5
#define Enc_A 2
#define Enc_B 3
#define acc_time 100

byte address[][6] = {"1Node", "2Node", "3Node", "4Node"};
int counter = 0;

//--------------PARAMETRY JAZDY ODEBRANE Z CENTRALI-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Control_Data
{
  public:
    Control_Data() {
      PWM = 0;
      direct = 0;
      acceleration = 0;
      encoder = 0;
      priority = false;
      fast_stop = false;
    }
    byte Get_PWM() const
    {
      return PWM;                       // Wypełnienie PWM
    }
    byte Get_dir() const
    {
      return direct;                     // Kierunek jazdy
    }
    byte Get_acceleration() const
    {
      return acceleration;              // Prędkość hamowania/przyśpieszania
    }
    byte Get_encoder() const
    {
      return encoder;                   // Zadany licznik enkodera (droga)
    }
    bool Get_priority() const
    {
      return priority;                  // Ustalany priorytet enkodera ( jazda swobodna lub zadana droga)
    }
    bool Get_fast_stop() const
    {
      return fast_stop;                 // Nakaz natychmiastowego zatrzymania
    }
  private:
    byte PWM;
    byte direct;
    byte acceleration;
    byte encoder;
    bool priority;
    bool fast_stop;
};

//--------------DANE WYSYŁANE DO CENTRALI----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Transmitted_Data
{
  public:
    void Set_address(const byte &a)
    {
      address = a;                        // Odczytanie adresu lokomotywy, wysyłającej dane
    }
    void Set_current_speed(const byte &a)
    {
      current_speed = a;                  //  Odczytanie aktualnej predkosci lokomotywy
    }
    void Set_error(const byte &a)
    {
      error = a;                          // Odebranie kodu błędu
    }
    void Set_encoder(const byte &a)
    {
      encoder = a;                        // Odebranie licznika encodera
    }
  private:
    byte address;
    byte current_speed;
    byte error;
    byte encoder;
};

//--------------ODBIERANIE I WYSYŁANIE DANYCH-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class RF_Communication
{
  public:
    RF_Communication(const int a, const int b): New_Transmission(a, b) {
      response = 1;
      error_status = 0;
      got_data = false;
    }
    //Ustawienie parametrów nadawania i odbioru
    void start()
    {
      New_Transmission.begin();                       //Rozpoczęcie pracy modułu
      New_Transmission.setChannel(108);               //Ustawienie częstotliwości nadawania
      New_Transmission.setDataRate(RF24_250KBPS);     //Ustawienie przepustowości
      New_Transmission.setPALevel(RF24_PA_LOW);       //Ustawienie mocy sygnału
      New_Transmission.setAutoAck(1);
      New_Transmission.enableDynamicPayloads();
      New_Transmission.enableAckPayload();
      //New_Transmission.setRetries(0,15);
      New_Transmission.openWritingPipe(address[2]);
      New_Transmission.openReadingPipe(1, address[0]);
      New_Transmission.startListening();
    }
    //Przesyłanie sygnału zwortnego z lokomotywy
    void transmit(const byte &a, const byte &b, const byte &c, const byte &d)
    {
      New_Data.Set_address(a);
      New_Data.Set_current_speed(b);
      New_Data.Set_error(c);
      New_Data.Set_encoder(d);
      New_Transmission.stopListening();
      //unsigned long response_time = micros();
      if (New_Transmission.write(&New_Data, sizeof(New_Data)))
      {
        if (!New_Transmission.available())
        {
          //DEBUG_PRINT("W przeciagu ");
          //DEBUG_PRINT(micros() - response_time);
          //DEBUG_PRINTLN("ms nie otrzymano odpowiedzi zwrotnej");
          error_status = 1;
        }
        else
        {
          while (New_Transmission.available())
          {
            New_Transmission.read(&response, 1);
            if ( response == 1)
            {
              //DEBUG_PRINT("Otrzymano odpowiedz po");
              //DEBUG_PRINT(micros() - response_time);
              //DEBUG_PRINTLN("ms");
              error_status = 0;
            }
            else
            {
              //DEBUG_PRINT("Błędna wielkość odpowiedzi zwrotnej");
              error_status = 1;
              break;
            }
          }
        }
      }
      else
      {
        //DEBUG_PRINT("Wysylanie wiadomosci nie powiodlo sie");
        error_status = 2;
      }
      New_Transmission.startListening();
    }
    //Odbieranie danych sterowania lokomotywą
    bool receive()
    {
      if (New_Transmission.available())
      {
        byte pipeNo;
        response = 1;
        while (New_Transmission.available(&pipeNo))
        {
          New_Transmission.read(&New_Control, sizeof(New_Control));
          New_Transmission.writeAckPayload(pipeNo, &response, 1);
        }
        response = 0;
        got_data = true;
      }
      else
      {
        got_data = false;
      }
      return got_data;
    }
    void reconnect()
    {
      New_Transmission.stopListening();
      New_Transmission.flush_tx();
      New_Transmission.closeReadingPipe(1);
      New_Transmission.openReadingPipe(1, address[0]);
      New_Transmission.startListening();
    }
    void Reset_Error_status()
    {
      error_status = 0;
    }
    byte Get_PWM()
    {
      return New_Control.Get_PWM();
    }
    byte Get_dir()
    {
      return New_Control.Get_dir();
    }
    byte Get_acceleration()
    {
      return New_Control.Get_acceleration();
    }
    byte Get_encoder()
    {
      return New_Control.Get_encoder();
    }
    bool Get_priority()
    {
      return New_Control.Get_priority();
    }
    bool Get_fast_stop()
    {
      return New_Control.Get_fast_stop();
    }
    byte Check_Error_Status()
    {
      return error_status;
    }
  private:
    RF24 New_Transmission;
    Control_Data New_Control;
    Transmitted_Data New_Data;
    byte error_status;
    byte response;
    bool got_data;
};

//--------------STEROWANIE SILNIKIEM----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Engine_Control
{
  public:
    Engine_Control() {
      current_PWM = 0;
      last_dir = 1;
      accelaration_time = 0;
      engine_break = false;
    }
    void Set_Config()
    {
      pinMode(PWMa, OUTPUT);
      pinMode(A, OUTPUT);
      pinMode(B, OUTPUT);
      analogWrite(PWMa, 0);
      digitalWrite(A, LOW);
      digitalWrite(B, LOW);
    }
    void controlling(RF_Communication &a)
    {
      if (a.receive() == true)
      {
        engine_break = false;
        if (a.Get_fast_stop() == true)
        {
          this->stop_engine();
        }
        a.transmit(0, current_PWM, a.Check_Error_Status(), counter);
        counter = 0;
        accelaration_time = millis();
      }
      if ((a.Get_PWM() != current_PWM) && ( engine_break == false) )
      {
        if (a.Get_dir() == 1 && last_dir == 1)
        {
          digitalWrite(A, HIGH);
          digitalWrite(B, LOW);
          if (current_PWM < a.Get_PWM())
          {
            if ( millis() - accelaration_time > acc_time )
            {
              current_PWM += a.Get_acceleration();
              if (current_PWM > 255)
              {
                current_PWM = 255;
              }
              analogWrite(PWMa, current_PWM);
              accelaration_time = millis();
            }
            last_dir = a.Get_dir();
          }
          else if (current_PWM > a.Get_PWM())
          {
            if ( millis() - accelaration_time > acc_time )
            {
              current_PWM -= a.Get_acceleration();
              if (current_PWM < 0)
              {
                current_PWM = 0;
              }
              analogWrite(PWMa, current_PWM);
              accelaration_time = millis();
            }
            last_dir = a.Get_dir();
          }
        }
        else if (a.Get_dir() == 0 && last_dir == 0)
        {
          digitalWrite(A, LOW);
          digitalWrite(B, HIGH);
          if (current_PWM < a.Get_PWM())
          {
            if ( millis() - accelaration_time > acc_time )
            {
              current_PWM += a.Get_acceleration();
              if (current_PWM > 255)
              {
                current_PWM = 255;
              }
              analogWrite(PWMa, current_PWM);
              accelaration_time = millis();
            }
            last_dir = a.Get_dir();
          }
          else if (current_PWM > a.Get_PWM())
          {
            if ( millis() - accelaration_time > acc_time )
            {
              current_PWM -= a.Get_acceleration();
              if (current_PWM < 0)
              {
                current_PWM = 0;
              }
              analogWrite(PWMa, current_PWM);
              accelaration_time = millis();
            }
            last_dir = a.Get_dir();
          }
        }
        else if (a.Get_dir() == 1 && last_dir == 0)
        {
          digitalWrite(A, LOW);
          digitalWrite(B, HIGH);
          if (current_PWM > 0)
          {
            if ( millis() - accelaration_time > acc_time )
            {
              current_PWM -= a.Get_acceleration();
              if (current_PWM < 0)
              {
                current_PWM = 0;
                last_dir = a.Get_dir();
              }
              analogWrite(PWMa, current_PWM);
              accelaration_time = millis();
            }
          }
          else
          {
            last_dir = a.Get_dir();
          }
        }
        else if (a.Get_dir() == 0 && last_dir == 1)
        {
          digitalWrite(A, HIGH);
          digitalWrite(B, LOW);
          if (current_PWM > 0)
          {
            if ( millis() - accelaration_time > acc_time )
            {
              current_PWM -= a.Get_acceleration();
              if (current_PWM < 0)
              {
                current_PWM = 0;
                last_dir = a.Get_dir();
              }
              analogWrite(PWMa, current_PWM);
              accelaration_time = millis();
            }
          }
          else
          {
            last_dir = a.Get_dir();
          }
        }
        else
        {
          a.reconnect();
        }
      }
    }
    void stop_engine()
    {
      current_PWM = 0;
      last_dir = 0;
      engine_break = true;
      analogWrite(A, LOW);
      analogWrite(B, LOW);
      analogWrite(PWMa, 0);
    }
  private:
    int current_PWM;                   // Obecna wartość PWM
    byte last_dir;                     // Obecny kierunek jazdy
    unsigned long accelaration_time;   // Zmienna czasowa odliczająca interwał przyśpieszenia
    bool engine_break;                 // Blokada jazdy


};

RF_Communication New_Com(7, 8);
Engine_Control New_Con;

//--------------FUNKCJA OBSŁUGI PRZERWANIA. AKTUALIZACJA LICZNIKA ENKODERA---------------------------------------------------------------------------
void count_distance(){
  if (digitalRead(Enc_B))
  {
    counter++;
  }
  else
  {
    counter--;
  }
  if (New_Com.Get_priority() == true)
  {
    if (abs(counter) == New_Com.Get_encoder())
    {
      New_Con.stop_engine();
      counter = 0;
    }
  }
}

void setup() {
  attachInterrupt(digitalPinToInterrupt(Enc_A), count_distance, FALLING);
  New_Com.start();
  New_Con.Set_Config();
}

void loop() {
  New_Con.controlling(New_Com);
}
