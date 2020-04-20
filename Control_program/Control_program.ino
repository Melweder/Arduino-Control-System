#include "RF24.h"
#include <SPI.h>
#include <PCF8574.h>
#include <Wire.h>

#define DIR1 1
#define DIR2 0
#define FAST 150
#define SLOW 50
#define LOC1 0
#define LOC2 1
#define DEBUG_PRINTER Serial

//#define PROGRAM_DEBUG
#ifdef PROGRAM_DEBUG
#define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
#define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#define DEBUG_START(...) { DEBUG_PRINTER.begin(__VA_ARGS__); }
#define DEBUG_STATUS DEBUG_PRINTER
#else
#define DEBUG_PRINT(...) {}
#define DEBUG_PRINTLN(...) {}
#define DEBUG_START(...) {}
#define DEBUG_STATUS 1
#endif

byte address[][6] = {"1Node", "2Node", "3Node", "4Node"};
bool STATE;
void On_Interrupt();

//--------------PARAMETRY PRZESYŁANE DO LOKOMOTYWY-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Control_Data {
  public:
    void Set_PWM(const byte &a)
    {
      PWM = a;                                  // Zadane wypełnienie PWM
    }
    void Set_direction(const byte &a)
    {
      direct = a;                               // Zadany kierunek jazdy
    }
    void Set_acceleration(const byte &a)
    {
      acceleration = a;                         // Zadana prędkość hamowania/przyśpieszania
    }
    void Set_encoder(const byte &a)
    {
      encoder = a;                              // Zadany licznik enkodera (droga)
    }
    void Set_encoder_priority(const bool &a)
    {
      priority = a;                             // Ustalany priorytet enkodera ( jazda swobodna lub zadana droga)
    }
    void Set_fast_stop(const bool &a)
    {
      fast_stop = a;                            // Nakaz natychmiastowego zatrzymania
    }
    void Print_Data()
    {
      DEBUG_PRINTLN("<<<<<<<<<<<----------->>>>>>>>>>>>");
      DEBUG_PRINT("PWM: ");
      DEBUG_PRINTLN(PWM);
      DEBUG_PRINT("STOP: ");
      DEBUG_PRINTLN(fast_stop);
      DEBUG_PRINTLN("<<<<<<<<<<<----------->>>>>>>>>>>>");
    }
  private:
    byte PWM;
    byte direct;
    byte acceleration;
    byte encoder;
    bool priority;
    bool fast_stop;
};

//--------------DANE ODEBRANE Z LOKOMOTYWY----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Transmitted_Data {
  public:
    byte Get_address() const
    {
      return address;                           // Odczytanie adresu lokomotywy, wysyłającej dane
    }
    byte Get_current_speed() const
    {
      return current_speed;                     //  Odczytanie aktualnej predkosci lokomotywy
    }
    byte Get_error() const
    {
      return error;                             // Odebranie kodu błędu
    }
    byte Get_encoder() const
    {
      return encoder;                           // Odebranie licznika encodera
    }
  private:
    byte address;
    byte current_speed;
    byte error;
    byte encoder;
};

//--------------STAN LOKOMOTYW----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class Loc_State {
  public:
    byte Get_loc_speed(const byte &a) const
    {
      if ( a == 0 )
      {
        return loc_speed_1;
      }
      else if ( a == 1 )
      {
        return loc_speed_2;
      }
      else
      {
        return 0;
      }
    }
    bool Get_loc_priority(const byte &a) const
    {
      if ( a == 0 )
      {
        return loc_priority_1;
      }
      else if ( a == 1 )
      {
        return loc_priority_2;
      }
      else
      {
        return 0;
      }
    }
    byte Get_loc_section(const byte &a) const
    {
      if ( a == 0 )
      {
        return loc_1_section;
      }
      else if ( a == 1 )
      {
        return loc_2_section;
      }
      else
      {
        return 0;
      }
    }
    bool Get_loc_waiting_status(const byte &a) const
    {
      if ( a == 0 )
      {
        return loc_1_waiting;
      }
      else if ( a == 1 )
      {
        return loc_2_waiting;
      }
      else
      {
        return 0;
      }
    }
    byte Get_loc_mode(const byte &a) const
    {
      if ( a == 0 )
      {
        return loc_1_mode;
      }
      else if ( a == 1 )
      {
        return loc_2_mode;
      }
      else
      {
        return 0;
      }
    }
    void Set_loc_speed(const byte &a, const byte &b)
    {
      if ( a == 0 )
      {
        loc_speed_1 = b;
      }
      else if ( a == 1 )
      {
        loc_speed_2 = b;
      }
      else
      {
        return 0;
      }
    }
    void Set_loc_priority(const byte &a, const bool &b)
    {
      if ( a == 0 )
      {
        loc_priority_1 = b;
      }
      else if ( a == 1 )
      {
        loc_priority_2 = b;
      }
      else
      {
        return 0;
      }
    }
    void Set_loc_section(const byte &a, const byte &b)
    {
      if ( a == 0 )
      {
        loc_1_section = b;
      }
      else if ( a == 1 )
      {
        loc_2_section = b;
      }
      else
      {
        return 0;
      }
    }
    void Set_loc_waiting_status(const byte &a, const byte &b)
    {
      if ( a == 0 )
      {
        loc_1_waiting = b;
      }
      else if ( a == 1 )
      {
        loc_2_waiting = b;
      }
      else
      {
        return 0;
      }
    }
    void Set_loc_mode(const byte &a, const byte &b)
    {
      if ( a == 0 )
      {
        loc_1_mode = b;
      }
      else if ( a == 1 )
      {
        loc_2_mode = b;
      }
      else
      {
        return 0;
      }
    }
  private:
    byte loc_speed_1;          // Predkosc lokomotywy 1 ZADANA! ( Tylko na potrzeby programu sterujacego )
    byte loc_speed_2;          // Predkosc lokomotywy 2 ZADANA! ( Tylko na potrzeby programu sterujacego )
    bool loc_priority_1;       // Pozwolenie startu dla lokokotywy 1
    bool loc_priority_2;       // Pozwolenie startu dla lokomotywy 2
    byte loc_1_section;        // Przypisanie lokomotywy 1 do sekcji torowej (1, 2 lub 3)
    byte loc_2_section;        // Przypisanie lokomotywy 2 do sekcji torowej (1, 2 lub 3)
    bool loc_1_waiting;        // Status oczekiwania lokomotywy 1 ( wartosc TRUE dla warunkowego trybu jazdy (opis ponizej) )
    bool loc_2_waiting;        // Status oczekiwania lokomotywy 2 ( wartosc TRUE dla warunkowego trybu jazdy (opis ponizej) )
    byte loc_1_mode;           // Warunkowy tryb jazdy lokomotywy 1 (niezalezny od odczytu czujnikow): 0 - nieaktywny, 1- chwilowe zatrzymanie (stacja, przejazd), 2- zmniejszenie predkosci w sekcji 1, 3 - zmniejszenie predkosci w sekcji 2, 4 - zmniejszenie predkosci w sekcji 3, 5 - zapytanie o pozwolenie na start, 6 - zapytanie o swobodny przejazd
    byte loc_2_mode;           // Warunkowy tryb jazdy lokomotywy 2 (niezalezny od odczytu czujnikow): 0 - nieaktywny, 1- chwilowe zatrzymanie (stacja, przejazd), 2- zmniejszenie predkosci w sekcji 1, 3 - zmniejszenie predkosci w sekcji 2, 4 - zmniejszenie predkosci w sekcji 3, 5 - zapytanie o pozowlenie na start, 6 - zapytanie o swobodny przejazd
};

//--------------ODBIERANIE I WYSYŁANIE DANYCH PRZEZ MODUŁ RADIOWY-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class RF_Communication {
  public:
    RF_Communication(const int a, const int b): New_Transmission(a, b) {
      error_status = 0;
      response = 1;
    }

    //Ustawienie parametrów nadawania i odbioru

    void start()
    {
      New_Transmission.begin();                           //Rozpoczęcie pracy modułu
      New_Transmission.setChannel(108);                   //Ustawienie częstotliwości nadawania
      New_Transmission.setDataRate(RF24_250KBPS);         //Ustawienie przepustowości
      New_Transmission.setPALevel(RF24_PA_LOW);           //Ustawienie mocy sygnału
      New_Transmission.setAutoAck(1);                     //Włączenie automatycznej odpowiedzi
      New_Transmission.enableDynamicPayloads();           //Włączenie dynamicznych lokowanych pakietów
      New_Transmission.enableAckPayload();                //Włączenie odpowiedzi zwrotnej potwierdzającej odebranie pakietu
      //New_Transmission.setRetries(0,15);                //Ustawienie czasu odpowiedzi oraz ilosci prób
      New_Transmission.openWritingPipe(address[0]);       //Otworzenie kanału nadawania danych
      New_Transmission.openReadingPipe(1, address[2]);    //Otworzenie kanału odbierania od pierwszej lokomotywy
      New_Transmission.openReadingPipe(2, address[3]);    //Otworzenie kanału odbierania od drugiej lokomotywy
      New_Transmission.startListening();                  //Rozpoczęcie nasłuchu
    }

    //Przesyłanie danych sterowania lokomotywą

    void transmit(const bool &a, const byte &b, const byte &c, const byte &d, const byte &e, const byte &f, const bool &g)
    {
      New_Control.Set_PWM(b);
      New_Control.Set_direction(c);
      New_Control.Set_acceleration(d);
      New_Control.Set_encoder(e);
      New_Control.Set_encoder_priority(a);
      New_Control.Set_fast_stop(g);
      New_Control.Print_Data();
      New_Transmission.stopListening();
      New_Transmission.openWritingPipe(address[f]);
      DEBUG_PRINT("Wysyłam dane do lokomotywy  ");
      if (f == LOC1)
      {
        DEBUG_PRINTLN("szarej");
      }
      else
      {
        DEBUG_PRINTLN("czarnej");
      }
      unsigned long response_time = micros();

      if (New_Transmission.write(&New_Control, sizeof(New_Control)))
      {
        if (!New_Transmission.available())
        {
          DEBUG_PRINT("W przeciagu ");
          DEBUG_PRINT(micros() - response_time);
          DEBUG_PRINTLN("ms nie otrzymano odpowiedzi zwrotnej");
          error_status = 1;
        }
        else
        {
          while (New_Transmission.available())
          {
            New_Transmission.read(&response, 1);
            if ( response == 1 )
            {
              if ( micros() - response_time < 5000)
              {
                //DEBUG_PRINT("Otrzymano odpowiedz po");
                //DEBUG_PRINT(micros() - response_time);
                //DEBUG_PRINTLN("ms");
                error_status = 0;
              }
              else
              {
                DEBUG_PRINTLN("Przekroczono czas odczytu bufora");
                error_status = 1;
                break;
              }
            }
            else
            {
              DEBUG_PRINTLN("Błędna wielkość odpowiedzi zwrotnej");
              error_status = 1;
              break;
            }
          }
        }
      }
      else
      {
        DEBUG_PRINTLN("Wysylanie wiadomosci nie powiodlo sie");
        error_status = 2;
      }
      New_Transmission.startListening();
    }

    //Odbieranie sygnału zwortnego z lokomotywy

    bool receive()
    {
      if (New_Transmission.available())
      {
        DEBUG_PRINTLN("Otrzymano pakiet danych z lokomotywy");
        byte pipeNo;
        response = 1;
        while (New_Transmission.available(&pipeNo))
        {
          New_Transmission.read(&New_Data, sizeof(New_Data));
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

    //Ponowne połączenie

    void reconnect()
    {
      DEBUG_PRINTLN("Wystapil blad w komunikacji! Łącze się ponownie...");
      New_Transmission.stopListening();
      New_Transmission.flush_tx();
      New_Transmission.closeReadingPipe(1);
      New_Transmission.closeReadingPipe(2);
      New_Transmission.openReadingPipe(1, address[2]);
      New_Transmission.openReadingPipe(2, address[3]);
      New_Transmission.startListening();
    }

    byte Check_Error_Status()
    {
      return error_status;
    }

    void Reset_Error_status()
    {
      error_status = 0;
    }
    
  private:
    RF24 New_Transmission;
    Control_Data New_Control;
    Transmitted_Data New_Data;
    byte error_status;
    byte response;
    bool got_data;
};

//--------------ODBIERANIE I PRZECHOWYWANIE DANYCH Z CZUJNIKÓW ODBICIOWYCH--------------------------------------------------------------------
class Sensor_Data {
  public:
    Sensor_Data(const uint8_t address_1, const uint8_t INT_PIN_1, void (*int_fun_1)(), const uint8_t address_2, const uint8_t INT_PIN_2, void (*int_fun_2)()):
      expander_1(address_1, INT_PIN_1, int_fun_1), expander_2(address_2, INT_PIN_2, int_fun_2) {}

    void start()
    {
      for (byte k = 0; k < 8; k++)
      {
        expander_1.pinMode(k, INPUT);
        expander_2.pinMode(k, INPUT);
      }
      expander_1.begin();
      expander_2.begin();
    }

    void Read_and_Save()
    {
      PCF8574::DigitalInput DI1 = expander_1.digitalReadAll();
      PCF8574::DigitalInput DI2 = expander_2.digitalReadAll();
      Sensors[0] = DI1.p0;
      Sensors[1] = DI1.p1;
      Sensors[2] = DI1.p2;
      Sensors[3] = DI1.p3;
      Sensors[4] = DI1.p4;
      Sensors[5] = DI1.p5;
      Sensors[6] = DI1.p6;
      Sensors[7] = DI1.p7;
      Sensors[8] = DI2.p0;
      Sensors[9] = DI2.p1;
      Sensors[10] = DI2.p2;
      Sensors[11] = DI2.p3;
      Sensors[12] = DI2.p4;
      Sensors[13] = DI2.p5;
      Sensors[14] = DI2.p6;
      Sensors[15] = DI2.p7;
    }
 
    byte *Get_Sensor_Data() const
    {
      return Sensors;
    }
  private:
    PCF8574 expander_1;
    PCF8574 expander_2;
    byte Sensors[16];
};

//--------------FUNKCJA OBSŁUGI PRZERWANIA-----------------------------------------------------------------------------------------------------
void On_Interrupt() {
  STATE = true;
}

//--------------STEROWANIE ZWROTNICĄ------------------------------------------------------------------------------------------------------------
class Steering_control {
  public:
    void steering_begin()
    {
      pinMode( 5, OUTPUT );
      pinMode( 6, OUTPUT );
      digitalWrite( 5, LOW );
      digitalWrite( 6, LOW );
      steering_status[0] = false; // PRAWA (1) ZWROTNICA - POZYCJA SPOCZYNKOWA - SEKCJA 3
      steering_status[1] = false; // LEWA (8) ZWROTNICA - POZYCJA SPOCZYNKOWA - SEKCJA 3
    }
    void steering_control(const byte &a, const bool &b)
    {
      if ( b == false )
      {
        if ( a == 2 )
        {
          digitalWrite( 5, HIGH );
          steering_status[b] = true;
        }
        else if ( a == 3 )
        {
          digitalWrite( 5, LOW );
          steering_status[b] = false;
        }
        else
        {
          DEBUG_PRINT("BŁĘDNA SEKCJA PRZY STEROWANIU ZWROTNICĄ");
        }
      }
      else if ( b == true )
      {
        if ( a == 2 )
        {
          digitalWrite( 6, HIGH );
          steering_status[b] = true;
        }
        else if ( a == 3 )
        {
          digitalWrite( 6, LOW );
          steering_status[b] = false;
        }
        else
        {
          DEBUG_PRINT("BŁĘDNA SEKCJA PRZY STEROWANIU ZWROTNICĄ");
        }
      }
      else
      {
        DEBUG_PRINT("BŁĘDNY ADRES ZWROTNICY");
      }
    }
  private:
    bool steering_status[2];

};

//--------------OKREŚLANIE LOKALIZACJI LOKOMOTYWY----------------------------------------------------------------------------------------------------
class Localization {
  public:
    Localization() {
      last_poz_loc_1 = 0;
      last_poz_loc_2 = 0;
      dir_loc_1 = DIR1;      //Nadac kierunki
      dir_loc_2 = DIR2;
      error_status_1 = 0;
      error_status_2 = 0;
      poz_loc_1_changed = false;
      poz_loc_2_changed = false;
    }

    // USTALANIE AKTUALNEJ LOKALIZACJI NA PODSTAWIE ODCZYTÓW Z CZUJNIKÓW
    void searching_localization(const byte *a)
    {
      byte sum = 0;
      bool done = false;
      byte sensor_1 = 0;
      byte sensor_2 = 0;
      poz_loc_1_changed = false;
      poz_loc_2_changed = false;
      for (byte k = 0; k < 16; k++)
      {
        sum += a[k];
        // OKREŚLANIE POZYCJI PIERWSZEJ LOKOMOTYWY TUŻ PO URUCHOMIENIU
        if ( last_poz_loc_1 == 0 && last_poz_loc_2 == 0 && a[k] == 0 )
        {
          last_poz_loc_1 = this->check_sensor_order(k);
          done = true;
          break;
        }
        // OKREŚLANIE POZYCJI DRUGIEJ LOKOMOTYWY TUŻ PO URUCHOMIENIU
        else if ( last_poz_loc_1 != 0 && last_poz_loc_2 == 0 && a[k] == 0 )
        {
          last_poz_loc_2 = this->check_sensor_order(k);
          if ( last_poz_loc_2 != last_poz_loc_1 )
          {
            done = true;
            break;
          }
          else
          {
            last_poz_loc_2 = 0;
          }
        }
        // ZAPIS NUMERU CZUJNIKA, KTÓRY WYKRYŁ LOKOMOTYWĘ
        else if ( last_poz_loc_1 != 0 && last_poz_loc_2 != 0 && a[k] == 0 && sensor_1 == 0 )
        {
          sensor_1 = k + 1;
        }
        // OPCJONALNY ZAPIS NUMERU CZUJNIKA, KTÓRY W TYM SAMYM CZASIE WYKRYŁ DRUGĄ LOKOMOTYWĘ
        else if ( last_poz_loc_1 != 0 && last_poz_loc_2 != 0 && a[k] == 0 && sensor_1 != 0 && sensor_2 == 0 )
        {
          sensor_2 = k + 1;
        }
        // BŁĄD!!! ODCZYT Z WIĘCEJ NIŻ 2 CZUJNIKÓW NARAZ!!!
        else if ( last_poz_loc_1 != 0 && last_poz_loc_2 != 0 && a[k] == 0 && sensor_1 != 0 && sensor_2 != 0 )
        {
          DEBUG_PRINT("Odczyt z więcej niż dwóch czujników naraz!!!");
          error_status_1 = 1;
          error_status_2 = 1;
          done = true;
        }
      }
      // OKREŚLANIE POZYCJI LOKOMOTYWY LUB LOKOMOTÓW W TRYBIE PRACY
      if ( done == false)
      {
        // LOKOMOTYWY POZA ZASIEGIEM CZUJNIKOW
        if ( sum == 16 )
        {
          done = true;
        }
        // ZAREJESTROWANO JEDNA LOKOMOTYWE
        else if ( sum == 15 )
        {
          done = this->find_proper_loc ( sensor_1 );
        }
        // ZAREJESTROWANO DWIE LOKOMOTYWY
        else if ( sum == 14 )
        {
          bool done_1 = false;
          bool done_2 = false;
          byte backup_pos_1 = last_poz_loc_1;
          byte backup_pos_2 = last_poz_loc_2;
          done_1 = this->find_proper_loc ( sensor_1 );
          done_2 = this->find_proper_loc ( sensor_2 );
          if ( done_1 == true && done_2 == true )
          {
            if ( last_poz_loc_1 != last_poz_loc_2 )
            {
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Algorytm wskazał tą samą lokomotywę dla dwóch różnych czujników");
              done = true;
              error_status_1 = 1;
              error_status_2 = 1;
            }
          }
          else
          {
            done = true;
            DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy");
            error_status_1 = 1;
            error_status_2 = 1;
          }

        }
      }

    }
    bool find_proper_loc( const byte &sensor_x )
    {
      byte actual_poz;
      bool done = false;
      actual_poz = this->check_sensor_order(sensor_x - 1);

      //POWTORZENIE - FILTROWANIE NADMIAROWYCH ODCZYTÓW Z TEGO SAMEGO CZUJNIKA

      if ( actual_poz == last_poz_loc_1 || actual_poz == last_poz_loc_2 )
      {
        done = true;
      }

      //POZYCJE STANDARDOWE

      else if ( actual_poz == 4 || actual_poz == 5 || actual_poz == 9 || actual_poz == 10 || actual_poz == 11 )
      {
        if ( abs(actual_poz - last_poz_loc_1) == 1 )
        {
          last_poz_loc_1 = actual_poz;
          poz_loc_1_changed = true;
          done = true;
        }
        else if ( abs(actual_poz - last_poz_loc_2) == 1 )
        {
          last_poz_loc_2 = actual_poz;
          poz_loc_2_changed = true;
          done = true;
        }
        else
        {
          DEBUG_PRINTLN("BŁĄD!!! Zgubiono lokalizację lokomotywy");
          error_status_1 = 1;
          error_status_2 = 1;
          done = false;
        }
      }

      // POZYCJE NIESTANDARDOWE

      else
      {
        switch ( actual_poz )
        {
          case 1: // POZYCJA NR 1

            if ( last_poz_loc_1 == 12 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if (last_poz_loc_2 == 12 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else if ( (last_poz_loc_1 == 2 || last_poz_loc_1 == 3) && dir_loc_1 == DIR1 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if ( (last_poz_loc_2 == 2 || last_poz_loc_2 == 3) && dir_loc_2 == DIR1 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy z pozycji 1");
              error_status_1 = 1;
              error_status_2 = 1;
              done = false;
            }
            break;

          case 2: // POZYCJA NR 2

            if ( last_poz_loc_1 == 7 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if (last_poz_loc_2 == 7 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else if ( last_poz_loc_1 == 1 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if ( last_poz_loc_2 == 1 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy z pozycji 2");
              error_status_1 = 1;
              error_status_2 = 1;
              done = false;
            }
            break;

          case 3: // POZYCJA NR 3

            if ( last_poz_loc_1 == 4 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if (last_poz_loc_2 == 4 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else if ( last_poz_loc_1 == 1 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if ( last_poz_loc_2 == 1 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy z pozycji 3");
              error_status_1 = 1;
              error_status_2 = 1;
              done = false;
            }
            break;

          case 6: // POZYCJA NR 6

            if ( last_poz_loc_1 == 5 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if (last_poz_loc_2 == 5 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else if ( last_poz_loc_1 == 8 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if ( last_poz_loc_2 == 8 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy z pozycji 6");
              error_status_1 = 1;
              error_status_2 = 1;
              done = false;
            }
            break;

          case 7: // POZYCJA NR 7

            if ( last_poz_loc_1 == 2 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if (last_poz_loc_2 == 2 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else if ( last_poz_loc_1 == 8 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if ( last_poz_loc_2 == 8 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy z pozycji 7");
              error_status_1 = 1;
              error_status_2 = 1;
              done = false;
            }
            break;

          case 8: // POZYCJA NR 8

            if ( last_poz_loc_1 == 9 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if (last_poz_loc_2 == 9 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else if ( (last_poz_loc_1 == 7 || last_poz_loc_1 == 6) && dir_loc_1 == DIR2 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if ( (last_poz_loc_2 == 7 || last_poz_loc_2 == 6) && dir_loc_2 == DIR2 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy z pozycji 8");
              error_status_1 = 1;
              error_status_2 = 1;
              done = false;
            }
            break;

          case 12: // POZYCJA NR 12

            if ( last_poz_loc_1 == 11 || last_poz_loc_1 == 1 )
            {
              last_poz_loc_1 = actual_poz;
              poz_loc_1_changed = true;
              done = true;
            }
            else if (last_poz_loc_2 == 11 || last_poz_loc_2 == 1 )
            {
              last_poz_loc_2 = actual_poz;
              poz_loc_2_changed = true;
              done = true;
            }
            else
            {
              DEBUG_PRINT("BŁĄD!!! Zgubiono lokalizację lokomotywy z pozycji 12");
              error_status_1 = 1;
              error_status_2 = 1;
              done = false;
            }
            break;

          default:
            done = false;
            break;
        }
      }
      return done;
    }

    // WAŻNE!!!! TUTAJ PRZYPISANE SĄ KONKERTNE CZUJNIKI DO KONKRETNYCH PUNKTÓW KONTROLNYCH

    byte check_sensor_order(const byte &a)
    {
      byte checkpoint;
      switch (a)
      {
        // EKSPANDER PIERWSZY
        case 0:  // LOKACJA CZUJNIKA PODPIĘTEGO DO PIERWSZEGO WEJŚCIA
          checkpoint = 1;
          break;
        case 1:  // LOKACJA CZUJNIKA PODPIĘTEGO DO DRUGIEGO WEJŚCIA
          checkpoint = 2;
          break;
        case 2:  // LOKACJA CZUJNIKA PODPIĘTEGO DO TRZECIEGO WEJŚCIA
          checkpoint = 3;
          break;
        case 3:  // LOKACJA CZUJNIKA PODPIĘTEGO DO CZWARTEGO WEJŚCIA
          checkpoint = 4;
          break;
        case 4:  // LOKACJA CZUJNIKA PODPIĘTEGO DO PIĄTEGO WEJŚCIA
          checkpoint = 5;
          break;
        case 5:  // LOKACJA CZUJNIKA PODPIĘTEGO DO SZÓSTEGO WEJŚCIA
          checkpoint = 6;
          break;
        case 6:  // LOKACJA CZUJNIKA PODPIĘTEGO DO SIÓDMEGO WEJŚCIA
          //checkpoint = 1;
          break;
        case 7:  // LOKACJA CZUJNIKA PODPIĘTEGO DO ÓSMEGO WEJŚCIA
          //checkpoint = 1;
          break;
        // EKSPANDER DRUGI
        case 8:  // LOKACJA CZUJNIKA PODPIĘTEGO DO PIERWSZEGO WEJŚCIA
          checkpoint = 7;
          break;
        case 9:  // LOKACJA CZUJNIKA PODPIĘTEGO DO DRUGIEGO WEJŚCIA
          checkpoint = 8;
          break;
        case 10:  // LOKACJA CZUJNIKA PODPIĘTEGO DO TRZECIEGO WEJŚCIA
          checkpoint = 9;
          break;
        case 11:  // LOKACJA CZUJNIKA PODPIĘTEGO DO CZWARTEGO WEJŚCIA
          checkpoint = 10;
          break;
        case 12:  // LOKACJA CZUJNIKA PODPIĘTEGO DO PIĄTEGO WEJŚCIA
          checkpoint = 11;
          break;
        case 13:  // LOKACJA CZUJNIKA PODPIĘTEGO DO SZÓSTEGO WEJŚCIA
          checkpoint = 12;
          break;
        case 14:  // LOKACJA CZUJNIKA PODPIĘTEGO DO SIÓDMEGO WEJŚCIA
          //checkpoint = 1;
          break;
        case 15:  // LOKACJA CZUJNIKA PODPIĘTEGO DO ÓSMEGO WEJŚCIA
          //checkpoint = 1;
          break;
        default:
          DEBUG_PRINTLN("Wielkość tablicy poza skalą!");
          checkpoint = 0;
          break;
      }
      return checkpoint;
    }

    void Reset_error()
    {
      error_status_1 = 0;
      error_status_2 = 0;
    }

    void Print_position(const byte &a)
    {
      if ( a == 0 )
      {
        DEBUG_PRINT("Pozycja lokomotywy szarej: ");
        DEBUG_PRINTLN(last_poz_loc_1);
      }
      else
      {
        DEBUG_PRINT("Pozycja lokomotywy czarnej: ");
        DEBUG_PRINTLN(last_poz_loc_2);
      }
      DEBUG_PRINTLN("************************************");
    }
    // POBIERANIE DANYCH

    byte Get_Poz_Loc(const byte &a)
    {
      if ( a == 0 )
      {
        return last_poz_loc_1;
      }
      else if ( a == 1 )
      {
        return last_poz_loc_2;
      }
      else
      {
        return 0;
        DEBUG_PRINT("Bledne odczytanie wartosci!");
      }
    }
    byte Get_Error_status()
    {
      if ( error_status_1 == 1 || error_status_2 == 1)
      {
        return 1;
      }
      else
      {
        return 0;
      }
    }
    byte Get_dir_loc(const byte &a)
    {
      if ( a == 0 )
      {
        return dir_loc_1;
      }
      else if ( a == 1 )
      {
        return dir_loc_2;
      }
      else
      {
        return 0;
        DEBUG_PRINT("Bledne odczytanie wartosci!");
      }
    }
    byte Get_change_status(const byte &a)
    {
      if ( a == 0 )
      {
        return poz_loc_1_changed;
      }
      else if ( a == 1 )
      {
        return poz_loc_2_changed;
      }
      else
      {
        return 0;
        DEBUG_PRINT("Bledne odczytanie wartosci!");
      }
    }

  private:
    byte last_poz_loc_1;
    byte last_poz_loc_2;
    byte dir_loc_1;
    byte dir_loc_2;
    bool error_status_1;
    bool error_status_2;
    bool poz_loc_1_changed;
    bool poz_loc_2_changed;
};

//--------------PRZETWARZANIE DANYCH I PRZELICZANIE STEROWANIA------------------------------------------------------------------------------------------
class Processing_Data {
  public:
    Processing_Data()
    {
      New_Com = new RF_Communication(8, 10);
      start_delay[LOC1] = false;
      start_delay[LOC2] = false;
      check_ride_permission[LOC1] = false;
      check_ride_permission[LOC2] = false;
    }

    //  OKREŚLANIE POŁOŻENIA POCZĄTKOWEGO LOKOMOTYW

    void auto_localization(Sensor_Data &a)
    {
      New_steering.steering_begin();
      DEBUG_PRINTLN("Rozpoczynam samoloklizacje");
      New_Com->start();
      STATE = false;
      New_Com->transmit(false, 150, DIR1, 10, 0, LOC1, false);
      while (New_Com->receive() == false){
      }
      while ( digitalRead(0) == HIGH )
      {
        if (New_Com->Check_Error_Status() == 0){
        }
        else
        {
          DEBUG_PRINTLN("BŁĄD AUTOLOKALIZACJI");
          break;
        }
      }
      DEBUG_PRINTLN("WYKRYTO LOKOMOTYWE");
      a.Read_and_Save();
      New_Localization.searching_localization(a.Get_Sensor_Data());
      DEBUG_PRINTLN("HAMUJE LOKOMOTYWE SZARA");
      New_Com->transmit(false, 0, DIR1, 0, 0, LOC1, true);
      while (New_Com->receive() == false){  
      }
      DEBUG_PRINT("POZYCJA SZAREJ LOKOMOTYWY TO: ");
      DEBUG_PRINTLN(New_Localization.Get_Poz_Loc(0));
      delay(1000);
      STATE = false;
      New_Com->transmit(false, 150, DIR2, 10, 0, LOC2, false);
      while (New_Com->receive() == false)
      {
        //DEBUG_PRINTLN("Oczekiwanie na dane z lokomotywy");
      }
      while ( digitalRead(1) == HIGH  )
      {
        if (New_Com->Check_Error_Status() == 0){
        }
        else
        {
          DEBUG_PRINTLN("BŁĄD AUTOLOKALIZACJI");
          break;
        }
      }
      a.Read_and_Save();
      New_Localization.searching_localization(a.Get_Sensor_Data());
      DEBUG_PRINTLN("HAMUJE LOKOMOTYWE CZARNA");
      New_Com->transmit(false, 0, DIR2, 0, 0, LOC2, true);
      while (New_Com->receive() == false)
      {
        //DEBUG_PRINTLN("Oczekiwanie na dane z lokomotywy");
      }
      DEBUG_PRINT("POZYCJA CZARNEJ LOKOMOTYWY TO: ");
      DEBUG_PRINTLN(New_Localization.Get_Poz_Loc(1));
      //New_Com->Print_Received_Data();

      // PARAMETRY STARTOWE:

      New_State.Set_loc_speed(LOC1, 0);
      New_State.Set_loc_speed(LOC2, 0);
      New_State.Set_loc_mode(LOC1, 1);
      New_State.Set_loc_mode(LOC2, 1);
      New_State.Set_loc_priority(LOC1, true);
      New_State.Set_loc_priority(LOC2, false);

      // Przypisanie lokomotywy 1 do odpowiedniej sekcji na podstawie pozycji

      if ((New_Localization.Get_Poz_Loc(LOC1) >= 8 && New_Localization.Get_Poz_Loc(LOC1) <= 12) || New_Localization.Get_Poz_Loc(LOC1) == 1)
      {
        New_State.Set_loc_section(LOC1, 1);
      }
      else if (New_Localization.Get_Poz_Loc(LOC1) >= 3 && New_Localization.Get_Poz_Loc(LOC1) <= 6)
      {
        New_State.Set_loc_section(LOC1, 2);
      }
      else if (New_Localization.Get_Poz_Loc(LOC1) == 2 || New_Localization.Get_Poz_Loc(LOC1) == 7)
      {
        New_State.Set_loc_section(LOC1, 3);
      }
      else
      {
        DEBUG_PRINTLN("BLAD OKRESLENIA POZYCJI");
      }

      // Przypisanie lokomotywy 2 do odpowiedniej sekcji na podstawie pozycji

      if ((New_Localization.Get_Poz_Loc(LOC2) >= 8 && New_Localization.Get_Poz_Loc(LOC2) <= 12) || New_Localization.Get_Poz_Loc(LOC2) == 1)
      {
        New_State.Set_loc_section(LOC2, 1);
      }
      else if (New_Localization.Get_Poz_Loc(LOC2) >= 3 && New_Localization.Get_Poz_Loc(LOC2) <= 6)
      {
        New_State.Set_loc_section(LOC2, 2);
      }
      else if (New_Localization.Get_Poz_Loc(LOC2) == 2 || New_Localization.Get_Poz_Loc(LOC2) == 7)
      {
        New_State.Set_loc_section(LOC2, 3);
      }
      else
      {
        DEBUG_PRINTLN("BLAD OKRESLENIA POZYCJI");
      }

      // Sprawdzenie poprawnosci sekcji oraz rozpoczęcie automatycznego cyklu

      if ( New_State.Get_loc_section(LOC1) == New_State.Get_loc_section(LOC2) )
      {
        DEBUG_PRINTLN("NIEPOPRAWNE WARUNKI STARTOWE: OBIE LOKOMOTYWY NA TYM SAMYM TORZE");
      }
      else
      {
        this->section_autopilot(LOC1);
        switch ( New_State.Get_loc_section(LOC1) )
        {
          case 1:

            if ( New_Localization.Get_Poz_Loc(LOC2) >= 2 && New_Localization.Get_Poz_Loc(LOC2) <= 5 )
            {
              New_State.Set_loc_priority(LOC2, true);
              this->section_autopilot(LOC2);
            }
            else if ( New_Localization.Get_Poz_Loc(LOC2) == 6 || New_Localization.Get_Poz_Loc(LOC2) == 7 )
            {
              New_State.Set_loc_waiting_status(LOC2, true);
              New_State.Set_loc_mode(LOC2, 8);
            }
            else
            {
              //DEBUG_PRINTLN("BLAD PRZY STARCIE DRUGIEJ LOKOMOTYWY");
            }
            break;

          case 2:

            if ( (New_Localization.Get_Poz_Loc(LOC2) >= 8 && New_Localization.Get_Poz_Loc(LOC2) <= 12) || New_Localization.Get_Poz_Loc(LOC2) ==  2 )
            {
              New_State.Set_loc_priority(LOC2, true);
              this->section_autopilot(LOC2);
            }
            else if ( New_Localization.Get_Poz_Loc(LOC2) == 1 )
            {
              New_State.Set_loc_priority(LOC2, this->check_free_section (LOC2));
              this->section_autopilot(LOC2);
            }
            else if ( New_Localization.Get_Poz_Loc(LOC2) == 7 )
            {
              New_State.Set_loc_waiting_status(LOC2, true);
              New_State.Set_loc_mode(LOC2, 8);
            }
            else
            {
              //DEBUG_PRINTLN("BLAD PRZY STARCIE DRUGIEJ LOKOMOTYWY");
            }
            break;

          case 3:

            if ( (New_Localization.Get_Poz_Loc(LOC2) >= 8 && New_Localization.Get_Poz_Loc(LOC2) <= 12) || (New_Localization.Get_Poz_Loc(LOC2) >= 3 && New_Localization.Get_Poz_Loc(LOC2) <= 5) )
            {
              New_State.Set_loc_priority(LOC2, true);
              this->section_autopilot(LOC2);
            }
            else if ( New_Localization.Get_Poz_Loc(LOC2) == 1 )
            {
              New_State.Set_loc_priority(LOC2, this->check_free_section (LOC2));
              this->section_autopilot(LOC2);
            }
            else if ( New_Localization.Get_Poz_Loc(LOC2) == 6 )
            {
              New_State.Set_loc_waiting_status(LOC2, true);
              New_State.Set_loc_mode(LOC2, 8);
            }
            else
            {
              //_DEBUG_PRINTLN("BLAD PRZY STARCIE DRUGIEJ LOKOMOTYWY");
            }
            break;
        }
      }
      DEBUG_PRINTLN("Rozpoczynam program");
    }
    void section_autopilot(const byte &a)
    {
      DEBUG_PRINTLN("************************************");
      DEBUG_PRINT("MODE LOC: ");
      DEBUG_PRINTLN(New_State.Get_loc_mode(a));
      DEBUG_PRINT("PRIORYTET: ");
      DEBUG_PRINTLN(New_State.Get_loc_priority(a));

      // SEKCJA 1

      if ( New_State.Get_loc_section(a) == 1 )
      {
        if ((New_Localization.Get_Poz_Loc(a) == 1 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 8 && New_Localization.Get_dir_loc(a) == DIR2 ) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if (  New_State.Get_loc_priority(a) == true )
              {
                New_State.Set_loc_mode(a, 2);
              }
              else
              {
                break;
              }

            case 2:

              New_Com->transmit(false, SLOW, New_Localization.Get_dir_loc(a), 5, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, SLOW);
              New_State.Set_loc_priority(a, false);
              New_State.Set_loc_mode(a, 3);
              break;

            case 3:

              New_State.Set_loc_priority(a, false);
              break;
          }
        }
        else if ((New_Localization.Get_Poz_Loc(a) == 8 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 1 && New_Localization.Get_dir_loc(a) == DIR2 ) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if (  New_State.Get_loc_priority(a) == true )
              {
                New_State.Set_loc_mode(a, 2);
              }
              else
              {
                break;
              }

            case 2:

              if (  New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_priority(a, false);
                New_State.Set_loc_section(a, section_memory);
                section_memory = 0 ;
              }
              else
              {
                New_Com->transmit(false, 0, New_Localization.Get_dir_loc(a), 0, 0, a, true);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, 0);
                New_State.Set_loc_mode(a, 1);
              }
              break;
          }
        }
        else if ((New_Localization.Get_Poz_Loc(a) == 9 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 12 && New_Localization.Get_dir_loc(a) == DIR2 ) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if (  New_State.Get_loc_priority(a) == true )
              {
                New_State.Set_loc_mode(a, 2);
              }
              else
              {
                break;
              }

            case 2:

              New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_priority(a, this->check_free_section (a));
              break;
          }
        }
        else if ((New_Localization.Get_Poz_Loc(a) == 10 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 11 && New_Localization.Get_dir_loc(a) == DIR2 ) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if (  New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_priority(a, false);
                New_State.Set_loc_mode(a, 2);
              }
              break;

            case 2:

              New_State.Set_loc_mode(a, 3);

            case 3:

              New_Com->transmit(false, 0, New_Localization.Get_dir_loc(a), 0, 0, a, true);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, 0);
              New_State.Set_loc_priority(a, false);
              stop_time[a] = millis();
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 4);
              break;
          }
        }
        else if ((New_Localization.Get_Poz_Loc(a) == 11 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 10 && New_Localization.Get_dir_loc(a) == DIR2 ) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_priority(a, false);
                stop_time[a] = millis() + 500;
                New_State.Set_loc_waiting_status(a, true);
                New_State.Set_loc_mode(a, 5);
              }
              break;

            case 2:

              stop_time[a] = millis();
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_priority(a, false);
              New_State.Set_loc_mode(a, 5);
              break;

            case 5: // trzeba chyba zmienic na case 3

              New_Com->transmit(false, SLOW, New_Localization.Get_dir_loc(a), 5, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, SLOW);
              New_State.Set_loc_priority(a, false);
              New_State.Set_loc_mode(a, 3);
              break;
          }
        }
        else if ((New_Localization.Get_Poz_Loc(a) == 12 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 9 && New_Localization.Get_dir_loc(a) == DIR2 ) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_priority(a, false);
                New_State.Set_loc_mode(a, 2);
              }
              break;

            case 2:

              New_State.Set_loc_mode(a, 3);

            case 3:

              New_Com->transmit(false, 0, New_Localization.Get_dir_loc(a), 0, 0, a, true);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, 0);
              New_State.Set_loc_priority(a, false);
              stop_time[a] = millis();
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 4);
              break;
          }
        }
      }

      // SEKCJA 2

      else if ( New_State.Get_loc_section(a) == 2 )
      {
        if ((New_Localization.Get_Poz_Loc(a) == 3 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 6 && New_Localization.Get_dir_loc(a) == DIR2) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_priority(a, false);
                New_State.Set_loc_waiting_status(a, false);
                New_State.Set_loc_mode(a, 2);
                New_State.Set_loc_section(a, 1);
              }
              break;

            case 2:

              New_State.Set_loc_mode(a, 9);

            case 3:

              New_State.Set_loc_mode(a, 9);

            case 9:

              New_Com->transmit(false, 0, New_Localization.Get_dir_loc(a), 0, 0, a, true);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, 0);
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 8);
              break;

            case 10:

              New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, FAST);
              New_State.Set_loc_priority(a, false);
              New_State.Set_loc_waiting_status(a, false);
              New_State.Set_loc_mode(a, 2);
              New_State.Set_loc_section(a, 1);
              break;
          }
        }
        if ((New_Localization.Get_Poz_Loc(a) == 4 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 5 && New_Localization.Get_dir_loc(a) == DIR2) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_priority(a, false);
                stop_time[a] = millis();
                New_State.Set_loc_waiting_status(a, true);
                New_State.Set_loc_mode(a, 6);
              }
              break;

            case 2:

              New_State.Set_loc_mode(a, 3);

            case 3:

              New_Com->transmit(false, 0, New_Localization.Get_dir_loc(a), 0, 0, a, true);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, 0);
              New_State.Set_loc_priority(a, false);
              stop_time[a] = millis();
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 4);
              break;

            case 6:

              New_Com->transmit(false, SLOW, New_Localization.Get_dir_loc(a), 5, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, SLOW);
              New_State.Set_loc_priority(a, false);
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 9);
              break;
          }
        }
        if ((New_Localization.Get_Poz_Loc(a) == 5 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 4 && New_Localization.Get_dir_loc(a) == DIR2) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_State.Set_loc_mode(a, 3);
              }
              else
              {
                break;
              }

            case 2:

              New_State.Set_loc_mode(a, 3);

            case 3:

              New_Com->transmit(false, SLOW, New_Localization.Get_dir_loc(a), 10, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, SLOW);
              New_State.Set_loc_priority(a, false);
              break;
          }
        }
        if ((New_Localization.Get_Poz_Loc(a) == 6 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 3 && New_Localization.Get_dir_loc(a) == DIR2) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_State.Set_loc_mode(a, 3);
              }
              else
              {
                break;
              }

            case 2:

              New_State.Set_loc_mode(a, 3);

            case 3:

              New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, FAST);
              New_State.Set_loc_priority(a, false);
              New_State.Set_loc_mode(a, 2);
              start_delay[a] = true;
              delay_time = millis();
          }
        }
      }

      // SEKCJA 3

      else if ( New_State.Get_loc_section(a) == 3 )
      {
        if ((New_Localization.Get_Poz_Loc(a) == 2 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 7 && New_Localization.Get_dir_loc(a) == DIR2) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_priority(a, false);
                New_State.Set_loc_waiting_status(a, false);
                New_State.Set_loc_mode(a, 2);
                New_State.Set_loc_section(a, 1);
              }
              break;

            case 2:

              New_State.Set_loc_mode(a, 9);

            case 3:

              New_State.Set_loc_mode(a, 9);

            case 9:

              New_Com->transmit(false, 0, New_Localization.Get_dir_loc(a), 0, 0, a, true);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, 0);
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 8);
              break;

            case 10:

              New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, FAST);
              New_State.Set_loc_priority(a, false);
              New_State.Set_loc_waiting_status(a, false);
              New_State.Set_loc_mode(a, 2);
              New_State.Set_loc_section(a, 1);
              break;
          }
        }
        if ((New_Localization.Get_Poz_Loc(a) == 7 && New_Localization.Get_dir_loc(a) == DIR1) || (New_Localization.Get_Poz_Loc(a) == 2 && New_Localization.Get_dir_loc(a) == DIR2) )
        {
          switch ( New_State.Get_loc_mode(a) )
          {
            case 1:

              if ( New_State.Get_loc_priority(a) == true )
              {
                New_Com->transmit(false, FAST, New_Localization.Get_dir_loc(a), 10, 0, a, false);
                error_time = millis();
                while (New_Com->receive() == false)
                {
                  if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                  {
                    New_Com->reconnect();
                    break;
                  }
                }
                stop_time[a] = millis();
                New_State.Set_loc_speed(a, FAST);
                New_State.Set_loc_waiting_status(a, true);
                New_State.Set_loc_mode(a, 7);
                start_delay[a] = true;
                delay_time = millis();
              }
              break;
            case 2:

              stop_time[a] = millis();
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 7);
              start_delay[a] = true;
              delay_time = millis();
              break;

            case 7:

              New_Com->transmit(false, SLOW, New_Localization.Get_dir_loc(a), 5, 0, a, false);
              error_time = millis();
              while (New_Com->receive() == false)
              {
                if ( ((millis() - error_time) > 5) || (New_Com->Check_Error_Status() != 0) )
                {
                  New_Com->reconnect();
                  break;
                }
              }
              New_State.Set_loc_speed(a, SLOW);
              New_State.Set_loc_waiting_status(a, true);
              New_State.Set_loc_mode(a, 9);
              break;
          }
        }
      }
      else
      {
        DEBUG_PRINT(" BLAD: SEKCJA 0 NIE ISTNIEJE! ");
      }
      DEBUG_PRINT("MODE LOC: ");
      DEBUG_PRINTLN(New_State.Get_loc_mode(a));
      DEBUG_PRINT("PRIORYTET: ");
      DEBUG_PRINTLN(New_State.Get_loc_priority(a));
    }

    void automatic_control_algorithm(Sensor_Data &a)
    {

      // ZMIANA ODCZYTU Z CZUJNIKOW - OKRESLENIE REAKCJI LOKOMOTYWY

      if (STATE)
      {
        a.Read_and_Save();
        New_Localization.searching_localization(a.Get_Sensor_Data());
        if ( New_Localization.Get_change_status(LOC1) == true )
        {

          this->section_autopilot(LOC1);
          New_Localization.Print_position(LOC1);
        }
        if ( New_Localization.Get_change_status(LOC2) == true )
        {
          this->section_autopilot(LOC2);
          New_Localization.Print_position(LOC2);
        }
        STATE = false;
      }

      // WARUNKOWE TRYBY JAZDY - OPÓŹNIENIA,POSTOJE, ROZPATRYWANIE ZAPYTAŃ I WYSTAWIANIE POZWOLEŃ

      if ( New_State.Get_loc_waiting_status(LOC1) == true )
      {
        switch (New_State.Get_loc_mode(LOC1))
        {
          case 4: // ZATRZYMANIE LOKOMOTYWY NA STACJI LUB PRZED PRZEJAZDEM - POSTÓJ CZASOWY
            if ((millis() - stop_time[LOC1]) > 2500 )
            {
              New_State.Set_loc_priority(LOC1, true);
              New_State.Set_loc_mode(LOC1, 1);
              New_State.Set_loc_waiting_status(LOC1, false);
              this->section_autopilot(LOC1);
            }
            break;
          case 5: // ZWOLNIENIE LOKOMOTYWY PRZY DOJEZDZIE DO STACJI LUB PRZEJAZDU
            if ((millis() - stop_time[LOC1]) > 6000 )
            {
              New_State.Set_loc_waiting_status(LOC1, false);
              this->section_autopilot(LOC1);
            }
            break;
          case 6: // ZWOLNIENIE LOKOMOTYWY PRZED DOJAZDEM DO ZWROTNICY (SEKCJA 2)
            if ((millis() - stop_time[LOC1]) > 2000 )
            {
              New_State.Set_loc_waiting_status(LOC1, false);
              this->section_autopilot(LOC1);
            }
            break;
          case 7: // ZWOLNIENIE LOKOMOTYWY PRZED DOJAZDEM DO ZWROTNICY (SEKCJA 3)
            if ((millis() - stop_time[LOC1]) > 8000 )
            {
              New_State.Set_loc_waiting_status(LOC1, false);
              this->section_autopilot(LOC1);
            }
            break;
          case 8: // ROZPATRZENIE PROŚBY O POZWOLENIE NA ROZPOCZĘCIE JAZDY Z POZYCJI 2,3,6,7 ( POJAWIA SIĘ WOLNA SEKCJA )
            if ( check_ride_permission[LOC1] == true )
            {
              if (New_Localization.Get_dir_loc(LOC1) == DIR1)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC1), true);
              }
              else if (New_Localization.Get_dir_loc(LOC1) == DIR2)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC1), false);
              }
              else
              {
                DEBUG_PRINTLN("Bledna wartosc kierunku lokomotywy")
              }
              New_State.Set_loc_priority(LOC1, true);
              New_State.Set_loc_mode(LOC1, 1);
              this->section_autopilot(LOC1);
              New_State.Set_loc_waiting_status(LOC1, false);
              check_ride_permission[LOC1] = false;
            }
            break;
          case 9: // ROZPATRZENIE PROŚBY O POZWOLENIE NA SWOBODNY PRZEJAZD ( WOLNA SEKCJA, BRAK POTRZEBY HAMOWANANIA )
            if ( check_ride_permission[LOC1] == true )
            {
              if (New_Localization.Get_dir_loc(LOC1) == DIR1)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC1), true);
              }
              else if (New_Localization.Get_dir_loc(LOC1) == DIR2)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC1), false);
              }
              else
              {
                DEBUG_PRINT("Bledna wartosc kierunku lokomotywy")
              }
              New_State.Set_loc_waiting_status(LOC1, false);
              New_State.Set_loc_mode(LOC1, 10);
              check_ride_permission[LOC1] = false;
            }
            break;
        }
      }
      if ( New_State.Get_loc_waiting_status(LOC2) == true )
      {
        switch (New_State.Get_loc_mode(LOC2))
        {
          case 4: // ZATRZYMANIE LOKOMOTYWY NA STACJI LUB PRZED PRZEJAZDEM - POSTÓJ CZASOWY
            if ((millis() - stop_time[LOC2]) > 2500 )
            {
              New_State.Set_loc_priority(LOC2, true);
              New_State.Set_loc_waiting_status(LOC2, false);
              New_State.Set_loc_mode(LOC2, 1);
              this->section_autopilot(LOC2);
            }
            break;
          case 5: // ZWOLNIENIE LOKOMOTYWY PRZY DOJEZDZIE DO STACJI LUB PRZEJAZDU
            if ((millis() - stop_time[LOC2]) > 3000 )
            {
              New_State.Set_loc_waiting_status(LOC2, false);
              this->section_autopilot(LOC2);
            }
            break;
          case 6: // ZWOLNIENIE LOKOMOTYWY PRZED DOJAZDEM DO ZWROTNICY (SEKCJA 2)
            if ((millis() - stop_time[LOC2]) > 2000 )
            {
              New_State.Set_loc_waiting_status(LOC2, false);
              this->section_autopilot(LOC2);
            }
            break;
          case 7: // ZWOLNIENIE LOKOMOTYWY PRZED DOJAZDEM DO ZWROTNICY (SEKCJA 3)
            if ((millis() - stop_time[LOC2]) > 3000 )
            {
              New_State.Set_loc_waiting_status(LOC2, false);
              this->section_autopilot(LOC2);
            }
            break;
          case 8: // OTRZYMANIE POZWOLENIA NA ROZPOCZĘCIE JAZDY Z POZYCJI 2,3,6,7 ( POJAWIA SIĘ WOLNA SEKCJA )
            if ( check_ride_permission[LOC2] == true )
            {
              if (New_Localization.Get_dir_loc(LOC2) == DIR1)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC2), true);
              }
              else if (New_Localization.Get_dir_loc(LOC2) == DIR2)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC2), false);
              }
              else
              {
                //_DEBUG_PRINTLN("Bledna wartosc kierunku lokomotywy")
              }
              New_State.Set_loc_priority(LOC2, true);
              New_State.Set_loc_mode(LOC2, 1);
              this->section_autopilot(LOC2);
              New_State.Set_loc_waiting_status(LOC2, false);
              check_ride_permission[LOC2] = false;
            }
            break;
          case 9: // ROZPATRZENIE PROŚBY O POZWOLENIE NA SWOBODNY PRZEJAZD ( WOLNA SEKCJA, BRAK POTRZEBY HAMOWANANIA )
            if ( check_ride_permission[LOC2] == true )
            {
              if (New_Localization.Get_dir_loc(LOC2) == DIR1)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC2), true);
              }
              else if (New_Localization.Get_dir_loc(LOC2) == DIR2)
              {
                New_steering.steering_control(New_State.Get_loc_section(LOC2), false);
              }
              else
              {
                DEBUG_PRINT("Bledna wartosc kierunku lokomotywy")
              }
              New_State.Set_loc_waiting_status(LOC2, false);
              New_State.Set_loc_mode(LOC2, 10);
              check_ride_permission[LOC2] = false;
            }
            break;
        }
      }

      // ZWŁOKA CZASOWA WYNIKAJĄCA Z DŁUGOŚCI SKŁADU (CZEKANIE NA PRZEJAZD WSZYSTKICH WAGONÓW )
      if ( New_State.Get_loc_section(LOC1) != 1)
      {
        if ( (start_delay[LOC1] == true) && ((millis() - delay_time) > 1000) )
        {
          DEBUG_PRINTLN("Wystawiam pozwolenie!!!");
          check_ride_permission[LOC2] = true;
          start_delay[LOC1] = false;
        }
      }
      if ( New_State.Get_loc_section(LOC2) != 1)
      {
        if ( (start_delay[LOC2] == true) && ((millis() - delay_time) > 1000) )
        {
          DEBUG_PRINTLN("Wystawiam pozwolenie!!!");
          check_ride_permission[LOC1] = true;
          start_delay[LOC2] = false;
        }
      }

    }

    // SPRAWDZENIE ZAJETOSCI SEKCJI I USTAWIENIE ZWROTNICY

    bool check_free_section (const byte &a)
    {
      if ( a == LOC1 )
      {
        if (New_State.Get_loc_section(LOC2) == 2 )
        {
          section_memory = 3;
        }
        else if (New_State.Get_loc_section(LOC2) == 3 )
        {
          section_memory = 2;
        }

        if (New_Localization.Get_dir_loc(LOC1) == DIR1)
        {
          New_steering.steering_control(section_memory, false);
        }
        else if (New_Localization.Get_dir_loc(LOC1) == DIR2)
        {
          New_steering.steering_control(section_memory, true);
        }
        else
        {
          DEBUG_PRINTLN("Bledna wartosc kierunku lokomotywy")
        }
      }
      else if ( a == LOC2 )
      {
        if (New_State.Get_loc_section(LOC1) == 2 )
        {
          section_memory = 3;
        }
        else if (New_State.Get_loc_section(LOC1) == 3 )
        {
          section_memory = 2;
        }

        if (New_Localization.Get_dir_loc(LOC2) == DIR1)
        {
          New_steering.steering_control(section_memory, false);
        }
        else if (New_Localization.Get_dir_loc(LOC2) == DIR2)
        {
          New_steering.steering_control(section_memory, true);
        }
        else
        {
          DEBUG_PRINTLN("Bledna wartosc kierunku lokomotywy")
        }
      }
      return true;
    }
  private:
    RF_Communication *New_Com;
    Localization New_Localization;
    Loc_State New_State;
    Steering_control New_steering;
    unsigned long stop_time[2];      // Odliczanie czasu postoju
    unsigned long delay_time;        // Odczekanie na przejazd całego składu
    unsigned long error_time;        // Odliczanie czasu oczekiwania na odpowiedz
    bool start_delay[2];             // Trigger do rozpoczęcia odliczania
    bool check_ride_permission[2];   // Pozwolenie na rozpoczęcie jazdy
    byte section_memory;             // Zapamiętana sekcja, do późniejszej zmiany



};

Sensor_Data New_Sensor(0x20, 0, On_Interrupt , 0x21, 1, On_Interrupt );
Processing_Data New_Process;

//--------------FUNKCJA SETUP-------------------------------------------------------------------------------------------------------------------------------
void setup() {
  DEBUG_START(115200);
  while (!DEBUG_STATUS) {
  }
  delay(5000);
  DEBUG_PRINTLN("ROZPOCZYNAM PROGRAM");
  New_Sensor.start();
  New_Process.auto_localization(New_Sensor);

}

//--------------FUNCKJA LOOP-----------------------------------------------------------------------------------------------------------------------------
void loop() {
  New_Process.automatic_control_algorithm(New_Sensor);
}
