/******************************************************************************
Programme d'identification des moteurs du robotu T-Quad, disponible à l'adresse:
http://boutique.3sigma.fr/12-robots

Auteur: 3Sigma
Version 1.1.1 - 15/12/2016
*******************************************************************************/


// Inclusion d'une bibliothèque permettant de lire et d'écrire plus rapidement sur les entrées-sorties digitales.
// Télécharger à l'adresse http://www.3sigma.fr/telechargements/digitalWriteFast.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
#include <digitalWriteFast.h>

// Inclusion d'une bibliothèque permettant de gérer les interruptions externes
// et le "PinChange Interrupts"
// Télécharger à l'adresse http://www.3sigma.fr/telechargements/EnableInterrupt.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
#include <EnableInterrupt.h>

// Inclusion d'une bibliothèque permettant l'exécution à cadence fixe
// d'une partie du programme. Télécharger à l'adresse http://www.3sigma.fr/telechargements/FlexiTimer2.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
// Pour plus de détails, voir les pages (en anglais): 
// http://www.arduino.cc/playground/Main/FlexiTimer2
// https://github.com/wimleers/flexitimer2
#include <FlexiTimer2.h>

// Inclusion d'une bibliothèque permettant de gérer l'écran OLED.
// Télécharger à l'adresse http://www.3sigma.fr/telechargements/U8glib.zip
// et décompresser dans le sous-répertoire « libraries » de votre installation Arduino
#include "U8glib.h"

// Taille des tableaux de mesure
#define arraysize 150

// Définitions et déclarations pour le codeur incrémental
#define codeurArriereGauchePinA 3
#define codeurArriereGauchePinB 9
volatile long ticksCodeur = 0;

// Définition pour le moteur arrière gauche du robot T-Quad (http://boutique.3sigma.fr/12-robots)
#define directionMoteurArriereGauche  7
#define pwmMoteurArriereGauche  6
#define mesurePWM_MoteurArriereGauche  12
#define mesureCourant_MoteurArriereGauche 1
#define mesureCourant_MoteurArriereDroit 0

int adc = 0;      // Mesure analogique
float courant[arraysize];      // Tableau des courants moyens du moteur
int omega[arraysize];      // Tableau des mesures de vitesse du moteur
unsigned long tval[arraysize];      // Tableau des mesures de temps
unsigned long firstTime;
long nbData = 0;
int over = 0;
int idecim = 0;

float courantsomme = 0.0;
long compteurMesureCourant = 0L;

float Vref = 5.;
float dt = 0.01;
float VBat;

// Résistances du circuit d'amplification de courant
float R1 = 1000.;
float R2 = 10000.;
// Résistance de mesure
float Rmes = 0.1;

// Déclaration de l'objet écran OLED
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_DEV_0|U8G_I2C_OPT_NO_ACK|U8G_I2C_OPT_FAST);  // Fast I2C / TWI 

// Fonction d'affichage sur l'écran
void draw(void) {
  // Définition de la font
  u8g.setFont(u8g_font_unifont);

  // Affichage d'une estimation de la tension d'alimentation des moteurs au cours de l'essai
  u8g.setPrintPos(0, 10);
  u8g.print("Tension d'alim");
  u8g.setPrintPos(0, 25);
  u8g.print("pour essai");
  u8g.setPrintPos(0, 40);
  u8g.print("en courant: ");
  u8g.setPrintPos(0, 55); 
  String strVAlim = String(VBat,2);
  strVAlim += " V";
  u8g.print(strVAlim);
}

void setup() {
  Serial.begin(115200);
  
  // Codeur incrémental
  pinMode(codeurArriereGauchePinA, INPUT_PULLUP);      // entrée digitale pin A codeur
  pinMode(codeurArriereGauchePinB, INPUT_PULLUP);      // entrée digitale pin B codeur

  // Moteur à courant continu arrière gauche
  pinMode(codeurArriereGauchePinA, OUTPUT);
  pinMode(pwmMoteurArriereGauche, OUTPUT);
  pinMode(mesurePWM_MoteurArriereGauche, INPUT);

  // Modification du prescaler de l'ADC: division par 8 de la fréquence du micro-contrôleur
  // au lieu de 128 par défaut, pour diminuer le temps entre deux mesures analogiques
  bitClear(ADCSRA, ADPS2);
  bitSet(ADCSRA, ADPS1);
  bitSet(ADCSRA, ADPS0);

  FlexiTimer2::set(1, 1/10000., isrt); // résolution timer = 0.1 ms
  FlexiTimer2::start();

  // On ne génère pas un PWM car la période est trop petite
  digitalWrite(pwmMoteurArriereGauche, HIGH);
}

void loop() {
  // Première phase: mesure du courant alors que le moteur
  // ne tourne pas encore
  if ((nbData > (arraysize/2)-1) && (over == 0)) {
    over = 1;
    
    // Arrêt des moteurs
    digitalWrite(pwmMoteurArriereGauche, LOW);
  }
  // Fin de la première phase: transmission des mesures de courant
  // via la liaison série
  else if (over == 1) {
    over = 2;

    // On met arbitrairement le premier et le dernier point à 0
    // pour que l'identification puisse se faire correctement dans MyViz
    courant[0] = 0.0;
    courant[nbData - 1] = 0.0;
    firstTime = tval[0];
    for (int i=0; i<arraysize/2; i++) {
      Serial.print(((float)(tval[i] - firstTime))/1000,3);
      Serial.print(",");
      Serial.print(courant[i], 3);
      Serial.print(",");
      Serial.println((float)omega[i], 3);
    }

    // Activation d'une interruption pour que, à chaque changement de niveau de tension sur le pin A du codeur,
    // on exécute la fonction GestionInterruptioncodeurArriereGauchePinA (définie à la fin du programme)
    enableInterrupt(codeurArriereGauchePinA, GestionInterruptioncodeurArriereGauchePinA, CHANGE);
  
    // Calcul courant moteur, sur interruption PWM
    enableInterrupt(mesurePWM_MoteurArriereGauche, calculCourantMoteur, RISING);

    // Reinitialisations pour la phase suivante
    delay(100);
    ticksCodeur = 0;
    over = 3;

    // Lecture de la tension batterie
    VBat = 0.;
    // La première lecture peut être fausse
    while (VBat < 6.) {
      VBat = 3. * (Vref * (float)analogRead(5) / 1024.);
    }

    // Seconde phase: mesure de la vitesse du moteur et du courant
    // pendant la montée en vitesse
    // Consigne de vitesse moteur à 6V
    analogWrite(pwmMoteurArriereGauche, (uint8_t)(1530/VBat)); // 1530 = 6 * 255
  }
  // Fin de la seconde phase: transmission des vitesses et mesures de courant
  // via la liaison série
  else if ((over == 3) && (nbData > arraysize-2)) {
    over = 4;
    // Arrêt du moteur
    analogWrite(pwmMoteurArriereGauche, 0);
    
    // On met arbitrairement les dernier points à 0 pour que l'identification
    // puisse se faire correctement dans MyViz
    courant[nbData - 1] = 0.0;
    omega[nbData - 1] = 0;
    for (int i=(arraysize/2); i<arraysize-1; i++) {
      Serial.print(((float)(tval[i] - firstTime))/1000,3);
      Serial.print(",");
      Serial.print(courant[i],3);
      Serial.print(",");
      // Calcul de la vitesse de rotation. C'est le nombre d'impulsions converti en radian, divisé par la période d'échantillonnage
      Serial.println((-2*(2.*3.141592*((float)omega[i]))/1200.)/dt,2);
    }
    
    // Arrêt de l'interruption Timer 2
    FlexiTimer2::stop();

  }
  else if (over == 4) {
    // Afichage sur l'écran OLED
    delay(500);
    u8g.firstPage();  
    do {
      draw();
    } while( u8g.nextPage() );
  }
}

// Routine exécutéee sur interruption Timer 2
void isrt() {
  // Première phase
  if (over == 0) {
    // Mesure du courant et stockage dans un tableau
    // Compte-tenu de la structure du driver de puissance utilisé, le courant est la somme
    // de celui passant dans les résistances arrière droite et arrière gauche
    adc = analogRead(mesureCourant_MoteurArriereGauche) + analogRead(mesureCourant_MoteurArriereDroit);
    tval[nbData] = micros();
    courant[nbData] = (Vref * ((float)adc)/1024.) / ((1 + R2/R1) * Rmes);
    nbData++;
  }
  // Seconde phase
  else if (over == 3) {
    // Compte-tenu de la structure du driver de puissance utilisé, le courant est la somme
    // de celui passant dans les résistances arrière droite et arrière gauche
    adc = analogRead(mesureCourant_MoteurArriereGauche) + analogRead(mesureCourant_MoteurArriereDroit);
    courantsomme += (Vref * ((float)adc)/1024.) / ((1 + R2/R1) * Rmes);
    compteurMesureCourant++;
    // Pour la vitesse, une exécution sur 100 interruptions Timer 2 (échantillonage à 10 ms au lieu de 100 µs)
    if (idecim == 100) {
      idecim = 0;
      tval[nbData] = micros();
      
      // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
      // Ce nombre est mis à jour par la fonction GestionInterruptioncodeurArriereGauchePinA
      // exécutées à chaque interruption due à une impulsion sur la voie A du codeur incrémental
      omega[nbData] = ticksCodeur;
  
      // Une fois lu, ce nombre est remis à 0 pour pouvoir l'incrémenter de nouveau sans risquer de débordement de variable
      ticksCodeur = 0;
    
      nbData++;
    }
    else {
      idecim++;
    }
  }


}

void calculCourantMoteur() {
  // Calcul du courant moyen sur la période PWM haut / PWM bas écoulée.
  if (compteurMesureCourant > 0) {
    courant[nbData] = courantsomme / (float)compteurMesureCourant;
    compteurMesureCourant = 0L;
  }
  else {
    courant[nbData] = 0.;
  }
  courantsomme = 0.0;
}

void GestionInterruptioncodeurArriereGauchePinA() {  
  // Routine de service d'interruption attachée à la voie A du codeur incrémental
  // On utilise la fonction digitalReadFast2 de la librairie digitalWriteFast
  // car les lectures doivent être très rapides pour passer le moins de temps possible
  // dans cette fonction (appelée de nombreuses fois, à chaque impulsion de codeur)
  if (digitalReadFast2(codeurArriereGauchePinA) == digitalReadFast2(codeurArriereGauchePinB)) {
    ticksCodeur++;
  }
  else {
    ticksCodeur--;
  }
}

