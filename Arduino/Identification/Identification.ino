
// Identification d'un moteur à courant continu

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

// Résistances du circuit d'amplification de courant
float R1 = 1000.;
float R2 = 10000.;
// Résistance de mesure
float Rmes = 0.1;

void setup() {
  Serial.begin(115200);
  
  // Codeur incrémental
  pinMode(codeurArriereGauchePinA, INPUT_PULLUP);      // entrée digitale pin A codeur
  pinMode(codeurArriereGauchePinB, INPUT_PULLUP);      // entrée digitale pin B codeur

  // Moteur à courant continu arrière gauche
  pinMode(codeurArriereGauchePinA, OUTPUT);
  pinMode(pwmMoteurArriereGauche, OUTPUT);
  pinMode(mesurePWM_MoteurArriereGauche, INPUT);

  // Modification du prescaler de l'ADC: division par 16 de la fréquence du micro-contrôleur
  // au lieu de 128 par défaut, pour diminuer le temps entre deux mesures analogiques
  bitSet(ADCSRA, ADPS2);
  bitClear(ADCSRA, ADPS1);
  bitClear(ADCSRA, ADPS0);

  FlexiTimer2::set(1, 1/10000., isrt); // résolution timer = 0.1 ms
  FlexiTimer2::start();

  // On ne génère pas un PWM car la période est trop petite
  digitalWrite(pwmMoteurArriereGauche, HIGH);
}

void loop() {
  if ((nbData > (arraysize/2)-1) && (over == 0)) {
    over = 1;
    
    // Stop the motors
    digitalWrite(pwmMoteurArriereGauche, LOW);
    // Arrêt de l'interruption Timer 2
    //FlexiTimer2::stop();
  }
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
    
    // A chaque changement de niveau de tension sur le pin A du codeur,
    // on exécute la fonction GestionInterruptioncodeurArriereGauchePinA (définie à la fin du programme)
    enableInterrupt(codeurArriereGauchePinA, GestionInterruptioncodeurArriereGauchePinA, CHANGE);
  
    // Calcul courant moteur, sur interruption PWM
    enableInterrupt(mesurePWM_MoteurArriereGauche, calculCourantMoteur, RISING);

    // Redémarrage du Timer 2 pour un échantillonnage moins rapide
    // (pour la mesure de la vitesse du moteur)
    delay(100);
    ticksCodeur = 0;
    //FlexiTimer2::set(10, 1/1000., isrt); // résolution timer = 0.1 ms
    //FlexiTimer2::start();
    over = 3;

    // Consigne de vitesse moteur (correspond à environ 6V)
    analogWrite(pwmMoteurArriereGauche, 195);    
  }
  else if ((over == 3) && (nbData > arraysize-2)) {
    over = 4;
    // Arrêt du moteur
    analogWrite(pwmMoteurArriereGauche, 0);
    
//    Serial.println("nbData: ");
//    Serial.println(nbData);
//    Serial.println("im: ");
    // On met arbitrairement le dernier point à 0 (ou plutôt à 512,
    // qui correspond au 0 physique) pour que l'identification
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
}

/*** Interrupt routine ***/
void isrt() {

  if (over == 0) {
    // Compte-tenu de la structure du driver moteur utilisé,
    // le courant qui passe dans un moteur est la somme des deux courants mesurés
    adc = analogRead(mesureCourant_MoteurArriereGauche);// + analogRead(mesureCourant_MoteurArriereDroit);
    tval[nbData] = micros();
    courant[nbData] = (Vref * ((float)adc)/1024.) / ((1 + R2/R1) * Rmes);
    nbData++;
  }
  else if (over == 3) {
    // Pour la vitesse, une exécution sur 100 (échantillonage à 10 ms au lieu de 100 µs)
    adc = analogRead(mesureCourant_MoteurArriereGauche);// + analogRead(mesureCourant_MoteurArriereDroit);
    courantsomme += (Vref * ((float)adc)/1024.) / ((1 + R2/R1) * Rmes);
    compteurMesureCourant++;
    if (idecim == 100) {
      idecim = 0;
      tval[nbData] = micros();
      
      // Nombre de ticks codeur depuis la dernière fois: correspond à l'angle de rotation du moteur pendant la période d'échantillonnage
      // Ce nombre est mis à jour par les fonctions GestionInterruptioncodeurArriereGauchePinA et GestionInterruptioncodeurArriereGauchePinB,
      // exécutées à chaque interruption due à une impulsion sur la voie A ou B du codeur incrémental
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
  // Calcul du courant moyen sur la période PWM haut / PWM bas écoulée. Il y a donc un décalage de 2 ms
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

