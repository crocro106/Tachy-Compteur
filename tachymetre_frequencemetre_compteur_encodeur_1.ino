/* Tachymetre - Frequencemetre - compteur d'impulsions - position en tours

Licence : creative commons CC-BY-SA : 
	disponible en suivant ce lien
	https://creativecommons.org/licenses/by-sa/3.0/deed.fr
	qui vous autorise à réutiliser et modifier ce programme à condition
    de citer l'original, d'indiquer qu'il y a eu modifications le cas échéant, 
    de placer le résultat sous la même licence
    
Note de responsabilité
CE PROJET ET LE CODE ASSOCIE EST FOURNI TEL QUEL SANS GARANTIE D’AUCUNE SORTE, 
EXPLICITE OU IMPLICITE, Y COMPRIS, MAIS SANS S’Y LIMITER, TOUTE GARANTIE IMPLICITE 
DE QUALITÉ MARCHANDE OU D’ADAPTATION À UN USAGE PARTICULIER

ce projet permet la mesure sur l'entrée "2" (interruption "0") soit d'une
fréquence de rotation (Hz) ou en (tr/min), soit d'un nombre de tours
soit d'une position en tours à condition d'utiliesr un encodeur en quadrature 
avec le second canal branché sur l'entrée "3" (interruption "0").

le fonctionnement est le suivant
 au démarrage la fonction par défaut est choisie par l'initialisation de la 
 variable "fonction" : 
 	0 : tachymètre, 
 	1, fréquencemetre, 
    2 : compteur, 
    3 : position encodeur en 1x, 
    4 : position encodeur en 2x, 
    5 : position encodeur en 4x)
 les fonctions "6" et "7" sont réservées aux réglages
 	6 : réglage du nombre d'impulsion par tour (ou CPR de l'encodeur, 
    	le multiplicateur est pris en compte dans la fonction
    7 : réglage du temps d'antirebond  en µs : durée minimale de l'état haut
        de la voie A pour être prise en compte comme une impulsion
        ce délai n'est pas actif pour les fonctions 4 et 5
  Attention : ne pas choisir d'initialiser la variable "fonction" sur "6" ou "7"
  (ça bouclerait sur le réglage)
  
  Le clavier est composé de 4 touches :
  	SET (à droite sur le simulateur)
  	ESC (à gauche sur le simulateur)
  	UP (en haut sur le simulateur)
  	DOWN (en bas sur le simulateur)
  
  Le réglage des variables fonctionne comme suit
  
  lorsqu'une fonction en cours : 
  	appui sur "set" : on entre dans le menu
    appui sur "esc" : on remet le compteur à 0 (pour les fonctions 2,3,4,5)
    les touches "up" et "down" sont inactives
   
   une fois entrée dans le menu on navigue avec "up" et "down" 
   "esc" ne conserve pas le choix et revient à la fonction précédente
   "set" valide le choix et active la fonction choisie
   		si la fonction est "6" ou "7" (réglage diviseur et antirebond)
        le réglage est possible, 
        	un appui court sur "up" : on incrémente de 1
            un appui long sur "up" : on incrémente de 10
            un appui simultané long sur "up" et sur "set" : on incrémente de 100
            un appui court sur "down" : on décrémente de 1
            un appui long sur "down" : on décrémente de 10
            un appui simultané long sur "down" et sur "set" : on décrémente de 100
           le réglage est dans les limites fixées au moment de la compilation  (voir plus bas)            
          un appui sur "set" valide le réglage, sur "esc" revient à la valeur précédemment choisie 
        on revient ensuite à la fonction préalablement choisie
        
        NOTE : les fonctions des broches A0, 6, le 74HC00, le L293D, 
               le potentiomètre, les condensateurs, l'alimentation, le moteur
               et le code associé ne sont là qu'à des fin de test
               ainsi que le code associé (les deux première commandes de la fonction loop())
               et les déclarations d'E/S associées
               
        VOUS DEVEZ ADAPTER VOTRE CHAINE DE MESURE ET VOS CAPTEURS pour assurer un signal 0-5V sur les
        broches 2 et 3 du shield arduino (un trigger de schmidt n'est pas forcément de trop !) 
        
        NOTE : en l'état ça rentre sur un NANO en ATMEGA 168
*/
//définition  des constantes et librairies utiles
// gestion de l'écran LCD
#include <LiquidCrystal.h> // on importe la bibliothèque
#define RS 7
#define E 8
#define DB4 9
#define DB5 10
#define DB6 11
#define DB7 12

// pour tester
#define pwmtest 6
#define led 13

// gestion du clavier
#define set A2
#define esc A4
#define up A5
#define down A3
#define tps_rebond_poussoir 10 // temps de rebond d'un poussoir en ms
#define tps_appui_long 150// temps de prise en compte d'un appui long

// pour les entrées compteur 
#define voie_A 2 //voie A du capteur
#define voie_B 3 // voie B du capteur
#define int_A 0 //n° interruption liée à la voie A
#define int_B 1 //n° interruption liée à la voie B
#define divmin 1 // nombre mini d'impulsions par tour
#define divmax 10000 // nombre maxi d'impulsions par tour
#define tmin 1
#define tmax 100000

// variables pour le fonctionnement du tachymètre, compteur
// ATTENTION : les temps sont en microsecondes
volatile long t = 0; // temps courant (µs)
volatile long tp = 0; // instant de l'impulsion précédente (µs)
volatile long nbimp = 0; //position actuelle
volatile long dt; //delta t entre deux positions pour le calcul de N
unsigned short diviseur = 60; // nombre d'impulsions par tour - ne pas descendre en dessous de 1
unsigned long tVoieA = 30; // temps d'antirebond en µs sur la voie A pour les modes tachymètres, fréquencemetre, compteur et encodeur x1 
unsigned long tcp =0; // instant du calcul précédent, à utiliser si le deltat est trop court
long nbtrs=0; // position actuelle en tours 
long nbimpold =0; //position au tour précédent de calcul
long N =0;// frequence de rotation

// variables pour la gestion du clavier
long talong = 0; //temps depuis le dernier appui long, utilisé pour mesurer un délai d'appui long
long tclavier = 0; //temps depuis le dernier appui clavier pris en compte
byte etatswitch = 0;//etat du clavier lors du dernier scan clavier pris en compte : le bit 0 : bouton set, le bit 1 : bouton esc, le bit 2 : bouton up, le bit 3 : bouton down, le bit 4 : bouton set long, le bit 5 : bouton esc long, bit 6 bouton up long, bit 7 : bouton down long
byte etatclavier = 0;//etat pris en compte du clavier (même symbolique)

// gestion de l'affichage
String tafn = String(-32000000); // chaine à afficher
int lafn = 1; // longueur de la chaine à afficher
int lafnold = 1; // longueur de la chaine précédente à afficher
float f=0;
float position_trs=0;
  
// déclaration de l'écran
LiquidCrystal monEcran(RS, E, DB4, DB5, DB6, DB7); // on crée l'objet écran
unsigned int nbcarmicro=0; // définition du symbole micro non présent par défaut
byte symmicro[8] = { // déclaration d’un tableau de 8 octets
  B00000, // définition de chaque octet au format binaire
  B00000, // 1 pour pixel affiché – 0 pour pixel éteint
  B01001, // les 3 bits de poids forts ne sont pas écrits car inutiles
  B01001,
  B01001,
  B01001,
  B01111,
  B10000,};

// structure du menu principal
const char* str_menu[]={"Tachymetre","Frequencemetre","Compteur","Encodeur x1","Encodeur x2","Encodeur x4","regl. diviseur","reg. antirebond"};
byte fonction = 0;
byte fonction_prec = 0;
const byte nb_fonction=8;

// fonctions de mesure et de calcul des paramètres

// fonction d'interruptions utilisée pour le tachymètre, le fréquencemetre, le compteur et l'encodeux x1
void delta() 
{
  t = micros(); // on récupère l'instant d'appel de l'interruption 
  if (tp >=t)   // si l'instant d'appel est antérieur à l'appel précédent à l'interruption.... problème de causalité
    {tp = t;}   // on a reculé dans le temps, on doit avoir un "débordement de pile"; on ne calculera pas de delta t cette fois on rafraichit tp jusqu'au prochain calcul
  if (t-tp>tVoieA) //au moins trebond microsecondes écoulées depuis le dernier top, on considère qu'il n'y a pas "rebond"
    {  
    dt = t-tp; // le delta t est calculé en microsecondes 
    tp = t;    // on synchronise nos instants pour pouvoir calculer le prochain delta t
    if (fonction<3) {nbimp++;} // si tachy fréquencemetre ou compteur on incrémente la position
    if (fonction==3)// si encodeur attention au sens !
    {
      if (digitalRead(3) == true) {nbimp++;} else {nbimp--;}
    }
  }
}
// fonction d'interruption utilisée pour l'encodeur x2 et x4
void encodeurchannelA() {
  if (digitalRead(3)==digitalRead(2)) // si le sens est mauvais faire un !=
    {nbimp++;} 
    else 
    {nbimp--;}
    }
// fonction d'interruption utilisée pour l'encodeur x4
void encodeurchannelB() {
  if (digitalRead(3)==digitalRead(2)) // si le sens n'est pas le bon faire un != sur cette fonction et sur la précédente !
    {nbimp--;}
    else 
    {nbimp++;}
    }

// fonction de calcul de la fréquence de rotation
void calculN() // calcul de N en tr/min à partir de la période dt mesurée en microsecondes
{
  long tact = micros();
  if (dt>0) // le delta est non nul
  {
    N= 60*1000000/diviseur/dt; // calcul de la fréquence de rotation 60 sec/min * 1000 000 µs/S divisé par le diviseur (impulsions/tr) et par le delta t
  }
  else // deltat trop rapide 
  {
    if (tact!=tcp) {N = 60*1000000*(nbimp-nbimpold)/diviseur/(tact-tcp);}
  } 
}

// fonction de gestion des touches du clavier, quatre touches "set" "esc" "up" "down" cablées sur A0 à A3
void clavier() // fonction avancée de gestion du clavier, pour les menus
{
  long t=millis(); // le temps courant est mesuré
  byte etatinput=etatswitch & B11110000; // on crée une variable locale pour lire les entrées à comparer avec l'état précédent
  if ((t-talong)>tps_appui_long)
  {
    if (digitalRead(set)==false && bitRead(etatswitch,0)==true) {bitSet(etatinput,4);} else {bitClear(etatinput,4);bitClear(etatswitch,4);}
    if (digitalRead(esc)==false && bitRead(etatswitch,1)==true) {bitSet(etatinput,5);} else {bitClear(etatinput,5);bitClear(etatswitch,5);}
    if (digitalRead(up)==false && bitRead(etatswitch,2)==true){bitSet(etatinput,6);} else {bitClear(etatinput,6);bitClear(etatswitch,6);}
    if (digitalRead(down)==false && bitRead(etatswitch,3)==true) {bitSet(etatinput,7);} else {bitClear(etatinput,7);bitClear(etatswitch,7);}
        talong=t;
  }
  if ((t-tclavier)>tps_rebond_poussoir)
  {
    if (digitalRead(set)==false) {bitSet(etatinput,0);}
    if (digitalRead(esc)==false) {bitSet(etatinput,1);}
    if (digitalRead(up)==false) {bitSet(etatinput,2);}
    if (digitalRead(down)==false) {bitSet(etatinput,3);}
    tclavier=t;
  };
  etatclavier=etatinput&etatswitch;
  etatswitch=etatinput;
}

// fonction d'incrémentation / décrémentation d'un paramètre réglable. Appui court : +-1, long +-10, long + set en même temps : +-100)
long incdec(long val, long min,long max ,byte x, byte y)
{
  long oldval=val; // on sauve la valeur courante en cas de "esc" et pour vérifier si elle a changé (pour ne pas réafficher si pas besoin)
  monEcran.setCursor(x,y); // on se place sur la zone texte
  monEcran.print("       "); // on nettoie la place pour un "long" signé
  String affichnum = String(val);
  monEcran.setCursor(x+6-affichnum.length(),y); // on se replace pour afficher la valeur (aligné à droite)
  monEcran.print(val); // et on affiche la valeur courante
  while (etatclavier>0) // avant de gérer le changement, attendre que le clavier soit au repos
  {
    clavier();
  }
  while (((etatclavier!=1)&& (etatclavier!=16)&&(etatclavier)!=17)&& (etatclavier!=2)&& (etatclavier!=32) && (etatclavier!=34))// tant que "esc" seul ou "set" seul ne sont pas activés
  {		
    clavier();
  	if((etatclavier==4) && (val<max)){val++;} // up court = +1
  	if((etatclavier==68) && (val<(max-9))){val=val+10;} // up long = +10
  	if(((etatclavier==69)||(etatclavier==85)) && (val<(max-99))){val=val+100;} // up long + set ou set long = +100
  	if((etatclavier==8) && (val>min)){val--;} // down court = -1
  	if(etatclavier==136) // down long 
    {
      if (val>(min+9)) {val=val-10;} //= -10 si possible
          else if (val>min) {val--;} // sinon -1, toujours si possible
    } 
    if((etatclavier==137)||(etatclavier==154)) {// down long + set ou set long
      if (val>(min+99)){val=val-100;} // = -100 si possible
      else if (val>(min+9)) {val=val-10;} //= -10 si possible
                else if (val>min) {val--;} // sinon -1, toujours si possible
    }
    if (val!=oldval || val==min) // si changement de valeur affichage à nouveau sinon on ne change pas pour ne pas clignoter
    {	
      String affichnum = String(val);
      monEcran.setCursor(x,y); // on se place sur la zone texte
  	  monEcran.print("        "); // on nettoie la place pour un "long" signé
  	  monEcran.setCursor(x+6-affichnum.length(),y); // on se replace pour afficher la valeur (aligné à droite)
      monEcran.print(val); // et on affiche la valeur courante
    }
   delay(500); // delai pour ne pas incrémenter trop vite si on est en mode incrémentation rapide (à régler)
  }
  if (etatclavier==1){return val;} else {return oldval;}
}

void fonction_diviseur() // fonction 6 : permet de régler la valeur du diviseur : la valeur est à régler au nombre d'impulsions (front montant) par tour du canal A
{
  monEcran.clear(); 
  monEcran.home();
  monEcran.print("nb impulsions/tr");
  diviseur=incdec(diviseur,divmin,divmax,1,1); // on récupère la nouvelle valeur
  monEcran.clear(); 
  monEcran.home();
  monEcran.print("diviseur fixe a :");  
  monEcran.setCursor(0,1);
  monEcran.print(diviseur);
  monEcran.print(" imp/tr");
  while (etatclavier>0) // on attend que le clavier soit au repos
  {
    clavier();
  }
  delay(1000);
  monEcran.clear();
  fonction=fonction_prec;
}

void fonction_antirebond() // fonction 7 : permet de régler la valeur du délai d'antirebond sur le capteur en µs
{
  monEcran.clear(); 
  monEcran.home();
  monEcran.print("temps antirebond");
  monEcran.setCursor(12,1);
  monEcran.write(nbcarmicro);
  monEcran.print("s");
  tVoieA=incdec(tVoieA,tmin,tmax,1,1); // on récupère la nouvelle valeur
  monEcran.clear(); 
  monEcran.home();
  monEcran.print("temps fixe a :");  
  monEcran.setCursor(0,1);
  monEcran.print(tVoieA);
  monEcran.print(" ");
  monEcran.write(nbcarmicro);
  monEcran.print("s");
  while (etatclavier>0) // on attend que le clavier soit au repos
  {
    clavier();
  }
  delay(1000);
  monEcran.clear();
  fonction=fonction_prec;
}

// fonction menu qui renvoie le n° de la fonction choisie 
byte menu()
 {
  int i=fonction; // mise à 0 de la fonction d'affichage
  monEcran.clear(); // on efface l'écran
  monEcran.setCursor(5,0); //(retour au début de l'écran)
  monEcran.print("menu");
  while(etatclavier>0) // on attend que toutes les touches soient relachées pour avancer
    {
      clavier(); 
    }
  delay(500);
  while((etatclavier==0)||(etatclavier>2)) // tant que l'on n'appuie pas sur "set" ou sur "esc"
    {
      monEcran.home(); // on revient au début du clavier
      monEcran.print(">"); // on affiche ">" pour signifier que c'est l'option active
      monEcran.setCursor(1,0); // on se place à coté
      monEcran.print(str_menu[i]); // on affiche l'option en cours
      monEcran.setCursor(1,1); // on se place juste en dessous
      if (i<7)                 // si il y a une option suivante
      {
        monEcran.print(str_menu[i+1]); // on l'affiche
      }
      else 
      {
        monEcran.print(str_menu[0]);  // sinon on affiche la première de la liste
      }
      clavier();
    if (etatclavier>0){monEcran.clear();}
      if (etatclavier==8 || etatclavier==136 || etatclavier==128) // appui court ou long sur "down" uniquement
      {
        etatswitch=0; // RAZ de l'état clavier, action prise en compte
        etatclavier=0; // RAZ de l'état clavier, action prise en compte
        if (i<7) {i++;} else {i=0;}
      }
      if (etatclavier==4 || etatclavier==68 || etatclavier ==64) // appui court ou long sur "up" uniquement
      {
        etatswitch=0; // RAZ de l'état clavier, action prise en compte
        etatclavier=0; // RAZ de l'état clavier, action prise en compte
        if (i==0) {i=7;} else {i--;}
      }
   }
  if ((etatclavier==2) || (etatclavier==34)){i=fonction_prec;}
  monEcran.clear();
  monEcran.setCursor(0,0);
  monEcran.print("choix : ");
  monEcran.print(i);
  monEcran.setCursor(0,1);
  monEcran.print(str_menu[i]);
  while(etatclavier>0) // on attend que toutes les touches soient relachées pour avancer
    {
      clavier();  
    }
  monEcran.clear();
  return i;  
 }

void cpt_ou_pos(int subdiv)
{
   monEcran.setCursor(0, 0);
   position_trs=nbimp/(diviseur*subdiv*1.0);
   monEcran.print(position_trs);
   monEcran.print(" trs       ");
   monEcran.setCursor(0, 1);
   if (fonction==3)
    {
      monEcran.print("compteur Esc=RAZ");
    } 
   else 
    {
      monEcran.print("Position Esc=RAZ");
    } 
   if ((etatclavier==2) || (etatclavier==32) || (etatclavier==34))
    {
      nbimp=0;
      monEcran.setCursor(0,0);
      monEcran.print("            ");              
    }
}
  
void setup() 
{
   monEcran.createChar(nbcarmicro, symmicro); 
   pinMode(set,INPUT_PULLUP);
   pinMode(esc,INPUT_PULLUP);
   pinMode(up,INPUT_PULLUP);
   pinMode(down,INPUT_PULLUP);
   pinMode(pwmtest,OUTPUT);
   pinMode(led,OUTPUT);
   pinMode(voie_A,INPUT_PULLUP);
   pinMode(voie_B,INPUT_PULLUP);
   pinMode(A0,INPUT);
   monEcran.begin(16, 2); //on initialise la communication avec 16 colonnes et deux lignes
   monEcran.clear(); // on efface l'écran
   monEcran.setCursor(0, 0); //on se place en haut à gauche de l'écran
   attachInterrupt(0, delta,RISING);
}
void loop() 
{
  analogWrite(pwmtest,(analogRead(A0)/4));
  clavier();
  if ((etatclavier==1) || (etatclavier==16)|| (etatclavier==17)) // appui court ou long sur "set" 
  {
    fonction_prec=fonction; // on sauvegarde la fonction précédente pour pouvoir y revenir si appui sur "esc" ou sortie des menus de réglage
    fonction=menu(); // on passe à la fonction "menu" pour choisir la fonction  à utiliser
  }  
  if (fonction_prec!=fonction)
  {
    if (fonction==4)
    { 
      attachInterrupt(0, encodeurchannelA,CHANGE);// Attache l'interruption de la pin 2 à la fonction delta
      detachInterrupt(1);
    }
    if (fonction==5);
      { 
      attachInterrupt(0, encodeurchannelA,CHANGE);// Attache l'interruption de la pin 2 à la fonction delta
      attachInterrupt(1, encodeurchannelB,CHANGE);
    }
    if (fonction<4)
    { 
      attachInterrupt(0, delta,RISING);// Attache l'interruption de la pin 2 à la fonction delta
      detachInterrupt(1);
    }
  }
  switch (fonction)
  {
    case 0 :
     calculN();
     lafnold=lafn;
     monEcran.setCursor(0,0);
     monEcran.print("N=");
     monEcran.setCursor(10,0);
     monEcran.print("tr/min");
     tafn = String(N);
     lafn=tafn.length();
     if (lafnold>lafn)            
     {
       monEcran.setCursor(2,0);
       monEcran.print("        ");
     }
     monEcran.setCursor(9-lafn,0);
     monEcran.print(N);
     monEcran.setCursor(0,1);
     monEcran.print(diviseur);
     monEcran.print(" imp/tr");
    break;
    case 1 :
     calculN();
     lafnold=lafn;
     f=N/60;
     monEcran.setCursor(0, 0);
     monEcran.print("f=");
     monEcran.setCursor(14,0);
     monEcran.print("Hz");
     tafn = String(f);
     lafn=tafn.length();
     f=N/60.0;
     if (lafnold!=lafn)            
     {
       monEcran.setCursor(2,0);
       monEcran.print("           ");
     }
     monEcran.setCursor(12-lafn,0);
     monEcran.print(f,2);
     monEcran.setCursor(0,1);
     monEcran.print(diviseur);
     monEcran.print(" imp/tr");
    break;
    case 2 :
     cpt_ou_pos(1);
    break;
    case 3 :
     cpt_ou_pos(1);
    break;
    case 4 :
     cpt_ou_pos(2);
    break;
    case 5 :
     cpt_ou_pos(4);
    break;
    case 6 :
     fonction_diviseur();
    break;
    case 7 :
     fonction_antirebond();
    break;
    default :
     monEcran.setCursor(0, 0);
     monEcran.print("allo Houston");
     monEcran.setCursor(0, 1);
     monEcran.print("on a un probleme");
    break;
  }
}
