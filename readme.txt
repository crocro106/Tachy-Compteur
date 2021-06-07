TACHYMETRE - FREQUENCEMETRE - COMPTEUR - MESURE DE POSITION

Ce projet vise à créer un tachymètre - frequencemetre - compteur d'impulsion ou de sens / distance à partir d'un arduino uno ou nano - d'un écran hitachi 2 lignes de 16 caractères et de quatre boutons

Une simulation est disponible sur tinkercad 

https://www.tinkercad.com/things/dooZBakVply-tachymetre-frequencemetre-compteur-encodeur-

https://www.tinkercad.com/things/dooZBakVply-tachymetre-frequencemetre-compteur-encodeur-/editel

le (les) capteurs sont supposés envoyer des impulsions entre 0 et 5V - adapter l'électronique si besoin. 

Pour un tachymètre / fréquencemètre / compteur seul un capteur relié à la patte 2 de l'arduino suffit

Pour la mesure de position il faut deux capteurs en quadrature - relative en x1 et x2, parfaite en x4 

un diviseur est réglable pour s'adapter au nombre d'impulsions par tour

en cas de capteur à front un peu incertain, un antirebond est programmé - il est réglable

les mesures d'antirebond des boutons poussoirs sont réglées "en dur" dans le code, mais peuvent être modifiées avant compilation. 

A des fins de tests il comporte à la marge une régulation de vitesse d'un petit moteur à CC à partir d'un pont en H L293D - la porte non (74HC00) permet d'injecter /PWM sur in2 et PWM sur in1 du L239D (il y a surement un moyen de faire faire ça à l'arduino mais je n'ai pas cherché). Un potentiometre permet de faire varier la vitesse. 

Vous pouvez aussi envoyer PWM sur la broche ena du L293 et sélectionner le sens par in1 et in2 (il vous faudra utiliser deux ports supplémentaires, il doit rester tout juste ce qu'il faut) Toutefois ce mode de commande est relativement inefficace si on veut réguler des vitesses lentes. 

retours et améliorations appréciées
