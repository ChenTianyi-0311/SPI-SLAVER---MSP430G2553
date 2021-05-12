#include <msp430.h>

 

int miliseconds; // time between the two front (front montant et front descendent)
int distance; // distance between obstacle and ultrason
long sensor; // sensor = distance * 58
int is_obstacle = 0;
int is_red = 0;

 

/**
* init_obstacle_detection
*
*/
void init_cap()
{
CCTL0 = CCIE; // Interruption CCR0 activée
CCR0 = 100000; // 1ms à 1mhz
TACTL = TASSEL_2 + MC_1; // SMCLK, upmode
P1IFG = 0x00; //effacer tous les flags d'interruption
P1DIR |= 0x01; // P1.0 output pour LED
P1OUT &= ~0x01; // P1.0 low power mode
P1DIR |= BIT6;
P1OUT &= ~BIT6;
}

 

/**
* init_time
*/
void init_time()
{
if (CALBC1_1MHZ == 0xFF)
{
while(1);
}
DCOCTL = 0;
BCSCTL1 = CALBC1_1MHZ;
DCOCTL = CALDCO_1MHZ;
}

 

/**
* init_spi
* initialize spi module of MSP430
*/
void init_spi()
{
__delay_cycles(250);

 

P1SEL |= (BIT5 | BIT6 | BIT7);
P1SEL2 |= (BIT5 + BIT6 + BIT7);

 

UCB0CTL1 |= UCSWRST;

 

IFG2 &= ~(UCB0TXIFG | UCB0RXIFG);
UCB0CTL0 |= (UCMODE_0 | UCSYNC | UCMSB);
UCB0CTL0 &= ~(UCCKPH | UCCKPL | UC7BIT | UCMST);
UCB0CTL1 |= UCSSEL_2;

 

UCB0CTL1 &= ~UCSWRST;
IE2 |= UCB0RXIE;
}

 

/**
* init_moteur
* initialize moteur of vehicle
*/
void init_moteur(void)
{
// I/O mode
P2SEL |= BIT2|BIT5;
P2SEL2 &= ~(BIT2|BIT5);
P2DIR |= BIT2|BIT5;
//timer1
TA1CTL = 0x0210;
TA1CCR0 = 20000;
//output mode
TA1CCTL1 = 0x00E0;
TA1CCTL2 = 0x00E0;
TA1CCR1 = 1400; //2.2 moteur
__delay_cycles(2000000);
TA1CCR2 = 1500;//2.5(5v 1150-1800 7.2v 1250 -2250)

 

}
void set_speed(void)
{
if(is_obstacle == 0 && is_red == 0)
{
TA1CCR1 = 1400;
__delay_cycles(600000);
TA1CCR1 = 1510;
__delay_cycles(1500000);
}
else
{
TA1CCR1 = 1400;
}
}
int main(void)
{
WDTCTL = WDTPW | WDTHOLD; // stop watchdog timer
P1DIR |= BIT0;
P1OUT |= BIT0;
init_time();
init_spi();
init_cap();
init_moteur();
__delay_cycles(100000);
__enable_interrupt();
while(1)
{
set_speed();
P1IE &= ~0x01; // désactiver l'interruption
P1DIR |= 0x02; // trigger pin en output
P1OUT |= 0x02; // générer une impulsion
__delay_cycles(10); // pour 10us
P1OUT &= ~0x02; // arrêter l'impulsion
P1DIR &= ~0x04; // make pin P1.2 input (ECHO)
P1IFG = 0x00; // effacer le flag juste au cas où quelque chose se passerait avant
P1IE |= 0x04; // activer l'interruption sur ECHO pin
P1IES &= ~0x04; // front montant sur la broche ECHO
__delay_cycles(30000); // délai de 30ms (après ce temps, l'écho expire s'il n'y a pas d'objet détecté)
distance = sensor/58; // conversion de la longueur ECHO en cm
if(distance <= 100 && distance != 0)
{
P1OUT |= 0x01; //allumer la LED si la distance est inférieure à 10 cm et si la distance n'est pas de 0.
TA1CCR1 = 1400;
is_obstacle = 1;
}
else
{
P1OUT &= ~0x01;
TA1CCR1 = 1500;
is_obstacle = 0;
}
}
return 0;
}

 

/**
* INTERRUPT Function
*/
#pragma vector = USCIAB0RX_VECTOR
__interrupt void receive(void){
switch(UCB0RXBUF){
case 0x31:
TA1CCR2 = 1300; // turn right
P1OUT |= BIT0;
break;
case 0x32:
TA1CCR2 = 1700; // turn left
P1OUT &= ~BIT0;
break;
case 0x33:
TA1CCR2 = 1500; // reposition
break;
case 0x34:
//TA1CCR1 = 1400; // reposition
is_red = 1;
break;
case 0x35:
// TA1CCR1 = 1500; // reposition
is_red = 0;
break;
default:
break;
}
}

 

#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
if(P1IFG&0x04) //y a-t-il une interruption en attente?
{
if(!(P1IES&0x04)) // est-ce le front montant?
{
TACTL|=TACLR; // efface le timer A
miliseconds = 0;
P1IES |= 0x04; //front descendant
}
else
{
sensor = (long)miliseconds*1000 + (long)TAR; //calcul de la longueur ECHO
}
P1IFG &= ~0x04; //effacer le flag
}
}

 


#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
miliseconds++;
}
