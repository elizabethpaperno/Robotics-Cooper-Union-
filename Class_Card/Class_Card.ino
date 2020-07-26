class Card {
  char suit;
  int value;

  public:
  Card(char, int); 
  char getSuit (){
    return suit;
  }
  int getValue(){
    return value;
  }
  
};
Card :: Card(char s, int v) {
  suit = s;
  value = constrain(v, 1, 13);
}

class Deck {
  Card* cards[52]; 
  int topcard;
  public:
  Deck ();
  
  
};

Deck :: Deck() {
  for( int i = 1; i <= 13; i++){
    cards[i] = new Card ('H',i);
  }
  for( int j = 1; j <= 13; j++){
    cards[j] = new Card ('C',j);
  }
  for( int k = 1; k <= 13; k++){
    cards[k] = new Card ('D',k);
  }
  for( int h = 1; h <= 13; h++){
    cards[h] = new Card ('D',h);
  }
}
  topCard = 0
  for (int i = 0; i < 52; i++){
    
  }
    
void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
